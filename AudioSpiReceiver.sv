// ============================================================================
// AudioSpiReceiver.sv
// SPI Slave interface for receiving WAV audio data from master
// Stores PCM samples in dual-port BRAM for FFT processing
// ============================================================================

module AudioSpiReceiver #(
    parameter int SampleWidth = 16,           // 16-bit PCM samples
    parameter int BufferDepth = 2048,         // BRAM depth for samples
    parameter int BufferAddressWidth = 11     // log2(2048)
) (
    // Clock and Reset
    input  logic                    sysClk,
    input  logic                    sysRst_n,
    
    // SPI Slave Interface
    input  logic                    spiClk,
    input  logic                    spiMosi,      // Master Out, Slave In
    output logic                    spiMiso,      // Master In, Slave Out
    input  logic                    spiCs_n,      // Chip Select (active low)
    
    // Control Signals
    output logic                    dataValidFlag,
    output logic [31:0]             wavHeaderInfo, // Sample rate in [31:16], Channels in [15:8], BitWidth in [7:0]
    
    // BRAM Write Interface
    output logic [BufferAddressWidth-1:0] bramWriteAddr,
    output logic [SampleWidth-1:0]        bramWriteData,
    output logic                          bramWriteEn
);

    // ========================================================================
    // SPI State Machine
    // ========================================================================
    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_RECEIVE_HEADER,
        STATE_RECEIVE_DATA,
        STATE_COMPLETE
    } SpiState_t;
    
    SpiState_t currentState, nextState;
    
    // ========================================================================
    // Internal Signals
    // ========================================================================
    logic [7:0]                 spiShiftReg;
    logic [4:0]                 bitCounter;
    logic [2:0]                 byteCounter;
    logic [31:0]                headerBuffer;
    logic [BufferAddressWidth-1:0] writeAddress;
    logic [SampleWidth-1:0]     sampleBuffer;
    
    // Synchronized SPI signals
    logic                       spiCs_n_sync1, spiCs_n_sync2;
    logic                       spiClk_sync1, spiClk_sync2;
    logic                       spiClk_prev;
    logic                       spiClk_rising;
    
    // ========================================================================
    // Clock Domain Crossing - CDC (from SPI clock to sys clock)
    // ========================================================================
    always_ff @(posedge sysClk or negedge sysRst_n) begin
        if (!sysRst_n) begin
            spiCs_n_sync1  <= 1'b1;
            spiCs_n_sync2  <= 1'b1;
            spiClk_sync1   <= 1'b0;
            spiClk_sync2   <= 1'b0;
            spiClk_prev    <= 1'b0;
        end else begin
            spiCs_n_sync1  <= spiCs_n;
            spiCs_n_sync2  <= spiCs_n_sync1;
            spiClk_sync1   <= spiClk;
            spiClk_sync2   <= spiClk_sync1;
            spiClk_prev    <= spiClk_sync2;
        end
    end
    
    assign spiClk_rising = (~spiClk_prev & spiClk_sync2);
    
    // ========================================================================
    // SPI Receive Shift Register (operates in SPI clock domain)
    // ========================================================================
    always_ff @(posedge spiClk or negedge spiCs_n) begin
        if (!spiCs_n) begin
            spiShiftReg <= 8'h00;
            bitCounter <= 5'h00;
        end else if (spiClk) begin
            spiShiftReg <= {spiShiftReg[6:0], spiMosi};
            bitCounter <= bitCounter + 1'b1;
        end
    end
    
    // SPI output - send back acknowledgment or status
    assign spiMiso = spiShiftReg[7];
    
    // ========================================================================
    // State Machine - Synchronized to System Clock
    // ========================================================================
    always_ff @(posedge sysClk or negedge sysRst_n) begin
        if (!sysRst_n) begin
            currentState <= STATE_IDLE;
            byteCounter <= 3'h0;
            headerBuffer <= 32'h0;
            writeAddress <= {BufferAddressWidth{1'b0}};
            dataValidFlag <= 1'b0;
            bramWriteEn <= 1'b0;
            wavHeaderInfo <= 32'h0;
        end else begin
            currentState <= nextState;
            bramWriteEn <= 1'b0; // Default: no write
            
            if (spiClk_rising && !spiCs_n_sync2) begin
                case (currentState)
                    STATE_IDLE: begin
                        byteCounter <= 3'h0;
                        headerBuffer <= 32'h0;
                        writeAddress <= {BufferAddressWidth{1'b0}};
                        dataValidFlag <= 1'b0;
                    end
                    
                    STATE_RECEIVE_HEADER: begin
                        // WAV header format: [31:16] SampleRate, [15:8] Channels, [7:0] BitWidth
                        headerBuffer <= {headerBuffer[23:0], spiShiftReg};
                        byteCounter <= byteCounter + 1'b1;
                        
                        if (byteCounter == 3'h3) begin
                            wavHeaderInfo <= {headerBuffer[23:0], spiShiftReg};
                        end
                    end
                    
                    STATE_RECEIVE_DATA: begin
                        // Receive 16-bit PCM samples (MSB first)
                        sampleBuffer <= {sampleBuffer[7:0], spiShiftReg};
                        byteCounter <= byteCounter + 1'b1;
                        
                        if (byteCounter == 3'h1) begin
                            bramWriteData <= {sampleBuffer[7:0], spiShiftReg};
                            bramWriteEn <= 1'b1;
                            bramWriteAddr <= writeAddress;
                            writeAddress <= writeAddress + 1'b1;
                            
                            if (writeAddress == BufferDepth - 1) begin
                                dataValidFlag <= 1'b1;
                            end
                        end
                        byteCounter <= (byteCounter == 3'h1) ? 3'h0 : (byteCounter + 1'b1);
                    end
                    
                    STATE_COMPLETE: begin
                        dataValidFlag <= 1'b1;
                    end
                endcase
            end
            
            // CS deassert handling
            if (spiCs_n_sync2 && !spiCs_n_sync1) begin
                if (currentState == STATE_RECEIVE_HEADER) begin
                    if (byteCounter >= 3'h3) begin
                        // Header reception complete
                        nextState <= STATE_RECEIVE_DATA;
                    end
                end else if (currentState == STATE_RECEIVE_DATA) begin
                    nextState <= STATE_COMPLETE;
                end
            end
        end
    end
    
    // ========================================================================
    // Next State Logic
    // ========================================================================
    always_comb begin
        nextState = currentState;
        
        case (currentState)
            STATE_IDLE: begin
                if (!spiCs_n_sync2) begin
                    nextState = STATE_RECEIVE_HEADER;
                end
            end
            
            STATE_RECEIVE_HEADER: begin
                // Transition handled by CS deassert
            end
            
            STATE_RECEIVE_DATA: begin
                // Transition handled by CS deassert or buffer full
            end
            
            STATE_COMPLETE: begin
                if (spiCs_n_sync2) begin
                    nextState = STATE_IDLE;
                end
            end
        endcase
    end

endmodule