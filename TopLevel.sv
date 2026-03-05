// ============================================================================
// TopLevel.sv
// Top-level integration module for FPGA Audio Analyzer
// Integrates SPI receiver, FFT processor, and frequency detector
// ============================================================================

module TopLevel (
    // System Clock and Reset
    input  logic                    sysClk,         // System clock (e.g., 100 MHz)
    input  logic                    sysRst_n,       // Active low reset
    
    // SPI Slave Interface (receive audio)
    input  logic                    spiClk,
    input  logic                    spiMosi,
    output logic                    spiMiso,
    input  logic                    spiCs_n,
    
    // SPI Master Interface (transmit results)
    input  logic                    spiClk_tx,
    output logic                    spiMosi_tx,
    input  logic                    spiCs_n_tx,
    
    // Status LEDs
    output logic                    statusLedReceivingData,
    output logic                    statusLedProcessingFft,
    output logic                    statusLedDetectionComplete,
    
    // Debug Outputs
    output logic [15:0]             debugFreq1,
    output logic [15:0]             debugFreq2,
    output logic [23:0]             debugMag1,
    output logic [23:0]             debugMag2
);

    // ========================================================================
    // Parameters
    // ========================================================================
    localparam int SampleWidth           = 16;
    localparam int BufferDepth           = 2048;
    localparam int BufferAddressWidth    = $clog2(BufferDepth);
    localparam int FftSize               = 1024;
    localparam int FftInputWidth         = 16;
    localparam int FftOutputWidth        = 24;
    
    // ========================================================================
    // Internal Signals - Audio BRAM
    // ========================================================================
    logic [BufferAddressWidth-1:0] bramWriteAddr;
    logic [SampleWidth-1:0]        bramWriteData;
    logic                          bramWriteEn;
    
    logic [BufferAddressWidth-1:0] bramReadAddr;
    logic [SampleWidth-1:0]        bramReadData;
    
    // ========================================================================
    // Internal Signals - Control
    // ========================================================================
    logic                          dataValidFlag;
    logic [31:0]                   wavHeaderInfo;
    
    // ========================================================================
    // Internal Signals - FFT Interface
    // ========================================================================
    logic                          fftEnable;
    logic [FftInputWidth-1:0]      fftDataIn;
    logic                          fftDataValid;
    logic [FftOutputWidth-1:0]     fftDataOutReal;
    logic [FftOutputWidth-1:0]     fftDataOutImag;
    
    // ========================================================================
    // Internal Signals - Frequency Detection
    // ========================================================================
    logic [15:0]                   maxFreq1;
    logic [15:0]                   maxFreq2;
    logic [23:0]                   maxMagnitude1;
    logic [23:0]                   maxMagnitude2;
    logic                          detectionComplete;
    
    // ========================================================================
    // SPI TX Control Signals
    // ========================================================================
    logic [31:0]                   spiTxData;
    logic                          spiTxValid;
    logic [4:0]                    spiTxByteCount;
    logic [7:0]                    spiTxShiftReg;
    logic [2:0]                    spiTxBitCount;
    
    // ========================================================================
    // Instantiate Audio SPI Receiver
    // ========================================================================
    AudioSpiReceiver #(
        .SampleWidth(SampleWidth),
        .BufferDepth(BufferDepth),
        .BufferAddressWidth(BufferAddressWidth)
    ) audio_spi_receiver_inst (
        .sysClk(sysClk),
        .sysRst_n(sysRst_n),
        .spiClk(spiClk),
        .spiMosi(spiMosi),
        .spiMiso(spiMiso),
        .spiCs_n(spiCs_n),
        .dataValidFlag(dataValidFlag),
        .wavHeaderInfo(wavHeaderInfo),
        .bramWriteAddr(bramWriteAddr),
        .bramWriteData(bramWriteData),
        .bramWriteEn(bramWriteEn)
    );
    
    // ========================================================================
    // Dual-Port BRAM for Audio Samples
    // Port A: Write (from SPI Rx)
    // Port B: Read (to FFT)
    // ========================================================================
    DualPortBram #(
        .DataWidth(SampleWidth),
        .AddressWidth(BufferAddressWidth),
        .Depth(BufferDepth)
    ) audio_bram_inst (
        .clk(sysClk),
        .rst_n(sysRst_n),
        
        // Port A: Write
        .addrA(bramWriteAddr),
        .dinA(bramWriteData),
        .weA(bramWriteEn),
        
        // Port B: Read
        .addrB(bramReadAddr),
        .doutB(bramReadData)
    );
    
    // ========================================================================
    // Instantiate Frequency Detector
    // ========================================================================
    FrequencyDetector #(
        .FftSize(FftSize),
        .FftInputWidth(FftInputWidth),
        .FftOutputWidth(FftOutputWidth),
        .SampleWidth(SampleWidth),
        .BufferAddressWidth(BufferAddressWidth)
    ) frequency_detector_inst (
        .sysClk(sysClk),
        .sysRst_n(sysRst_n),
        .dataValidFlag(dataValidFlag),
        .wavHeaderInfo(wavHeaderInfo),
        .bramReadAddr(bramReadAddr),
        .bramReadData(bramReadData),
        .fftEnable(fftEnable),
        .fftDataIn(fftDataIn),
        .fftDataValid(fftDataValid),
        .fftDataOutReal(fftDataOutReal),
        .fftDataOutImag(fftDataOutImag),
        .maxFreq1(maxFreq1),
        .maxFreq2(maxFreq2),
        .maxMagnitude1(maxMagnitude1),
        .maxMagnitude2(maxMagnitude2),
        .detectionComplete(detectionComplete)
    );
    
    // ========================================================================
    // Altera FFT Megafunction IP Instantiation
    // (User must generate this IP core in Quartus)
    // ========================================================================
    // altera_fft_ip fft_ip_inst (
    //     .clk(sysClk),
    //     .reset(~sysRst_n),
    //     .sink_valid(fftEnable),
    //     .sink_data({{8{fftDataIn[FftInputWidth-1]}}, fftDataIn}), // Sign extend to 24-bit
    //     .source_valid(fftDataValid),
    //     .source_real(fftDataOutReal),
    //     .source_imag(fftDataOutImag)
    // );
    
    // ========================================================================
    // SPI Master TX - Send Results Back to Host
    // ========================================================================
    always_ff @(posedge sysClk or negedge sysRst_n) begin
        if (!sysRst_n) begin
            spiTxValid <= 1'b0;
            spiTxData <= 32'h0;
            spiTxShiftReg <= 8'h0;
            spiTxBitCount <= 3'h0;
            spiTxByteCount <= 5'h0;
            spiMosi_tx <= 1'b0;
        end else begin
            // When detection is complete, queue results for transmission
            if (detectionComplete && !spiTxValid) begin
                spiTxValid <= 1'b1;
                // Format: [31:16] Freq1, [15:0] Freq2
                spiTxData <= {maxFreq1, maxFreq2};
            end
            
            // Shift out one bit per SPI clock
            if (spiTxValid && spiClk_tx && !spiCs_n_tx) begin
                spiMosi_tx <= spiTxShiftReg[7];
                spiTxShiftReg <= {spiTxShiftReg[6:0], 1'b0};
                spiTxBitCount <= spiTxBitCount + 1'b1;
                
                if (spiTxBitCount == 3'h7) begin
                    spiTxByteCount <= spiTxByteCount + 1'b1;
                    if (spiTxByteCount == 5'h3) begin
                        spiTxValid <= 1'b0;
                    end
                    // Load next byte
                    case (spiTxByteCount)
                        5'h0: spiTxShiftReg <= spiTxData[31:24];
                        5'h1: spiTxShiftReg <= spiTxData[23:16];
                        5'h2: spiTxShiftReg <= spiTxData[15:8];
                        5'h3: spiTxShiftReg <= spiTxData[7:0];
                        default: spiTxShiftReg <= 8'h0;
                    endcase
                end
            end
        end
    end
    
    // ========================================================================
    // Status LEDs
    // ========================================================================
    assign statusLedReceivingData    = dataValidFlag;
    assign statusLedProcessingFft    = fftEnable;
    assign statusLedDetectionComplete = detectionComplete;
    
    // ========================================================================
    // Debug Outputs
    // ========================================================================
    assign debugFreq1 = maxFreq1;
    assign debugFreq2 = maxFreq2;
    assign debugMag1  = maxMagnitude1;
    assign debugMag2  = maxMagnitude2;

endmodule