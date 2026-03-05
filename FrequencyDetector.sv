// ============================================================================
// FrequencyDetector.sv
// FFT-based frequency detection module
// Identifies 2 highest-powered frequencies from audio samples
// ============================================================================

module FrequencyDetector #(
    parameter int FftSize = 1024,
    parameter int FftInputWidth = 16,
    parameter int FftOutputWidth = 24,
    parameter int SampleWidth = 16,
    parameter int BufferAddressWidth = 11,
    parameter int MaxFrequencies = 2
) (
    // Clock and Reset
    input  logic                        sysClk,
    input  logic                        sysRst_n,
    
    // Input Control Signals
    input  logic                        dataValidFlag,
    input  logic [31:0]                 wavHeaderInfo,
    
    // BRAM Read Interface
    output logic [BufferAddressWidth-1:0] bramReadAddr,
    input  logic [SampleWidth-1:0]        bramReadData,
    
    // FFT IP Interface
    output logic                        fftEnable,
    output logic [FftInputWidth-1:0]    fftDataIn,
    input  logic                        fftDataValid,
    input  logic [FftOutputWidth-1:0]   fftDataOutReal,
    input  logic [FftOutputWidth-1:0]   fftDataOutImag,
    
    // Output: 2 Maximum Frequencies
    output logic [15:0]                 maxFreq1,      // Highest frequency
    output logic [15:0]                 maxFreq2,      // 2nd highest frequency
    output logic [23:0]                 maxMagnitude1,
    output logic [23:0]                 maxMagnitude2,
    output logic                        detectionComplete
);

    // ========================================================================
    // Internal State Machine
    // ========================================================================
    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_LOAD_FFT,
        STATE_RUN_FFT,
        STATE_FIND_PEAKS,
        STATE_COMPLETE
    } DetectorState_t;
    
    DetectorState_t currentState, nextState;
    
    // ========================================================================
    // Frequency Detection Parameters
    // ========================================================================
    logic [31:0]                sampleRate;
    logic [15:0]                frequencyResolution; // Hz per FFT bin
    
    // ========================================================================
    // FFT Processing Signals
    // ========================================================================
    logic [BufferAddressWidth-1:0] fftReadAddr;
    logic [9:0]                    fftBinCounter;
    logic [9:0]                    fftBinIndex;
    
    logic [23:0]                magnitude;
    logic [23:0]                peakMagnitude1, peakMagnitude1_next;
    logic [9:0]                 peakBin1, peakBin1_next;
    logic [23:0]                peakMagnitude2, peakMagnitude2_next;
    logic [9:0]                 peakBin2, peakBin2_next;
    
    // ========================================================================
    // Magnitude Calculation (from Real and Imaginary parts)
    // magnitude = sqrt(real^2 + imag^2)
    // ========================================================================
    logic [47:0] realSquared;
    logic [47:0] imagSquared;
    logic [48:0] magnitudeSquared;
    
    assign realSquared = fftDataOutReal * fftDataOutReal;
    assign imagSquared = fftDataOutImag * fftDataOutImag;
    assign magnitudeSquared = realSquared + imagSquared;
    
    // Simplified square root using bit shift (approximation for high magnitudes)
    // For better accuracy, use dedicated SQRT IP or iterative Newton-Raphson
    always_comb begin
        if (magnitudeSquared[48:40] != 9'h0) begin
            magnitude = magnitudeSquared[48:24];
        end else if (magnitudeSquared[40:32] != 9'h0) begin
            magnitude = magnitudeSquared[40:16];
        end else begin
            magnitude = magnitudeSquared[32:8];
        end
    end
    
    // ========================================================================
    // Extract Sample Rate from WAV Header
    // ========================================================================
    assign sampleRate = {wavHeaderInfo[31:16], 16'h0000}; // wavHeaderInfo[31:16] = sample rate in kHz
    
    // Frequency Resolution: sampleRate / FFT_SIZE (in Hz)
    // This is simplified; actual implementation would use divider IP
    always_comb begin
        case (sampleRate[31:16])
            16'd48: frequencyResolution = 16'd47;   // 48000 / 1024 ≈ 47 Hz/bin
            16'd44: frequencyResolution = 16'd43;   // 44100 / 1024 ≈ 43 Hz/bin
            16'd96: frequencyResolution = 16'd94;   // 96000 / 1024 ≈ 94 Hz/bin
            default: frequencyResolution = 16'd48;
        endcase
    end
    
    // ========================================================================
    // State Machine
    // ========================================================================
    always_ff @(posedge sysClk or negedge sysRst_n) begin
        if (!sysRst_n) begin
            currentState <= STATE_IDLE;
            fftReadAddr <= {BufferAddressWidth{1'b0}};
            fftEnable <= 1'b0;
            fftBinCounter <= 10'h0;
            fftBinIndex <= 10'h0;
            peakMagnitude1 <= 24'h0;
            peakBin1 <= 10'h0;
            peakMagnitude2 <= 24'h0;
            peakBin2 <= 10'h0;
            detectionComplete <= 1'b0;
            maxFreq1 <= 16'h0;
            maxFreq2 <= 16'h0;
            maxMagnitude1 <= 24'h0;
            maxMagnitude2 <= 24'h0;
        end else begin
            currentState <= nextState;
            peakMagnitude1 <= peakMagnitude1_next;
            peakBin1 <= peakBin1_next;
            peakMagnitude2 <= peakMagnitude2_next;
            peakBin2 <= peakBin2_next;
            
            case (currentState)
                STATE_IDLE: begin
                    detectionComplete <= 1'b0;
                    fftReadAddr <= {BufferAddressWidth{1'b0}};
                    fftBinCounter <= 10'h0;
                    peakMagnitude1 <= 24'h0;
                    peakMagnitude2 <= 24'h0;
                    peakBin1 <= 10'h0;
                    peakBin2 <= 10'h0;
                end
                
                STATE_LOAD_FFT: begin
                    fftEnable <= 1'b1;
                    if (fftReadAddr < FftSize) begin
                        fftReadAddr <= fftReadAddr + 1'b1;
                    end else begin
                        fftEnable <= 1'b0;
                    end
                end
                
                STATE_RUN_FFT: begin
                    // Wait for FFT computation to complete
                    // FFT IP will signal completion
                end
                
                STATE_FIND_PEAKS: begin
                    if (fftDataValid) begin
                        // Compare current magnitude with stored peaks
                        // Update peakMagnitude1 and peakMagnitude2
                        fftBinIndex <= fftBinIndex + 1'b1;
                        fftBinCounter <= fftBinCounter + 1'b1;
                    end
                end
                
                STATE_COMPLETE: begin
                    // Convert bin indices to frequencies
                    maxFreq1 <= peakBin1 * frequencyResolution;
                    maxFreq2 <= peakBin2 * frequencyResolution;
                    maxMagnitude1 <= peakMagnitude1;
                    maxMagnitude2 <= peakMagnitude2;
                    detectionComplete <= 1'b1;
                end
            endcase
        end
    end
    
    // ========================================================================
    // Peak Detection Logic (Combinational)
    // ========================================================================
    always_comb begin
        peakMagnitude1_next = peakMagnitude1;
        peakBin1_next = peakBin1;
        peakMagnitude2_next = peakMagnitude2;
        peakBin2_next = peakBin2;
        
        if (currentState == STATE_FIND_PEAKS && fftDataValid) begin
            // Skip DC bin (index 0) and Nyquist (for real signals)
            if (fftBinIndex > 10'h0 && fftBinIndex < (FftSize >> 1)) begin
                
                // Check if current magnitude is greater than peak 1
                if (magnitude > peakMagnitude1) begin
                    // Shift peak 1 to peak 2
                    peakMagnitude2_next = peakMagnitude1;
                    peakBin2_next = peakBin1;
                    // Update peak 1
                    peakMagnitude1_next = magnitude;
                    peakBin1_next = fftBinIndex;
                end else if (magnitude > peakMagnitude2) begin
                    // Update peak 2 only
                    peakMagnitude2_next = magnitude;
                    peakBin2_next = fftBinIndex;
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
                if (dataValidFlag) begin
                    nextState = STATE_LOAD_FFT;
                end
            end
            
            STATE_LOAD_FFT: begin
                if (fftReadAddr >= FftSize) begin
                    nextState = STATE_RUN_FFT;
                end
            end
            
            STATE_RUN_FFT: begin
                // Wait for FFT completion signal (simplified)
                // In actual implementation, monitor FFT IP status signal
                nextState = STATE_FIND_PEAKS;
            end
            
            STATE_FIND_PEAKS: begin
                if (fftBinCounter >= (FftSize >> 1)) begin
                    nextState = STATE_COMPLETE;
                end
            end
            
            STATE_COMPLETE: begin
                nextState = STATE_IDLE;
            end
        endcase
    end

endmodule