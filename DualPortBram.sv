module DualPortBram
#(parameter DataWidth=16, AddressWidth=11, Depth=2048)
(
    input  logic [DataWidth-1:0]  portA_data,
    input  logic [AddressWidth-1:0] portA_addr,
    input  logic                  portA_wen,
    input  logic                  portA_clk,
    output logic [DataWidth-1:0]  portA_out,

    input  logic [DataWidth-1:0]  portB_data,
    input  logic [AddressWidth-1:0] portB_addr,
    input  logic                  portB_clk,
    output logic [DataWidth-1:0]  portB_out
);
    // Declare memory array
    logic [DataWidth-1:0] mem_array [0:Depth-1];

    // Port A: Write Operation
    always_ff @(posedge portA_clk) begin
        if (portA_wen) begin
            mem_array[portA_addr] <= portA_data;
        end
        portA_out <= mem_array[portA_addr]; // Read the same address after writing
    end

    // Port B: Read Operation
    always_ff @(posedge portB_clk) begin
        portB_out <= mem_array[portB_addr];
    end
endmodule