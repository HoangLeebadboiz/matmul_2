`timescale 1ns/1ps
`default_nettype none

// One packed word contains TWO signed DATA_W-bit elements.
// A 2x2 tile occupies words 2*t and 2*t+1, not a separate RAM block.
// Default: 8 words x 32 bits, four tiles in one logical bank.
// Scalar writes update only one element via byte enables. No read-modify-write.
// Packed byte lanes follow the Quartus Standard RAM inference template.
module tiled_bram_bank #(
    parameter integer DATA_W = 16, // byte-aligned; validated at 8 and 16
    parameter integer ADDR_W = 3
) (
    input  wire                         clk,
    input  wire                         wr_en,
    input  wire [ADDR_W-1:0]            wr_addr,
    input  wire                         wr_lane, // 0=low element, 1=high
    input  wire signed [DATA_W-1:0]     wr_data,
    input  wire                         rd_en,
    input  wire [ADDR_W-1:0]            rd_addr,
    output logic [2*DATA_W-1:0]         rd_data
);
    localparam integer BYTES_PER_ELEMENT = DATA_W/8;
    localparam integer BYTES_PER_WORD = 2*BYTES_PER_ELEMENT;
    wire [2*DATA_W-1:0] wr_word;
    wire [BYTES_PER_WORD-1:0] wr_byte_en;

    // Duplicate the scalar onto both lanes; enables select which is written.
    assign wr_word = {wr_data, wr_data};
    assign wr_byte_en = wr_lane ?
        {{BYTES_PER_ELEMENT{1'b1}}, {BYTES_PER_ELEMENT{1'b0}}} :
        {{BYTES_PER_ELEMENT{1'b0}}, {BYTES_PER_ELEMENT{1'b1}}};

    (* ramstyle = "M9K" *) logic [BYTES_PER_WORD-1:0][7:0] mem [0:(1<<ADDR_W)-1];

    // synthesis translate_off
    initial begin
        if (DATA_W < 8 || DATA_W%8 != 0)
            $fatal(1, "tiled_bram_bank needs a positive whole-byte DATA_W");
    end
    // synthesis translate_on

    integer byte_index;
    always_ff @(posedge clk) begin
        if (wr_en) begin
            for (byte_index = 0; byte_index < BYTES_PER_WORD; byte_index = byte_index + 1)
                if (wr_byte_en[byte_index])
                    mem[wr_addr][byte_index] <= wr_word[8*byte_index +: 8];
        end
        if (rd_en)
            rd_data <= mem[rd_addr];
    end
    // No reset on RAM or its data register. The top's load masks and FSM
    // prevent uninitialized reads and separate loading from computation.
endmodule

`default_nettype wire
