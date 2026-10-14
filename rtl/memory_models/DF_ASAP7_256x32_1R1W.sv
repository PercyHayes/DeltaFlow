module DF_ASAP7_256x32_1R1W (
    input clock,
    input ren,
    input wen,
    input [7:0] raddr,
    input [7:0] waddr,
    input [31:0] wdata,
    input [3:0] wmask,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:255];
    integer lane;

    always @(posedge clock) begin
        if (ren) begin
            rdata <= mem[raddr];
        end
        if (wen) begin
            for (lane = 0; lane < 4; lane = lane + 1) begin
                if (wmask[lane]) begin
                    mem[waddr][8*lane +: 8] <= wdata[8*lane +: 8];
                end
            end
        end
    end
endmodule
