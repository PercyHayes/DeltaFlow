module DF_ASAP7_512x128_1R1W (
    input clock,
    input ren,
    input wen,
    input [8:0] raddr,
    input [8:0] waddr,
    input [127:0] wdata,
    input [15:0] wmask,
    output reg [127:0] rdata
);
    reg [127:0] mem [0:511];
    integer lane;

    always @(posedge clock) begin
        if (ren) begin
            rdata <= mem[raddr];
        end
        if (wen) begin
            for (lane = 0; lane < 16; lane = lane + 1) begin
                if (wmask[lane]) begin
                    mem[waddr][8*lane +: 8] <= wdata[8*lane +: 8];
                end
            end
        end
    end
endmodule
