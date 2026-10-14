(* blackbox *)
module DF_ASAP7_512x128_1R1W (
    input clock,
    input ren,
    input wen,
    input [8:0] raddr,
    input [8:0] waddr,
    input [127:0] wdata,
    input [15:0] wmask,
    output [127:0] rdata
);
endmodule
