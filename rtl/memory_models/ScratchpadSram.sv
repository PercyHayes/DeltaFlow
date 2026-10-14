// Fixed-budget SRAM dispatch. Use functional views for simulation and
// blackbox views for synthesis; both expose the same 1R1W port interface.
module ScratchpadSram #(
    parameter DEPTH = 1024,
    parameter BYTES = 16
) (
    input clock,
    input ren,
    input wen,
    input [$clog2(DEPTH)-1:0] raddr,
    input [$clog2(DEPTH)-1:0] waddr,
    input [8*BYTES-1:0] wdata,
    input [BYTES-1:0] wmask,
    output [8*BYTES-1:0] rdata
);
    generate
        if (DEPTH == 1024 && BYTES == 16) begin : shape_0
            DF_ASAP7_1024x128_1R1W impl (
                .clock(clock),
                .ren(ren),
                .wen(wen),
                .raddr(raddr),
                .waddr(waddr),
                .wdata(wdata),
                .wmask(wmask),
                .rdata(rdata)
            );
        end
        else if (DEPTH == 512 && BYTES == 16) begin : shape_1
            DF_ASAP7_512x128_1R1W impl (
                .clock(clock),
                .ren(ren),
                .wen(wen),
                .raddr(raddr),
                .waddr(waddr),
                .wdata(wdata),
                .wmask(wmask),
                .rdata(rdata)
            );
        end
        else if (DEPTH == 256 && BYTES == 4) begin : shape_2
            DF_ASAP7_256x32_1R1W impl (
                .clock(clock),
                .ren(ren),
                .wen(wen),
                .raddr(raddr),
                .waddr(waddr),
                .wdata(wdata),
                .wmask(wmask),
                .rdata(rdata)
            );
        end
        else begin : unsupported_geometry
            DELTAFLOW_UNSUPPORTED_SRAM_GEOMETRY fail();
        end
    endgenerate
endmodule
