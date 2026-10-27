`timescale 1ns/1ps

// Streaming "fit to 640x480" decimator for the ingest path.
//
// Step 1 of the adaptive-geometry work: pictures that are at least 640x480
// (800x600, 1024x768, 1280x720, 1920x1080 ...) are decimated to exactly
// DST_W x DST_H with nearest-neighbour sampling; smaller pictures are
// rejected by the decoder. Because the source is never larger in destination
// space, at most one destination pixel is produced per source pixel, so this
// block is a pure pipeline: no line buffer, no replay engine and no
// arbitration, which keeps it immune to the bursty SD byte stream.
//
// Mapping (both axes) is the phase accumulator
//     acc += DST;  if (acc >= SRC) { acc -= SRC; keep this element; }
// which yields exactly DST kept elements over SRC inputs.
module bmp_scale_fit #(
    parameter integer DST_W = 640,
    parameter integer DST_H = 480
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,   // pulse: a new BMP file begins
    input  wire        dims_valid,    // level: header accepted, src_w/h usable
    input  wire [15:0] src_w,
    input  wire [15:0] src_h,
    input  wire        src_valid,
    input  wire [23:0] src_rgb,
    output reg         dst_valid,
    output reg  [23:0] dst_rgb,
    output reg         dst_frame_done
);
    reg [15:0] width_q;
    reg [15:0] height_q;
    reg [15:0] src_col;      // 0..W-1 within the current source row
    reg [15:0] acc_x;
    reg [15:0] acc_y;
    reg [15:0] rows_out;
    reg        row_keep;

    wire [15:0] w_eff = (width_q  < DST_W) ? DST_W[15:0] : width_q;
    wire [15:0] h_eff = (height_q < DST_H) ? DST_H[15:0] : height_q;

    wire        row_start = (src_col == 16'd0);
    wire [15:0] acc_x_next = acc_x + DST_W[15:0];
    wire        col_keep   = (acc_x_next >= w_eff);
    wire [15:0] acc_x_after = acc_x_next - w_eff;

    wire [15:0] acc_y_next = acc_y + DST_H[15:0];
    wire        row_keep_new = (acc_y_next >= h_eff);
    wire [15:0] acc_y_after  = acc_y_next - h_eff;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            width_q <= 16'd640;
            height_q <= 16'd480;
            src_col <= 16'd0;
            acc_x <= 16'd0;
            acc_y <= 16'd0;
            rows_out <= 16'd0;
            row_keep <= 1'b1;
            dst_valid <= 1'b0;
            dst_rgb <= 24'd0;
            dst_frame_done <= 1'b0;
        end else begin
            dst_valid <= 1'b0;
            dst_frame_done <= 1'b0;

            // The decoder publishes the geometry once the header is accepted.
            if (dims_valid && (src_col == 16'd0)) begin
                width_q <= src_w;
                height_q <= src_h;
            end

            if (frame_start) begin
                src_col <= 16'd0;
                acc_x <= 16'd0;
                acc_y <= 16'd0;
                rows_out <= 16'd0;
                row_keep <= 1'b1;
            end else if (src_valid) begin
                if (row_start) begin
                    // Row decision: keep or drop this source row entirely.
                    // The first pixel of the row starts from a zero phase.
                    row_keep <= row_keep_new;
                    acc_y <= row_keep_new ? acc_y_after : acc_y_next;
                    acc_x <= (DST_W[15:0] >= w_eff) ? (DST_W[15:0] - w_eff)
                                                    : DST_W[15:0];
                    if ((DST_W[15:0] >= w_eff) && row_keep_new && (rows_out < DST_H)) begin
                        dst_valid <= 1'b1;
                        dst_rgb <= src_rgb;
                    end
                end else begin
                    acc_x <= col_keep ? acc_x_after : acc_x_next;
                    if (col_keep && row_keep && (rows_out < DST_H)) begin
                        dst_valid <= 1'b1;
                        dst_rgb <= src_rgb;
                    end
                end

                // Column index wrap and end-of-row bookkeeping.
                if (src_col == (w_eff - 1'b1)) begin
                    src_col <= 16'd0;
                    if (row_keep && (rows_out < DST_H)) begin
                        rows_out <= rows_out + 1'b1;
                        if (rows_out == (DST_H - 1))
                            dst_frame_done <= 1'b1;
                    end
                end else begin
                    src_col <= src_col + 1'b1;
                end
            end
        end
    end
endmodule
