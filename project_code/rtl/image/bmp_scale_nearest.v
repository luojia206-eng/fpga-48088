`timescale 1ns/1ps

// Nearest-neighbour picture resampler for the ingest path.
//
// The BMP decoder hands over the native raster (W x H, 16..2048 on either
// axis, one pixel per src_valid) and this block rewrites it into exactly
// DST_W x DST_H pixels so the frame loader and the SDRAM frame layout stay
// untouched.
//
// Mapping on both axes is the usual phase accumulator:
//     acc += DST;  while (acc >= SRC) { acc -= SRC; keep one more pixel/row }
// i.e. floor((k+1)*SRC/DST) - floor(k*SRC/DST) copies per source element,
// which is exact nearest neighbour for both up- and down-scaling.
//
// Timing: the SD path replays sector data in bursts, so src_valid may arrive
// every 2..3 clocks. Therefore:
//   * every source pixel goes into a 64-entry input FIFO that never
//     back-pressures the decoder;
//   * a plain single-copy pixel is consumed in ONE clock (pop + accumulate +
//     line-buffer write), extra copies continue in PS_H;
//   * the write engine and the replay engine share cycles through a
//     round-robin arbiter, so neither can starve the other.
// The source row lands in one of two DST_W-wide line buffers and is replayed
// k times from the other one; that needs no back-pressure.
module bmp_scale_nearest #(
    parameter integer DST_W = 640,
    parameter integer DST_H = 480
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,   // pulse: a new BMP file begins
    input  wire        dims_valid,    // level: BMP header accepted, src_w/h usable
    input  wire [15:0] src_w,         // native width  (stable after the header)
    input  wire [15:0] src_h,         // native height
    input  wire        src_valid,
    input  wire [23:0] src_rgb,
    output reg         dst_valid,
    output reg  [23:0] dst_rgb,
    output reg         dst_frame_done
);
    localparam integer FIFO_AW = 6;                       // 64 entries
    localparam integer FIFO_D  = 1 << FIFO_AW;

    localparam PS_IDLE = 2'd0;   // waiting for the next source pixel
    localparam PS_VDEC = 2'd1;   // vertical copy count for the row just started
    localparam PS_H1   = 2'd2;   // first horizontal step of a latched pixel
    localparam PS_H    = 2'd3;   // remaining horizontal copies (acc already added)

    reg [25:0] fifo_mem [0:FIFO_D-1];      // {row_start, row_last, rgb}
    reg [FIFO_AW:0] fifo_wp;
    reg [FIFO_AW:0] fifo_rp;
    reg [FIFO_AW:0] fifo_cnt;
    wire        fifo_empty = (fifo_cnt == 0);
    wire [25:0] fifo_head  = fifo_mem[fifo_rp[FIFO_AW-1:0]];

    reg [23:0] line_mem [0:2*DST_W-1];     // two DST_W lines, {bank, column}
    reg [23:0] rd_data;

    reg        wr_en;
    reg        wr_sel;
    reg [9:0]  wr_col;
    reg [23:0] wr_data;

    reg        rd_sel;
    reg [9:0]  rd_col;

    reg [15:0] width_q;
    reg [15:0] height_q;
    reg [15:0] in_col;
    reg [15:0] acc_x;
    reg [15:0] acc_y;
    reg [15:0] rows_out;
    reg [5:0]  k_new;
    reg [5:0]  k_left;
    reg        row_pending;
    reg        row_used;
    reg        rr;                 // round-robin: 0 -> writes, 1 -> replay

    reg [1:0]  ps;
    reg [23:0] px_rgb;
    reg        cur_last;

    wire [15:0] w_eff = (width_q  < 16'd16) ? 16'd16 : width_q;
    wire [15:0] h_eff = (height_q < 16'd16) ? 16'd16 : height_q;

    wire [23:0] head_rgb   = fifo_head[23:0];
    wire        head_first = fifo_head[25];
    wire        head_last  = fifo_head[24];

    // One horizontal step: add DST_W, then subtract w_eff once per copy.
    wire [15:0] acc_add   = acc_x + DST_W[15:0];
    wire        add_due   = (acc_add >= w_eff);
    wire [15:0] add_after = acc_add - w_eff;
    wire        add_more  = add_due && (add_after >= w_eff);

    wire        h_due     = (acc_x >= w_eff);          // PS_H: acc already added
    wire [15:0] h_after   = acc_x - w_eff;
    wire        h_more    = (h_after >= w_eff);

    wire can_pop  = (ps == PS_IDLE) && !fifo_empty;
    wire can_emit = row_pending;
    // The arbiter decides once: pop only when the write engine really wins
    // this cycle, otherwise the FIFO level would drift away from the pointers.
    wire do_pop   = can_pop && (!can_emit || (rr == 1'b0));

    always @(posedge clk) begin
        if (wr_en)
            line_mem[{wr_sel, wr_col}] <= wr_data;
    end

    always @(posedge clk) begin
        rd_data <= line_mem[{rd_sel, rd_col}];
    end

    always @(posedge clk) begin
        if (src_valid)
            fifo_mem[fifo_wp[FIFO_AW-1:0]] <=
                {(in_col == 16'd0), (in_col == (w_eff - 1'b1)), src_rgb};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wp <= 0;
            fifo_rp <= 0;
            fifo_cnt <= 0;
            wr_en <= 1'b0;
            wr_sel <= 1'b0;
            wr_col <= 10'd0;
            wr_data <= 24'd0;
            rd_sel <= 1'b0;
            rd_col <= 10'd0;
            width_q <= 16'd640;
            height_q <= 16'd480;
            in_col <= 16'd0;
            acc_x <= 16'd0;
            acc_y <= 16'd0;
            rows_out <= 16'd0;
            k_new <= 6'd0;
            k_left <= 6'd0;
            row_pending <= 1'b0;
            row_used <= 1'b0;
            rr <= 1'b0;
            ps <= PS_IDLE;
            px_rgb <= 24'd0;
            cur_last <= 1'b0;
            dst_valid <= 1'b0;
            dst_rgb <= 24'd0;
            dst_frame_done <= 1'b0;
        end else begin
            wr_en <= 1'b0;
            dst_valid <= 1'b0;
            dst_frame_done <= 1'b0;

            // Geometry is published by the decoder right after the header.
            if (dims_valid && (in_col == 16'd0) && fifo_empty && (ps == PS_IDLE)) begin
                width_q <= src_w;
                height_q <= src_h;
            end

            if (frame_start) begin
                fifo_wp <= 0;
                fifo_rp <= 0;
                fifo_cnt <= 0;
                in_col <= 16'd0;
                acc_x <= 16'd0;
                acc_y <= 16'd0;
                rows_out <= 16'd0;
                k_new <= 6'd0;
                k_left <= 6'd0;
                row_pending <= 1'b0;
                row_used <= 1'b0;
                rr <= 1'b0;
                ps <= PS_IDLE;
                wr_col <= 10'd0;
                rd_col <= 10'd0;
                wr_sel <= 1'b0;
                rd_sel <= 1'b1;
            end else begin
                // ---------------- input side ----------------
                if (src_valid) begin
                    fifo_wp <= fifo_wp + 1'b1;
                    in_col <= (in_col == (w_eff - 1'b1)) ? 16'd0 : (in_col + 1'b1);
                end

                case ({src_valid, do_pop})
                    2'b10: fifo_cnt <= fifo_cnt + 1'b1;
                    2'b01: fifo_cnt <= fifo_cnt - 1'b1;
                    default: fifo_cnt <= fifo_cnt;
                endcase

                // ---------------- pixel pipeline ----------------
                if (ps == PS_VDEC) begin
                    if (acc_y >= h_eff) begin
                        acc_y <= acc_y - h_eff;
                        k_new <= k_new + 1'b1;
                    end else begin
                        row_used <= (k_new != 6'd0);
                        if (k_new != 6'd0) begin
                            wr_sel <= ~rd_sel;
                            wr_col <= 10'd0;
                        end
                        ps <= PS_H1;
                    end
                end else if (ps == PS_H1) begin
                    // First copy of a latched pixel: add then subtract once.
                    acc_x <= add_due ? add_after : acc_add;
                    if (add_due && row_used && (wr_col < DST_W)) begin
                        wr_en <= 1'b1;
                        wr_data <= px_rgb;
                        wr_col <= wr_col + 1'b1;
                    end
                    if (add_more) begin
                        ps <= PS_H;
                    end else begin
                        ps <= PS_IDLE;
                        if (cur_last && row_used && (k_new != 6'd0) && !row_pending) begin
                            rd_sel <= wr_sel;
                            rd_col <= 10'd0;
                            k_left <= k_new;
                            row_pending <= 1'b1;
                        end
                    end
                end else if (ps == PS_H) begin
                    // Extra copies: acc_x already carries the added phase.
                    acc_x <= h_due ? h_after : acc_x;
                    if (h_due && row_used && (wr_col < DST_W)) begin
                        wr_en <= 1'b1;
                        wr_data <= px_rgb;
                        wr_col <= wr_col + 1'b1;
                    end
                    if (!h_more) begin
                        ps <= PS_IDLE;
                        if (cur_last && row_used && (k_new != 6'd0) && !row_pending) begin
                            rd_sel <= wr_sel;
                            rd_col <= 10'd0;
                            k_left <= k_new;
                            row_pending <= 1'b1;
                        end
                    end
                end else if (do_pop) begin
                    // Fast path: one source pixel in one clock.
                    if (head_first) begin
                        cur_last <= head_last;
                        px_rgb <= head_rgb;
                        fifo_rp <= fifo_rp + 1'b1;
                        acc_x <= 16'd0;
                        acc_y <= acc_y + DST_H[15:0];
                        k_new <= 6'd0;
                        row_used <= 1'b0;
                        ps <= PS_VDEC;
                    end else begin
                        acc_x <= add_due ? add_after : acc_add;
                        if (add_due && row_used && (wr_col < DST_W)) begin
                            wr_en <= 1'b1;
                            wr_data <= head_rgb;
                            wr_col <= wr_col + 1'b1;
                        end
                        fifo_rp <= fifo_rp + 1'b1;
                        if (add_more) begin
                            cur_last <= head_last;
                            px_rgb <= head_rgb;
                            ps <= PS_H;
                        end else if (head_last && row_used && (k_new != 6'd0) && !row_pending) begin
                            rd_sel <= wr_sel;
                            rd_col <= 10'd0;
                            k_left <= k_new;
                            row_pending <= 1'b1;
                        end
                    end
                    rr <= 1'b1;
                end else if (can_emit) begin
                    // ---------------- replay ----------------
                    dst_valid <= 1'b1;
                    dst_rgb <= rd_data;
                    rr <= 1'b0;
                    if (rd_col == (DST_W - 1)) begin
                        rd_col <= 10'd0;
                        rows_out <= rows_out + 1'b1;
                        if (rows_out == (DST_H - 1))
                            dst_frame_done <= 1'b1;
                        if (k_left <= 6'd1) begin
                            k_left <= 6'd0;
                            row_pending <= 1'b0;
                        end else begin
                            k_left <= k_left - 1'b1;
                        end
                    end else begin
                        rd_col <= rd_col + 1'b1;
                    end
                end
            end
        end
    end
endmodule
