`timescale 1ns/1ps

// Scrolling banner whose glyphs are pushed over the UART as raw 12x12 bitmaps.
//
// Why bitmap transfer instead of a font ROM: the board only has 72 KB of block
// RAM, so a GB2312 glyph set (6763 chars x 24 B = 162 KB) cannot fit. Letting
// the PC render the text with its own font and send the dots sidesteps the
// problem entirely - any character the PC can draw (Chinese, ASCII, symbols)
// can be shown, and changing the message never needs a rebuild.
//
// Wire protocol (all bytes, no handshake):
//   0xA5 0x01 <n>                    set the number of visible cells (1..32)
//   0xA5 0x02 <cell> <32 bytes>      cell 0..31, 16 rows x 2 bytes, high byte
//                                    first; bit 11 of each row is the leftmost
//                                    pixel, rows 12..15 are unused and 0
//   0xA5 0x03                        clear (cell count = 0)
//
// Layout mirrors the original Chinese banner: 32 cells x 16 px stride = 512 px
// wide, scrolling from +H_ACT to -512 at SCROLL_STEP px per frame, drawn on the
// 12 rows starting at TEXT_Y.
module serial_glyph_line #(
    parameter integer TEXT_Y = 438,
    parameter integer SCROLL_STEP = 2,
    parameter integer H_ACT = 640,
    parameter integer MAX_CELLS = 32,
    parameter integer CLKS_PER_BIT = 217,
    parameter [23:0] TEXT_COLOR = 24'h9FE8FF
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    input  wire        hsync,
    input  wire        vsync,
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [23:0] rgb_in,
    output reg         hsync_out,
    output reg         vsync_out,
    output reg         de_out,
    output reg  [9:0]  pixel_x_out,
    output wire [23:0] rgb_out,
    output wire        glyph_active
);
    localparam integer CELL_W = 12;
    localparam integer CELL_H = 12;
    localparam integer STRIDE = 16;
    localparam integer DRAM   = MAX_CELLS * STRIDE;     // 512 words of 12 bits
    localparam integer TEXT_W = MAX_CELLS * STRIDE;

    localparam [11:0]  Y_TOP  = TEXT_Y;
    localparam [11:0]  Y_BOT  = TEXT_Y + CELL_H;
    localparam [3:0]   Y_ROW0 = TEXT_Y % 16;
    localparam [10:0]  H_ACT_R  = H_ACT;
    localparam [10:0]  TEXT_W_R = TEXT_W;
    localparam [10:0]  SPAN_R   = H_ACT + TEXT_W;
    localparam [10:0]  STEP_R   = SCROLL_STEP;

    // ------------------------------------------------ UART receive
    wire       rx_valid;
    wire [7:0] rx_data;
    uart_rx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .rx_valid(rx_valid), .rx_data(rx_data)
    );

    // ------------------------------------------------ glyph RAM (simple dual port)
    // One always block, no reset: TD infers a block RAM with a registered read.
    // rd_addr is declared here and assigned in the pixel section below so the
    // read port and the display maths share the same array.
    reg  [11:0] gbuf [0:DRAM-1];
    reg  [11:0] gq;
    reg         wr_en;
    reg  [8:0]  wr_addr;
    reg  [11:0] wr_data;
    wire [8:0]  rd_addr;

    always @(posedge clk) begin
        if (wr_en)
            gbuf[wr_addr] <= wr_data;
        gq <= gbuf[rd_addr];
    end

    // ------------------------------------------------ frame parser
    localparam [2:0] S_HUNT = 3'd0;
    localparam [2:0] S_CMD  = 3'd1;
    localparam [2:0] S_CNT  = 3'd2;
    localparam [2:0] S_IDX  = 3'd3;
    localparam [2:0] S_DH   = 3'd4;
    localparam [2:0] S_DL   = 3'd5;

    reg [2:0] st;
    reg [4:0] cell_wr;
    reg [3:0] row_wr;
    reg [7:0] hi_wr;
    reg [5:0] count_q;
    reg       kick;         // one-cycle pulse: park the banner at the left edge

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_HUNT;
            wr_en   <= 1'b0;
            wr_addr <= 9'd0;
            wr_data <= 12'd0;
            cell_wr <= 5'd0;
            row_wr  <= 4'd0;
            hi_wr   <= 8'd0;
            count_q <= 6'd0;
            kick    <= 1'b0;
        end else begin
            wr_en <= 1'b0;
            kick  <= 1'b0;
            if (rx_valid) begin
                case (st)
                S_HUNT: if (rx_data == 8'hA5) st <= S_CMD;
                S_CMD: begin
                    if (rx_data == 8'h01)      st <= S_CNT;
                    else if (rx_data == 8'h02) st <= S_IDX;
                    else if (rx_data == 8'h03) begin
                        count_q <= 6'd0;
                        st      <= S_HUNT;
                    end else                   st <= S_HUNT;
                end
                S_CNT: begin
                    count_q <= (rx_data > 6'd32) ? 6'd32 : rx_data[5:0];
                    kick    <= 1'b1;      // show the new text right away
                    st      <= S_HUNT;
                end
                S_IDX: begin
                    cell_wr <= rx_data[4:0];
                    row_wr  <= 4'd0;
                    st      <= S_DH;
                end
                S_DH: begin
                    hi_wr <= rx_data;
                    st    <= S_DL;
                end
                S_DL: begin
                    wr_en   <= 1'b1;
                    wr_addr <= {cell_wr, row_wr};      // cell * 16 + row
                    wr_data <= {hi_wr[3:0], rx_data};
                    if (row_wr == 4'd15) st <= S_HUNT;
                    else begin
                        row_wr <= row_wr + 1'b1;
                        st     <= S_DH;
                    end
                end
                default: st <= S_HUNT;
                endcase
            end
        end
    end

    // ------------------------------------------------ pixel side
    reg [11:0] line_y;
    reg        de_q;
    reg        vs_q;
    reg [10:0] scroll_pos;

    wire frame_start = vs_q && !vsync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_y     <= 12'd0;
            de_q       <= 1'b0;
            vs_q       <= 1'b1;
            scroll_pos <= 11'd0;
        end else begin
            de_q <= de;
            vs_q <= vsync;
            if (!vsync)
                line_y <= 12'd0;
            else if (de_q && !de)
                line_y <= line_y + 1'b1;
            if (kick)
                scroll_pos <= H_ACT_R;          // text starts at x = 0
            else if (frame_start) begin
                if (scroll_pos >= SPAN_R - STEP_R)
                    scroll_pos <= 11'd0;
                else
                    scroll_pos <= scroll_pos + STEP_R;
            end
        end
    end

    wire signed [10:0] banner_x = $signed({1'b0, H_ACT_R}) -
                                  $signed({1'b0, scroll_pos});
    wire signed [10:0] diff = $signed({1'b0, pixel_x}) - banner_x;

    wire        y_band    = de && (line_y >= Y_TOP) && (line_y < Y_BOT);
    wire [4:0]  cell_idx  = diff[8:4];
    wire [3:0]  glyph_col = diff[3:0];
    wire [3:0]  row_idx   = line_y[3:0] - Y_ROW0;
    wire        in_text   = y_band && (diff >= 0) && (diff < TEXT_W_R) &&
                            ({1'b0, cell_idx} < count_q);
    assign      rd_addr   = {cell_idx, row_idx};

    reg [23:0] rgb_q;
    reg [3:0]  col_q;
    reg        hit_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hsync_out   <= 1'b1;
            vsync_out   <= 1'b1;
            de_out      <= 1'b0;
            pixel_x_out <= 10'd0;
            rgb_q       <= 24'd0;
            col_q       <= 4'd0;
            hit_q       <= 1'b0;
        end else begin
            hsync_out   <= hsync;
            vsync_out   <= vsync;
            de_out      <= de;
            pixel_x_out <= pixel_x;
            rgb_q       <= rgb_in;
            col_q       <= glyph_col;
            hit_q       <= in_text && (glyph_col < CELL_W);
        end
    end

    wire glyph_on = hit_q && gq[11 - col_q];
    assign rgb_out      = glyph_on ? TEXT_COLOR : rgb_q;
    assign glyph_active = glyph_on;
endmodule
