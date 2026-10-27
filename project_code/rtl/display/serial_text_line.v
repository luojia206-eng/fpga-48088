`timescale 1ns/1ps

// ASCII text line driven by the UART receive pin.
//
// The PC types into the terminal and the characters appear on screen: each
// received printable byte is appended to a 40-character line buffer, CR/LF
// rewinds to the start of the line (the old text stays visible until it is
// overwritten), BS deletes, and FF clears the line. The line is rendered with
// the 8x12 bitmap font in ascii_font_rom.v.
//
// Geometry: 40 cells x 8 px = 320 px, centred at x=160..479, rows 412..423
// (just above the Chinese scrolling banner at y=438, below the waveform which
// ends at y=407).
//
// Like the other overlay stages every video signal is delayed by exactly one
// clock so the de/hs/vs/pixel_x bundle stays aligned downstream.
module serial_text_line #(
    parameter integer TEXT_Y = 438,
    parameter integer SCROLL_STEP = 2,
    parameter integer H_ACT = 640,
    parameter integer LINE_CHARS = 40,
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
    output wire        text_active
);
    localparam integer CELL_W = 8;
    localparam integer CELL_H = 12;
    localparam integer PRESET_CELLS = 16;   // cells per preset message
    localparam integer TEXT_W = LINE_CHARS * CELL_W;
    localparam integer SPAN   = H_ACT + TEXT_W;
    localparam [11:0]  Y_TOP  = TEXT_Y;
    localparam [11:0]  Y_BOT  = TEXT_Y + CELL_H;
    localparam [3:0]   Y_ROW0 = TEXT_Y % 16;
    localparam [10:0]  H_ACT_R = H_ACT;
    localparam [10:0]  TEXT_W_R = TEXT_W;
    localparam [10:0]  SPAN_R  = SPAN;
    localparam [10:0]  STEP_R  = SCROLL_STEP;

    // ------------------------------------------------ receive side
    wire       rx_valid;
    wire [7:0] rx_data;
    uart_rx_byte #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .rx_valid(rx_valid), .rx_data(rx_data)
    );

    reg [7:0] line_buf [0:LINE_CHARS-1];
    reg [5:0] wr_pos;
    reg [3:0] msg_sel;   // 0 = ASCII text mode, 1..8 = preset message
    reg       kick;      // one-cycle pulse: park the banner at the left edge
    integer   k;

    // Power-on message so the banner is visibly alive before anything is typed.
    localparam [103:0] BOOT_PAD = {13{8'h20}};
    localparam [LINE_CHARS*8-1:0] BOOT_MSG =
        {"PROMax READY - TYPE FROM PC", BOOT_PAD};

    initial begin
        for (k = 0; k < LINE_CHARS; k = k + 1)
            line_buf[k] = BOOT_MSG[(LINE_CHARS - 1 - k) * 8 +: 8];
    end

    wire rx_printable = (rx_data >= 8'h20) && (rx_data <= 8'h7E);
    wire rx_backspace = (rx_data == 8'h08) || (rx_data == 8'h7F);
    wire rx_newline   = (rx_data == 8'h0D) || (rx_data == 8'h0A);
    wire rx_clear     = (rx_data == 8'h0C);
    wire rx_bang      = (rx_data == 8'h21);           // '!'
    wire rx_digit     = (rx_data >= 8'h31) && (rx_data <= 8'h38); // '1'..'8'

    reg bang_q;   // previous byte was '!', so the next digit picks a message

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_pos  <= 6'd0;
            msg_sel <= 4'd0;
            kick    <= 1'b0;
            bang_q  <= 1'b0;
        end else begin
            kick <= 1'b0;
            if (rx_valid) begin
                // Any received byte brings the banner back into view so new text
                // is visible immediately instead of waiting for the scroll to
                // wrap around.
                kick <= 1'b1;

                // Preset selection, two ways so neither terminal setting can
                // break the demo:
                //   0x01..0x08   single byte (serial assistant in HEX mode)
                //   !1 .. !8     plain text     (serial assistant in text mode)
                if (rx_data >= 8'h01 && rx_data <= 8'h08) begin
                    msg_sel <= rx_data[3:0];
                    bang_q  <= 1'b0;
                end else if (rx_bang) begin
                    bang_q <= 1'b1;
                end else if (bang_q) begin
                    bang_q <= 1'b0;
                    if (rx_digit)
                        msg_sel <= rx_data[3:0];     // '1' -> 1 ... '8' -> 8
                end else if (rx_printable) begin
                    msg_sel <= 4'd0;                 // back to the typed line
                    line_buf[wr_pos] <= rx_data;
                    if (wr_pos != LINE_CHARS - 1)
                        wr_pos <= wr_pos + 1'b1;
                end else if (rx_backspace) begin
                    if (wr_pos != 6'd0) begin
                        wr_pos <= wr_pos - 1'b1;
                        line_buf[wr_pos - 1'b1] <= 8'h20;
                    end
                end else if (rx_newline) begin
                    // Start a fresh line: with "send newline" enabled in the
                    // terminal this fires before each message, so what is typed
                    // is exactly what ends up on screen.
                    wr_pos <= 6'd0;
                    for (k = 0; k < LINE_CHARS; k = k + 1)
                        line_buf[k] <= 8'h20;
                end else if (rx_clear) begin
                    wr_pos <= 6'd0;
                    for (k = 0; k < LINE_CHARS; k = k + 1)
                        line_buf[k] <= 8'h20;
                end
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
            line_y <= 12'd0;
            de_q   <= 1'b0;
            vs_q   <= 1'b1;
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

    // banner_x walks from +H_ACT (fully off the right edge) down to -TEXT_W
    // (fully off the left), exactly like the original Chinese banner.
    wire signed [10:0] banner_x = $signed({1'b0, H_ACT_R}) -
                                  $signed({1'b0, scroll_pos});
    wire signed [10:0] diff = $signed({1'b0, pixel_x}) - banner_x;

    wire        y_band   = de && (line_y >= Y_TOP) && (line_y < Y_BOT);
    wire [3:0]  row_raw  = line_y[3:0] - Y_ROW0;
    // Rows 12..15 of the 16-row cell are never drawn; clamp so the ROM address
    // can never walk past the end of the table.
    wire [3:0]  glyph_row = (row_raw > 4'd11) ? 4'd0 : row_raw;

    // ---- ASCII path: LINE_CHARS cells of 8 px, font index from ascii_font_rom
    wire        a_in_text = y_band && (diff >= 0) && (diff < TEXT_W_R);
    wire [5:0]  a_cell    = diff[8:3];
    wire [2:0]  a_col     = diff[2:0];

    wire [7:0]  ch_code = line_buf[a_cell];
    // Map the printable range to ROM index 0..94; anything else renders blank.
    wire [7:0]  ch_idx  = (ch_code >= 8'h20) && (ch_code <= 8'h7E)
                          ? (ch_code - 8'h20) : 8'd95;
    wire [10:0] font_addr = (ch_idx << 3) + (ch_idx << 2) + glyph_row;

    wire [7:0]  font_data;
    ascii_font_rom u_font (
        .clk(clk),
        .addr(font_addr),
        .data(font_data)
    );

    // ---- preset path: PRESET_CELLS cells of 16 px, glyph = msg*16 + cell
    localparam integer P_STRIDE = 16;
    localparam integer P_WIDTH  = PRESET_CELLS * P_STRIDE;
    localparam [10:0]  P_W_R    = P_WIDTH;

    wire        p_in_text = y_band && (diff >= 0) && (diff < P_W_R);
    wire [4:0]  p_cell    = diff[8:4];
    wire [3:0]  p_col     = diff[3:0];
    wire [2:0]  p_mbase   = msg_sel[2:0] - 3'd1;
    wire [6:0]  p_gidx    = {p_mbase, 4'd0} + {2'b00, p_cell};
    wire [10:0] p_addr    = (p_gidx << 3) + (p_gidx << 2) + glyph_row;

    wire [11:0] preset_data;
    preset_banner_rom u_preset (
        .clk(clk),
        .addr(p_addr),
        .data(preset_data)
    );

    // ---- mode select: 0 = whatever was typed, 1..8 = a preset message
    wire       preset_mode = (msg_sel != 4'd0);
    wire       in_text     = preset_mode ? p_in_text : a_in_text;
    wire [3:0] glyph_col   = preset_mode ? p_col : {1'b0, a_col};
    wire       col_ok      = preset_mode ? (p_col < 4'd12) : 1'b1;

    reg [23:0] rgb_q;
    reg [3:0]  col_q;
    reg        hit_q;
    reg        pmode_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hsync_out   <= 1'b1;
            vsync_out   <= 1'b1;
            de_out      <= 1'b0;
            pixel_x_out <= 10'd0;
            rgb_q       <= 24'd0;
            col_q       <= 4'd0;
            hit_q       <= 1'b0;
            pmode_q     <= 1'b0;
        end else begin
            hsync_out   <= hsync;
            vsync_out   <= vsync;
            de_out      <= de;
            pixel_x_out <= pixel_x;
            rgb_q       <= rgb_in;
            col_q       <= glyph_col;
            hit_q       <= in_text && col_ok;
            pmode_q     <= preset_mode;
        end
    end

    wire glyph_on = hit_q && (pmode_q ? preset_data[4'd11 - col_q]
                                      : font_data[3'd7 - col_q]);
    assign rgb_out      = glyph_on ? TEXT_COLOR : rgb_q;
    assign text_active  = glyph_on;
endmodule
