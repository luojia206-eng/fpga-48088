`timescale 1ns/1ps

// One-clock registered watermark stage: adaptive-ink brand mark.
//
// The 12-bit row ROM is forced into block RAM. RGB and every video timing
// signal consumed by the HDMI bridge are delayed by the same single clock,
// so DE/X/sync alignment is preserved while the large font mux is removed
// from the pixel critical path.
module brand_watermark_24 #(
    parameter integer WM_X = 568,
    parameter integer WM_Y = 460,
    // Forensic build tag. 8'h00 keeps the pixel output bit-identical to the
    // legacy behaviour. A non-zero tag modulates the blue-channel LSB of the
    // non-glyph pixels inside the watermark box (bit k of the tag lands on
    // box column k mod 8), which is invisible on a display but decodable
    // from any screenshot row inside the box.
    parameter [7:0] BUILD_ID = 8'h00
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        hsync,
    input  wire        vsync,
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [11:0] pixel_y,
    input  wire [23:0] rgb_in,
    input  wire        overlay_active_in,
    output reg         hsync_out,
    output reg         vsync_out,
    output reg         de_out,
    output reg  [9:0]  pixel_x_out,
    output wire [23:0] rgb_out,
    output wire        overlay_active_out
);
    localparam integer GLYPH_W = 12;
    localparam integer GLYPH_H = 12;
    localparam integer GLYPH_STRIDE = 16;
    localparam integer GLYPH_COUNT = 4;
    localparam integer WM_W = GLYPH_STRIDE * GLYPH_COUNT;
    localparam [9:0] WM_X_R = WM_X;

    reg [11:0] glyph_rom [0:47] /* fehdl force_ram=1, ram_style="bram" */;
    integer init_index;
    initial begin
        for (init_index = 0; init_index < 48; init_index = init_index + 1)
            glyph_rom[init_index] = 12'h000;

        // glyph U+5E94
        glyph_rom[0]=12'h040; glyph_rom[1]=12'h020;
        glyph_rom[2]=12'h7FE; glyph_rom[3]=12'h400;
        glyph_rom[4]=12'h440; glyph_rom[5]=12'h424;
        glyph_rom[6]=12'h524; glyph_rom[7]=12'h4A4;
        glyph_rom[8]=12'h488; glyph_rom[9]=12'h488;
        glyph_rom[10]=12'h410; glyph_rom[11]=12'hBFE;

        // glyph U+6025
        glyph_rom[12]=12'h200; glyph_rom[13]=12'h3F0;
        glyph_rom[14]=12'h410; glyph_rom[15]=12'hBFC;
        glyph_rom[16]=12'h004; glyph_rom[17]=12'h3FC;
        glyph_rom[18]=12'h004; glyph_rom[19]=12'h7FC;
        glyph_rom[20]=12'h040; glyph_rom[21]=12'h524;
        glyph_rom[22]=12'h50A; glyph_rom[23]=12'h8FA;

        // glyph U+53D1
        glyph_rom[24]=12'h090; glyph_rom[25]=12'h888;
        glyph_rom[26]=12'h880; glyph_rom[27]=12'hFFE;
        glyph_rom[28]=12'h100; glyph_rom[29]=12'h100;
        glyph_rom[30]=12'h1F8; glyph_rom[31]=12'h288;
        glyph_rom[32]=12'h250; glyph_rom[33]=12'h420;
        glyph_rom[34]=12'h8D8; glyph_rom[35]=12'h306;

        // glyph U+5E03
        glyph_rom[36]=12'h080; glyph_rom[37]=12'h080;
        glyph_rom[38]=12'hFFE; glyph_rom[39]=12'h120;
        glyph_rom[40]=12'h220; glyph_rom[41]=12'h3FC;
        glyph_rom[42]=12'h524; glyph_rom[43]=12'h524;
        glyph_rom[44]=12'h924; glyph_rom[45]=12'h124;
        glyph_rom[46]=12'h12C; glyph_rom[47]=12'h020;

    end

    wire watermark_bounds = de &&
        pixel_x >= WM_X && pixel_x < WM_X + WM_W &&
        pixel_y >= WM_Y && pixel_y < WM_Y + GLYPH_H;
    wire [3:0] glyph_index = (pixel_x - WM_X) >> 4;
    wire [3:0] glyph_col = (pixel_x - WM_X) & 4'hf;
    wire [3:0] glyph_row_index = pixel_y - WM_Y;
    wire [7:0] glyph_base = ({4'b0, glyph_index} << 3) +
                            ({4'b0, glyph_index} << 2);
    wire [7:0] glyph_address = glyph_base + glyph_row_index;

    reg [23:0] rgb_q;
    reg [11:0] glyph_row_q;
    reg [3:0] glyph_col_q;
    reg watermark_bounds_q;
    reg overlay_active_q;
    reg [9:0] box_col_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hsync_out <= 1'b1;
            vsync_out <= 1'b1;
            de_out <= 1'b0;
            pixel_x_out <= 10'd0;
            rgb_q <= 24'd0;
            glyph_row_q <= 12'd0;
            glyph_col_q <= 4'd0;
            watermark_bounds_q <= 1'b0;
            overlay_active_q <= 1'b0;
            box_col_q <= 10'd0;
        end else begin
            hsync_out <= hsync;
            vsync_out <= vsync;
            de_out <= de;
            pixel_x_out <= pixel_x;
            rgb_q <= rgb_in;
            glyph_col_q <= glyph_col;
            watermark_bounds_q <= watermark_bounds && glyph_col < GLYPH_W;
            overlay_active_q <= overlay_active_in;
            box_col_q <= pixel_x - WM_X_R;
            if (watermark_bounds && glyph_col < GLYPH_W)
                glyph_row_q <= glyph_rom[glyph_address];
            else
                glyph_row_q <= 12'd0;
        end
    end

    wire watermark_hit = watermark_bounds_q &&
                         glyph_row_q[11 - glyph_col_q];
    // Adaptive ink. A single fixed colour cannot survive every image: 24'hE8F4FF
    // vanishes on a white footer, 24'h7E8FA3 vanishes on a dark photo. Pick the
    // ink furthest from the local background instead - weighted luma
    // (0.5R + G + 0.25B) is the cheapest 3-add approximation, and it lands the
    // threshold near true mid-grey. rgb_q is already the registered background,
    // so this adds no pipeline stage.
    wire [8:0]  bg_luma_q  = rgb_q[23:17] + rgb_q[15:8] + rgb_q[7:6];
    wire        bg_bright  = bg_luma_q > 9'd220;
    wire [23:0] wm_ink     = bg_bright ? 24'h101820 : 24'hE8F4FF;

    wire id_patch_en = (BUILD_ID != 8'h00) && watermark_bounds_q &&
                       !watermark_hit;
    wire id_bit = BUILD_ID[box_col_q[2:0]];
    assign rgb_out = watermark_hit ? wm_ink :
                     (id_patch_en ? {rgb_q[23:8], rgb_q[7:1], id_bit} :
                                    rgb_q);
    assign overlay_active_out = overlay_active_q || watermark_hit;
endmodule
