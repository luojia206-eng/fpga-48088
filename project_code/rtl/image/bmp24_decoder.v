`timescale 1ns/1ps

// 24-bit BMP streaming decoder with adaptive geometry. The legacy
// EXPECTED_WIDTH/EXPECTED_HEIGHT parameters are kept for compatibility but are
// no longer enforced: the real dimensions come from the file header and are
// published on image_width/image_height for the downstream resampler.
module bmp24_decoder #(
    parameter integer EXPECTED_WIDTH = 640,
    parameter integer EXPECTED_HEIGHT = 480,
    parameter integer MIN_WIDTH  = 640,
    parameter integer MIN_HEIGHT = 480,
    parameter integer MAX_DIMENSION = 2048,
    parameter integer EXACT_GEOMETRY = 1'b0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire [31:0] file_size,
    input  wire        file_valid,
    input  wire [7:0]  file_byte,
    input  wire [31:0] file_byte_offset,
    input  wire        file_end,
    input  wire [7:0]  upstream_error,
    output reg         header_accepted,
    output reg         pixel_valid,
    output reg  [23:0] pixel_rgb,
    output reg  [15:0] pixel_x,
    output reg  [15:0] pixel_y,
    output reg         frame_commit,
    output reg         bmp_done,
    output reg  [7:0]  bmp_error,
    output reg         top_down,
    output reg  [31:0] pixel_offset,
    output reg  [31:0] row_stride,
    output reg  [15:0] image_width,
    output reg  [15:0] image_height
);
    localparam ERR_NONE        = 8'h00;
    localparam ERR_SIGNATURE   = 8'h31;
    localparam ERR_DIMENSIONS  = 8'h32;
    localparam ERR_PLANES      = 8'h33;
    localparam ERR_BPP         = 8'h34;
    localparam ERR_COMPRESSION = 8'h35;
    localparam ERR_OFFSET      = 8'h36;
    localparam ERR_TRUNCATED   = 8'h37;
    localparam ERR_UPSTREAM    = 8'h38;

    reg [7:0] header [0:53];
    reg decode_enabled;
    reg [31:0] byte_in_row;
    reg [31:0] physical_row;
    reg [31:0] pixels_received;
    reg [1:0] channel_phase;
    reg [7:0] blue_hold;
    reg [7:0] green_hold;
    reg [15:0] x_count;

    function [15:0] le16;
        input integer offset;
        begin
            le16 = {header[offset+1], header[offset]};
        end
    endfunction

    function [31:0] le32;
        input integer offset;
        begin
            le32 = {header[offset+3], header[offset+2], header[offset+1], header[offset]};
        end
    endfunction

    task validate_header;
        reg signed [31:0] parsed_height;
        reg [31:0] absolute_height;
        reg [31:0] parsed_width;
        reg [31:0] parsed_offset;
        reg [31:0] parsed_stride;
        reg [31:0] required_end;
        begin
            parsed_height = le32(22);
            absolute_height = parsed_height[31] ? (~parsed_height + 1'b1) : parsed_height;
            parsed_width = le32(18);
            parsed_offset = le32(10);
            parsed_stride = ((parsed_width * 3 + 3) / 4) * 4;
            required_end = parsed_offset + parsed_stride * absolute_height;
            if (header[0] != "B" || header[1] != "M") begin
                bmp_error <= ERR_SIGNATURE;
                decode_enabled <= 1'b0;
            end else if (parsed_width < MIN_WIDTH || parsed_width > MAX_DIMENSION ||
                         absolute_height < MIN_HEIGHT || absolute_height > MAX_DIMENSION ||
                         (EXACT_GEOMETRY &&
                          (parsed_width != EXPECTED_WIDTH ||
                           absolute_height != EXPECTED_HEIGHT)) ||
                         le32(14) < 40) begin
                bmp_error <= ERR_DIMENSIONS;
                decode_enabled <= 1'b0;
            end else if (le16(26) != 1) begin
                bmp_error <= ERR_PLANES;
                decode_enabled <= 1'b0;
            end else if (le16(28) != 24) begin
                bmp_error <= ERR_BPP;
                decode_enabled <= 1'b0;
            end else if (le32(30) != 0) begin
                bmp_error <= ERR_COMPRESSION;
                decode_enabled <= 1'b0;
            end else if (parsed_offset < 54 || parsed_offset >= file_size) begin
                bmp_error <= ERR_OFFSET;
                decode_enabled <= 1'b0;
            end else if (required_end > file_size || le32(2) < required_end || le32(2) > file_size) begin
                bmp_error <= ERR_TRUNCATED;
                decode_enabled <= 1'b0;
            end else begin
                header_accepted <= 1'b1;
                decode_enabled <= 1'b1;
                top_down <= parsed_height[31];
                pixel_offset <= parsed_offset;
                row_stride <= parsed_stride;
                image_width <= parsed_width[15:0];
                image_height <= absolute_height[15:0];
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            header_accepted <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_rgb <= 24'd0;
            pixel_x <= 16'd0;
            pixel_y <= 16'd0;
            frame_commit <= 1'b0;
            bmp_done <= 1'b0;
            bmp_error <= ERR_NONE;
            top_down <= 1'b0;
            pixel_offset <= 32'd0;
            row_stride <= 32'd0;
            image_width <= 16'd0;
            image_height <= 16'd0;
            decode_enabled <= 1'b0;
            byte_in_row <= 32'd0;
            physical_row <= 32'd0;
            pixels_received <= 32'd0;
            channel_phase <= 2'd0;
            blue_hold <= 8'd0;
            green_hold <= 8'd0;
            x_count <= 16'd0;
        end else begin
            pixel_valid <= 1'b0;
            frame_commit <= 1'b0;
            bmp_done <= 1'b0;

            if (file_start) begin
                header_accepted <= 1'b0;
                bmp_error <= ERR_NONE;
                top_down <= 1'b0;
                pixel_offset <= 32'd0;
                row_stride <= 32'd0;
                image_width <= 16'd0;
                image_height <= 16'd0;
                decode_enabled <= 1'b0;
                byte_in_row <= 32'd0;
                physical_row <= 32'd0;
                pixels_received <= 32'd0;
                channel_phase <= 2'd0;
                x_count <= 16'd0;
            end

            if (file_valid && file_byte_offset < 54)
                header[file_byte_offset] <= file_byte;

            if (file_valid && file_byte_offset == 53)
                validate_header();

            if (file_valid && decode_enabled && file_byte_offset >= pixel_offset &&
                physical_row < image_height) begin
                if (byte_in_row < image_width * 3) begin
                    case (channel_phase)
                        2'd0: begin
                            blue_hold <= file_byte;
                            channel_phase <= 2'd1;
                        end
                        2'd1: begin
                            green_hold <= file_byte;
                            channel_phase <= 2'd2;
                        end
                        default: begin
                            pixel_valid <= 1'b1;
                            pixel_rgb <= {file_byte, green_hold, blue_hold};
                            pixel_x <= x_count;
                            pixel_y <= top_down ? physical_row[15:0]
                                               : image_height - 1'b1 - physical_row[15:0];
                            pixels_received <= pixels_received + 1'b1;
                            x_count <= x_count + 1'b1;
                            channel_phase <= 2'd0;
                        end
                    endcase
                end

                if (byte_in_row == row_stride - 1'b1) begin
                    byte_in_row <= 32'd0;
                    physical_row <= physical_row + 1'b1;
                    channel_phase <= 2'd0;
                    x_count <= 16'd0;
                end else begin
                    byte_in_row <= byte_in_row + 1'b1;
                end
            end

            if (file_end) begin
                bmp_done <= 1'b1;
                decode_enabled <= 1'b0;
                if (upstream_error != 0) begin
                    bmp_error <= ERR_UPSTREAM;
                end else if (bmp_error == ERR_NONE && header_accepted &&
                             pixels_received == (image_width * image_height)) begin
                    frame_commit <= 1'b1;
                end else if (bmp_error == ERR_NONE) begin
                    bmp_error <= ERR_TRUNCATED;
                end
            end
        end
    end
endmodule
