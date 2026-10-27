`timescale 1ns/1ps

module media_ingest_pipeline #(
    parameter integer IMAGE_WIDTH = 640,
    parameter integer IMAGE_HEIGHT = 480,
    parameter integer ADDR_WIDTH = 21,
    parameter integer FIFO_DEPTH = 512,
    parameter integer STALL_TIMEOUT = 65535,
    parameter integer USE_SCALER = 1
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,
    output wire                  busy,
    output reg                   done,
    output reg  [7:0]            ingest_error,
    output reg  [7:0]            ingest_detail_error,
    output reg  [1:0]            current_image,
    output wire [3:0]            resource_valid,
    output wire [3:0]            image_complete,
    // Source geometry of the last accepted BMP header, already registered in
    // the decoder, so this is a plain wire tap for the on-screen readout.
    output wire [15:0]           src_width,
    output wire [15:0]           src_height,

    output wire                  sector_req,
    output wire [31:0]           sector_lba,
    input  wire                  sector_busy,
    input  wire                  sector_valid,
    input  wire [7:0]            sector_byte,
    input  wire [8:0]            sector_byte_index,
    input  wire                  sector_done,
    input  wire [7:0]            sector_error,

    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    // Index of the picture currently on screen (already synced into this
    // clock domain). Selects which cached geometry the readout shows.
    input  wire [1:0]            disp_index,
    output wire                  app_wr_en,
    output wire [ADDR_WIDTH-1:0] app_wr_addr,
    output wire [3:0]            app_wr_mask,
    output wire [31:0]           app_wr_data
);
    localparam [15:0] NATIVE_W = IMAGE_WIDTH;
    localparam [15:0] NATIVE_H = IMAGE_HEIGHT;
    localparam integer IMAGE_WORDS = IMAGE_WIDTH * IMAGE_HEIGHT;
    localparam ERR_NONE   = 8'h00;
    localparam ERR_SCAN   = 8'h80;
    localparam ERR_FILE   = 8'h82;
    localparam ERR_BMP    = 8'h83;
    localparam ERR_MEMORY = 8'h84;
    localparam ERR_TIMEOUT = 8'h85;
    localparam [19:0] WD_LIMIT = 20'd4_000_000;   // ~100 ms without a sector

    localparam ST_IDLE      = 3'd0;
    localparam ST_SCAN      = 3'd1;
    localparam ST_FILE_START= 3'd2;
    localparam ST_FILE      = 3'd3;
    localparam ST_FAIL_WAIT = 3'd4;

    reg [2:0] state;
    reg scan_start;
    reg file_start;
    reg load_start;
    reg load_abort;
    reg [19:0] wd_count;
    reg wd_fired;
    reg retried;

    wire scan_busy;
    wire scan_done;
    wire [7:0] scan_error;
    wire [7:0] sectors_per_cluster;
    wire [31:0] fat_start_lba;
    wire [31:0] data_start_lba;
    wire [31:0] max_cluster;
    wire [31:0] resource0_cluster, resource1_cluster;
    wire [31:0] resource2_cluster, resource3_cluster;
    wire [31:0] resource0_size, resource1_size;
    wire [31:0] resource2_size, resource3_size;
    wire scan_sector_req;
    wire [31:0] scan_sector_lba;

    wire file_busy;
    wire file_done;
    wire file_valid;
    wire [7:0] file_byte;
    wire [31:0] file_byte_offset;
    wire [7:0] file_error;
    wire file_sector_req;
    wire [31:0] file_sector_lba;
    wire [31:0] selected_cluster;
    wire [31:0] selected_size;

    wire pixel_valid;
    wire [23:0] pixel_rgb;
    wire frame_commit;
    wire bmp_done;
    wire [7:0] bmp_error;
    wire load_busy;
    wire load_done;
    wire [7:0] memory_error;

    assign busy = state != ST_IDLE;
    assign sector_req = scan_sector_req | file_sector_req;
    assign sector_lba = scan_sector_req ? scan_sector_lba : file_sector_lba;

    assign selected_cluster = current_image == 2'd0 ? resource0_cluster :
                              current_image == 2'd1 ? resource1_cluster :
                              current_image == 2'd2 ? resource2_cluster : resource3_cluster;
    assign selected_size = current_image == 2'd0 ? resource0_size :
                           current_image == 2'd1 ? resource1_size :
                           current_image == 2'd2 ? resource2_size : resource3_size;

    fat32_volume_scanner_fast u_volume_scanner (
        .clk(clk), .rst_n(rst_n), .scan_start(scan_start), .scan_busy(scan_busy),
        .scan_done(scan_done), .fs_error(scan_error), .partition_lba(),
        .sectors_per_cluster(sectors_per_cluster), .fat_start_lba(fat_start_lba),
        .data_start_lba(data_start_lba), .max_cluster(max_cluster),
        .resource_valid(resource_valid),
        .resource0_cluster(resource0_cluster), .resource1_cluster(resource1_cluster),
        .resource2_cluster(resource2_cluster), .resource3_cluster(resource3_cluster),
        .resource0_size(resource0_size), .resource1_size(resource1_size),
        .resource2_size(resource2_size), .resource3_size(resource3_size),
        .sector_req(scan_sector_req), .sector_lba(scan_sector_lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(sector_done),
        .sector_error(sector_error)
    );

    fat32_file_streamer u_file_streamer (
        .clk(clk), .rst_n(rst_n), .file_start(file_start),
        .first_cluster(selected_cluster), .file_size(selected_size),
        .sectors_per_cluster(sectors_per_cluster), .fat_start_lba(fat_start_lba),
        .data_start_lba(data_start_lba), .max_cluster(max_cluster),
        .file_busy(file_busy), .file_done(file_done), .file_valid(file_valid),
        .file_byte(file_byte), .file_byte_offset(file_byte_offset), .fs_error(file_error),
        .sector_req(file_sector_req), .sector_lba(file_sector_lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(sector_done),
        .sector_error(sector_error)
    );

    wire top_down_w;
    wire header_accepted_w;
    wire bmp_commit_w;
    wire [15:0] image_width_w;
    wire [15:0] image_height_w;
    // Per-picture geometry cache. image_width_w only ever holds the LAST
    // decoded header, so a single latch would always report the final file;
    // keeping one entry per bank lets the readout follow the picture on screen.
    reg [15:0] geo_w0, geo_w1, geo_w2, geo_w3;
    reg [15:0] geo_h0, geo_h1, geo_h2, geo_h3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            geo_w0 <= 16'd0; geo_w1 <= 16'd0; geo_w2 <= 16'd0; geo_w3 <= 16'd0;
            geo_h0 <= 16'd0; geo_h1 <= 16'd0; geo_h2 <= 16'd0; geo_h3 <= 16'd0;
        end else if (header_accepted_w) begin
            case (current_image)
                2'd0: begin geo_w0 <= image_width_w; geo_h0 <= image_height_w; end
                2'd1: begin geo_w1 <= image_width_w; geo_h1 <= image_height_w; end
                2'd2: begin geo_w2 <= image_width_w; geo_h2 <= image_height_w; end
                default: begin geo_w3 <= image_width_w; geo_h3 <= image_height_w; end
            endcase
        end
    end

    reg [15:0] geo_w_sel, geo_h_sel;
    always @* begin
        case (disp_index)
            2'd0: begin geo_w_sel = geo_w0; geo_h_sel = geo_h0; end
            2'd1: begin geo_w_sel = geo_w1; geo_h_sel = geo_h1; end
            2'd2: begin geo_w_sel = geo_w2; geo_h_sel = geo_h2; end
            default: begin geo_w_sel = geo_w3; geo_h_sel = geo_h3; end
        endcase
    end

    assign src_width  = geo_w_sel;
    assign src_height = geo_h_sel;
    bmp24_decoder #(
        .EXPECTED_WIDTH(IMAGE_WIDTH), .EXPECTED_HEIGHT(IMAGE_HEIGHT)
    ) u_bmp_decoder (
        .clk(clk), .rst_n(rst_n), .file_start(file_start), .file_size(selected_size),
        .file_valid(file_valid), .file_byte(file_byte), .file_byte_offset(file_byte_offset),
        .file_end(file_done), .upstream_error(file_error),
        .header_accepted(header_accepted_w),
        .pixel_valid(pixel_valid), .pixel_rgb(pixel_rgb), .pixel_x(), .pixel_y(),
        .frame_commit(bmp_commit_w), .bmp_done(bmp_done), .bmp_error(bmp_error),
        .top_down(top_down_w), .pixel_offset(), .row_stride(),
        .image_width(image_width_w), .image_height(image_height_w)
    );

    // Extension (3): the decimator only handles pictures that are not already
    // at panel geometry. A native 640x480 image keeps the original direct path
    // so the scaler can never regress the plain case.
    wire        fit_valid;
    wire [23:0] fit_rgb;
    wire        fit_frame_done;
    bmp_scale_fit #(
        .DST_W(IMAGE_WIDTH), .DST_H(IMAGE_HEIGHT)
    ) u_fit (
        .clk(clk), .rst_n(rst_n), .frame_start(file_start),
        .dims_valid(header_accepted_w),
        .src_w(image_width_w), .src_h(image_height_w),
        .src_valid(pixel_valid), .src_rgb(pixel_rgb),
        .dst_valid(fit_valid), .dst_rgb(fit_rgb),
        .dst_frame_done(fit_frame_done)
    );

    wire        native_size  = (image_width_w == NATIVE_W) &&
                               (image_height_w == NATIVE_H);
    reg         native_size_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            native_size_q <= 1'b1;
        else if (header_accepted_w)
            native_size_q <= native_size;
        else
            native_size_q <= 1'b1;
    end

    wire        use_fit      = (USE_SCALER != 0) && !native_size_q;
    wire        load_valid_w = use_fit ? fit_valid : pixel_valid;
    wire [23:0] load_rgb_w   = use_fit ? fit_rgb   : pixel_rgb;

    assign frame_commit = use_fit ? fit_frame_done : bmp_commit_w;

    media_frame_loader #(
        .ADDR_WIDTH(ADDR_WIDTH), .IMAGE_WORDS(IMAGE_WORDS),
        .IMAGE_WIDTH(IMAGE_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH), .STALL_TIMEOUT(STALL_TIMEOUT)
    ) u_frame_loader (
        .clk(clk), .rst_n(rst_n), .load_start(load_start), .image_index(current_image),
        .load_abort(load_abort),
        .pixel_valid(load_valid_w), .pixel_rgb(load_rgb_w),
        .top_down(top_down_w), .frame_commit(frame_commit),
        .decode_error(bmp_error), .load_busy(load_busy), .load_done(load_done),
        .image_complete(image_complete), .memory_error(memory_error),
        .controller_init_done(controller_init_done), .controller_busy(controller_busy),
        .refresh_active(refresh_active), .app_wr_en(app_wr_en),
        .app_wr_addr(app_wr_addr), .app_wr_mask(app_wr_mask), .app_wr_data(app_wr_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_start <= 1'b0;
            file_start <= 1'b0;
            load_start <= 1'b0;
            load_abort <= 1'b0;
            done <= 1'b0;
            ingest_error <= ERR_NONE;
            ingest_detail_error <= ERR_NONE;
            current_image <= 2'd0;
            wd_count <= 20'd0;
            wd_fired <= 1'b0;
        end else begin
            scan_start <= 1'b0;
            file_start <= 1'b0;
            load_start <= 1'b0;
            load_abort <= 1'b0;
            done <= 1'b0;

            case (state)
                ST_IDLE: if (start) begin
                    ingest_error <= ERR_NONE;
                    ingest_detail_error <= ERR_NONE;
                    current_image <= 2'd0;
                    retried <= 1'b0;
                    scan_start <= 1'b1;
                    state <= ST_SCAN;
                end
                ST_SCAN: if (scan_done) begin
                    if (scan_error != 0) begin
                        ingest_error <= ERR_SCAN;
                        ingest_detail_error <= scan_error;
                        state <= ST_FAIL_WAIT;
                    end else begin
                        current_image <= 2'd0;
                        state <= ST_FILE_START;
                    end
                end
                ST_FILE_START: begin
                    file_start <= 1'b1;
                    load_start <= 1'b1;
                    wd_count <= 20'd0;
                    wd_fired <= 1'b0;
                    state <= ST_FILE;
                end                ST_FILE: begin
                    // Watchdog: a sector takes ~16k clocks at this SPI rate, so
                    // ~25 ms of silence means the read path is stuck. Abort the
                    // frame instead of hanging the whole player (the display
                    // keeps the previous picture and OSD/UART show 0x85).
                    if (!load_done) begin
                        if (sector_done)
                            wd_count <= 20'd0;
                        else if (wd_count == WD_LIMIT)
                            wd_fired <= 1'b1;
                        else
                            wd_count <= wd_count + 1'b1;
                    end
                    if (wd_fired && !load_done)
                        load_abort <= 1'b1;

                    if (load_done) begin
                        // One retry per picture: a stalled sector fetch (common
                        // straight after a large frame) usually succeeds on the
                        // second attempt.
                        if (!retried &&
                            (wd_fired || file_error != 0 || bmp_error != 0 ||
                             memory_error != 0)) begin
                            retried <= 1'b1;
                            state <= ST_FILE_START;
                        end else begin
                        // Record the fault but keep going: a single bad picture
                        // must not stop the other three from loading.
                        if (wd_fired) begin
                            ingest_error <= ERR_TIMEOUT;
                            ingest_detail_error <= 8'h47;
                        end else if (file_error != 0) begin
                            ingest_error <= ERR_FILE;
                            ingest_detail_error <= file_error;
                        end else if (bmp_error != 0) begin
                            ingest_error <= ERR_BMP;
                            ingest_detail_error <= bmp_error;
                        end else if (memory_error != 0) begin
                            ingest_error <= ERR_MEMORY;
                            ingest_detail_error <= memory_error;
                        end

                        if (current_image == 2'd3) begin
                            done <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            current_image <= current_image + 1'b1;
                            retried <= 1'b0;
                            state <= ST_FILE_START;
                        end
                        end
                    end
                end
                ST_FAIL_WAIT: if (!scan_busy && !file_busy && !load_busy) begin
                    done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
