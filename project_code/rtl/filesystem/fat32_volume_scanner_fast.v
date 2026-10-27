`timescale 1ns/1ps
// Resource-optimised FAT32 volume scanner.
//
// The original kept the whole 512-byte sector in registers and read directory
// entries through a dynamically indexed port, which synthesis turned into a
// ~3600-LUT read multiplexer. This version captures the boot-sector fields on
// the fly (fixed byte indices) and processes each 32-byte directory entry as
// its last byte arrives, so no sector buffer and no wide read mux are needed.
module fat32_volume_scanner_fast (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);

    localparam [7:0] ERR_NONE             = 8'h00;
    localparam [7:0] ERR_SECTOR           = 8'h10;
    localparam [7:0] ERR_BOOT_SIGNATURE   = 8'h20;
    localparam [7:0] ERR_UNSUPPORTED_BPB  = 8'h21;
    localparam [7:0] ERR_RESOURCE_MISSING = 8'h22;

    localparam [3:0] ST_IDLE        = 4'd0;
    localparam [3:0] ST_READ0_REQ   = 4'd1;
    localparam [3:0] ST_READ0_WAIT  = 4'd2;
    localparam [3:0] ST_BOOT_PARSE  = 4'd3;
    localparam [3:0] ST_BOOT_REQ    = 4'd4;
    localparam [3:0] ST_BOOT_WAIT   = 4'd5;
    localparam [3:0] ST_ROOT_REQ    = 4'd6;
    localparam [3:0] ST_ROOT_WAIT   = 4'd7;
    localparam [3:0] ST_FINISH      = 4'd9;
    localparam [3:0] ST_BOOT_LAYOUT = 4'd10;
    localparam [3:0] ST_BOOT_ROOT   = 4'd11;

    reg [3:0]  state;
    reg [31:0] root_cluster;
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [31:0] root_lba;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [7:0]  fat_count;
    reg [15:0] reserved_sectors;
    reg [2:0]  sectors_per_cluster_shift;
    reg [6:0]  root_sector_index;
    reg        dir_end;

    assign scan_busy = (state != ST_IDLE);

    // ------------------------------------------------------------------
    // Streaming capture: boot-sector fields at fixed indices plus one
    // 32-byte directory entry window.
    // ------------------------------------------------------------------
    reg [7:0] cap0,   cap11, cap12, cap13, cap14, cap15, cap16, cap17,
              cap18,  cap22, cap23, cap32, cap33, cap34, cap35, cap36,
              cap37,  cap38, cap39, cap44, cap45, cap46, cap47, cap450,
              cap454, cap455, cap456, cap457, cap510, cap511;
    reg [7:0] ent_byte [0:31];

    always @(posedge clk) begin
        if (sector_valid) begin
            case (sector_byte_index)
                9'd0:   cap0   <= sector_byte;
                9'd11:  cap11  <= sector_byte;
                9'd12:  cap12  <= sector_byte;
                9'd13:  cap13  <= sector_byte;
                9'd14:  cap14  <= sector_byte;
                9'd15:  cap15  <= sector_byte;
                9'd16:  cap16  <= sector_byte;
                9'd17:  cap17  <= sector_byte;
                9'd18:  cap18  <= sector_byte;
                9'd22:  cap22  <= sector_byte;
                9'd23:  cap23  <= sector_byte;
                9'd32:  cap32  <= sector_byte;
                9'd33:  cap33  <= sector_byte;
                9'd34:  cap34  <= sector_byte;
                9'd35:  cap35  <= sector_byte;
                9'd36:  cap36  <= sector_byte;
                9'd37:  cap37  <= sector_byte;
                9'd38:  cap38  <= sector_byte;
                9'd39:  cap39  <= sector_byte;
                9'd44:  cap44  <= sector_byte;
                9'd45:  cap45  <= sector_byte;
                9'd46:  cap46  <= sector_byte;
                9'd47:  cap47  <= sector_byte;
                9'd450: cap450 <= sector_byte;
                9'd454: cap454 <= sector_byte;
                9'd455: cap455 <= sector_byte;
                9'd456: cap456 <= sector_byte;
                9'd457: cap457 <= sector_byte;
                9'd510: cap510 <= sector_byte;
                9'd511: cap511 <= sector_byte;
                default: ;
            endcase
            ent_byte[sector_byte_index[4:0]] <= sector_byte;
        end
    end

    // Boot-sector views
    wire [15:0] bps      = {cap12, cap11};
    wire [15:0] rsvd     = {cap15, cap14};
    wire [15:0] root_ent = {cap18, cap17};
    wire [15:0] fatsz16  = {cap23, cap22};
    wire [31:0] totsec   = {cap35, cap34, cap33, cap32};
    wire [31:0] fatsz32  = {cap39, cap38, cap37, cap36};
    wire [31:0] rootclus = {cap47, cap46, cap45, cap44};
    wire [31:0] part_lba = {cap457, cap456, cap455, cap454};
    wire        boot_sig_ok = (cap510 == 8'h55) && (cap511 == 8'haa);

    // Directory-entry views (constant indices: no read mux)
    wire [7:0] eb0  = ent_byte[0];
    wire [7:0] eb1  = ent_byte[1];
    wire [7:0] eb2  = ent_byte[2];
    wire [7:0] eb3  = ent_byte[3];
    wire [7:0] eb4  = ent_byte[4];
    wire [7:0] eb5  = ent_byte[5];
    wire [7:0] eb6  = ent_byte[6];
    wire [7:0] eb7  = ent_byte[7];
    wire [7:0] eb8  = ent_byte[8];
    wire [7:0] eb9  = ent_byte[9];
    wire [7:0] eb10 = ent_byte[10];
    wire [7:0] eb11 = ent_byte[11];

    wire [31:0] ent_cluster = {ent_byte[21], ent_byte[20],
                               ent_byte[27], ent_byte[26]};
    wire [31:0] ent_size    = {ent_byte[31], ent_byte[30],
                               ent_byte[29], ent_byte[28]};
    wire [1:0]  ent_index   = eb3 - 8'd49;   // "1".."4"

    wire ent_match = (eb0  == "P") && (eb1 == "I") && (eb2 == "C") &&
                     (eb3  >= "1") && (eb3 <= "4") &&
                     (eb4  == " ") && (eb5 == " ") &&
                     (eb6  == " ") && (eb7 == " ") &&
                     (eb8  == "B") && (eb9 == "M") &&
                     (eb10 == "P") && (eb11 == 8'h20);

    wire [3:0] valid_after = resource_valid |
                             (ent_match ? (4'b0001 << ent_index) : 4'b0000);
    wire       at_ent_end  = sector_valid && (sector_byte_index[4:0] == 5'd31);

    // Evaluate one cycle after the last byte so ent_byte[31] is visible.
    reg        ent_ready;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ent_ready <= 1'b0;
        else        ent_ready <= at_ent_end;
    end

    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2:   spc_shift = 3'd1;
                8'd4:   spc_shift = 3'd2;
                8'd8:   spc_shift = 3'd3;
                8'd16:  spc_shift = 3'd4;
                8'd32:  spc_shift = 3'd5;
                8'd64:  spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = value != 0 && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            resource_valid <= 4'd0;
            resource0_cluster <= 32'd0;
            resource1_cluster <= 32'd0;
            resource2_cluster <= 32'd0;
            resource3_cluster <= 32'd0;
            resource0_size <= 32'd0;
            resource1_size <= 32'd0;
            resource2_size <= 32'd0;
            resource3_size <= 32'd0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            root_cluster <= 32'd0;
            total_sectors <= 32'd0;
            fat_sectors <= 32'd0;
            fat_count <= 8'd0;
            reserved_sectors <= 16'd0;
            root_lba <= 32'd0;
            data_sector_count <= 32'd0;
            fat_span <= 32'd0;
            sectors_per_cluster_shift <= 3'd0;
            root_sector_index <= 7'd0;
            dir_end <= 1'b0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;

            case (state)
                ST_IDLE: if (scan_start) begin
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    resource_valid <= 4'd0;
                    dir_end <= 1'b0;
                    state <= ST_READ0_REQ;
                end

                ST_READ0_REQ: if (!sector_busy) begin
                    sector_lba <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_READ0_WAIT;
                end

                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (!boot_sig_ok)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else if (cap0 == 8'heb || cap0 == 8'he9) begin
                        partition_lba <= 32'd0;
                        state <= ST_BOOT_PARSE;
                    end else if (cap450 == 8'h0b || cap450 == 8'h0c) begin
                        partition_lba <= part_lba;
                        sector_lba <= part_lba;
                        state <= ST_BOOT_REQ;
                    end else
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                end

                ST_BOOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_BOOT_WAIT;
                end

                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (!boot_sig_ok)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else
                        state <= ST_BOOT_PARSE;
                end

                ST_BOOT_PARSE: begin
                    if (bps != 16'd512 || !valid_spc(cap13) || rsvd == 16'd0 ||
                        (cap16 != 8'd1 && cap16 != 8'd2) || root_ent != 16'd0 ||
                        fatsz16 != 16'd0 || totsec == 32'd0 ||
                        fatsz32 == 32'd0 || rootclus < 32'd2)
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    else begin
                        sectors_per_cluster <= cap13;
                        reserved_sectors <= rsvd;
                        fat_count <= cap16;
                        total_sectors <= totsec;
                        fat_sectors <= fatsz32;
                        root_cluster <= rootclus;
                        sectors_per_cluster_shift <= spc_shift(cap13);
                        fat_span <= (cap16 == 8'd2) ? (fatsz32 << 1) : fatsz32;
                        fat_start_lba <= partition_lba + {16'd0, rsvd};
                        state <= ST_BOOT_LAYOUT;
                    end
                end

                ST_BOOT_LAYOUT: begin
                    data_start_lba <= partition_lba + {16'd0, reserved_sectors}
                                      + fat_span;
                    data_sector_count <= total_sectors
                                         - {16'd0, reserved_sectors} - fat_span;
                    state <= ST_BOOT_ROOT;
                end

                ST_BOOT_ROOT: begin
                    root_sector_index <= 7'd0;
                    dir_end <= 1'b0;
                    max_cluster <= (data_sector_count
                                    >> sectors_per_cluster_shift) + 32'd1;
                    root_lba <= data_start_lba
                                + ((root_cluster - 32'd2)
                                   << sectors_per_cluster_shift);
                    sector_lba <= data_start_lba
                                  + ((root_cluster - 32'd2)
                                     << sectors_per_cluster_shift);
                    state <= ST_ROOT_REQ;
                end

                ST_ROOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_ROOT_WAIT;
                end

                ST_ROOT_WAIT: begin
                    if (ent_ready) begin
                        if (eb0 == 8'h00) begin
                            dir_end <= 1'b1;
                            if (valid_after == 4'hf)
                                state <= ST_FINISH;
                        end else if (ent_match) begin
                            case (ent_index)
                                2'd0: begin
                                    resource0_cluster <= ent_cluster;
                                    resource0_size <= ent_size;
                                    resource_valid[0] <= 1'b1;
                                end
                                2'd1: begin
                                    resource1_cluster <= ent_cluster;
                                    resource1_size <= ent_size;
                                    resource_valid[1] <= 1'b1;
                                end
                                2'd2: begin
                                    resource2_cluster <= ent_cluster;
                                    resource2_size <= ent_size;
                                    resource_valid[2] <= 1'b1;
                                end
                                default: begin
                                    resource3_cluster <= ent_cluster;
                                    resource3_size <= ent_size;
                                    resource_valid[3] <= 1'b1;
                                end
                            endcase
                            if (valid_after == 4'hf)
                                state <= ST_FINISH;
                        end
                    end
                    if (sector_done) begin
                        if (sector_error != 0)
                            finish_with_error(ERR_SECTOR);
                        else if (valid_after == 4'hf)
                            state <= ST_FINISH;
                        else if (dir_end)
                            finish_with_error(ERR_RESOURCE_MISSING);
                        else if (({1'b0, root_sector_index} + 1'b1) <
                                 (8'd1 << sectors_per_cluster_shift)) begin
                            root_sector_index <= root_sector_index + 1'b1;
                            sector_lba <= root_lba +
                                          {25'd0, root_sector_index} + 1'b1;
                            dir_end <= 1'b0;
                            state <= ST_ROOT_REQ;
                        end else
                            finish_with_error(ERR_RESOURCE_MISSING);
                    end
                end

                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
