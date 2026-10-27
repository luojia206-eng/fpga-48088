`timescale 1ns/1ps

// Top level: emergency bulletin playback terminal. TF/FAT32 ingest into the
// SDRAM frame cache, 640x480p60 HDMI with embedded audio, key interaction,
// layered graphics overlays and the UART status reporter (boot banner, per
// switch lines, parameter changes, error codes) on the board USB-UART.
module media_player_excellence_top (
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire       key2_n,
    input  wire       key3_n,
    input  wire       key4_n,
    output wire       uart_tx_o,
    input  wire       uart_rx_i,
    input  wire       uart_rx_ext_i,
    output wire       sd_cs_n,
    output wire       sd_sclk,
    output wire       sd_mosi,
    input  wire       sd_miso,
    input  wire       flash_miso,
    output wire       flash_cs_n,
    output wire       flash_sclk,
    output wire       flash_mosi,
    output wire       flash_wp_n,
    output wire       flash_hold_n,
    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,
    output wire       HDMI_DDC_SCL,
    inout  wire       HDMI_DDC_SDA,
    output wire [3:0] led,
    output wire [35:0] dbg_tx
);
    // Debug fan-out (temporary): the UART TX bit is mirrored onto every free IO
    // of the J2 40-pin expansion header, so the USB-TTL RX lead can be clamped
    // onto any convenient header pin without having to locate pin 1. Remove
    // this port once the header wiring is confirmed.
    assign dbg_tx = {36{uart_tx_o}};

    wire mem_feedback_clk;
    wire mem_clk;
    wire mem_clk_shift;
    wire mem_locked;
    sys_pll u_mem_pll (
        .refclk(sys_clk), .reset(~sys_rst_n),
        .clk0_out(mem_feedback_clk), .clk1_out(mem_clk),
        .clk2_out(mem_clk_shift), .locked(mem_locked)
    );

    wire pixel_clk_5x;
    wire pixel_clk;
    wire video_locked;
    clk_wize u_video_pll (
        .refclk(sys_clk), .reset(~sys_rst_n),
        .clk0_out(pixel_clk_5x), .clk1_out(pixel_clk),
        .locked(video_locked)
    );

    wire mem_rst_n;
    wire pix_rst_n;
    wire pix_5x_rst_n;
    wire ctrl_rst_n;
    reset_sync u_mem_reset (
        .clk(mem_clk), .ext_rst_n(sys_rst_n & mem_locked),
        .rst_n_sync(mem_rst_n)
    );
    reset_sync u_pix_reset (
        .clk(pixel_clk), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(pix_rst_n)
    );
    reset_sync u_pix_5x_reset (
        .clk(pixel_clk_5x), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(pix_5x_rst_n)
    );
    reset_sync u_ctrl_reset (
        .clk(sys_clk), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(ctrl_rst_n)
    );

    wire sdram_init_done;
    wire sdram_refresh_active;
    wire sdram_busy;
    wire ingest_app_wr_en;
    wire [20:0] ingest_app_wr_addr;
    wire [3:0] ingest_app_wr_mask;
    wire [31:0] ingest_app_wr_data;
    wire player_app_rd_en;
    wire [20:0] player_app_rd_addr;
    wire sdram_rd_valid;
    wire [31:0] sdram_rd_data;

    sdram u_sdram (
        .Clk(mem_clk), .Clk_sft(mem_clk_shift), .Rst(~mem_rst_n),
        .Sdr_init_done(sdram_init_done),
        .Sdr_init_ref_vld(sdram_refresh_active), .Sdr_busy(sdram_busy),
        .App_wr_en(ingest_app_wr_en), .App_wr_addr(ingest_app_wr_addr),
        .App_wr_dm(ingest_app_wr_mask), .App_wr_din(ingest_app_wr_data),
        .App_rd_en(player_app_rd_en), .App_rd_addr(player_app_rd_addr),
        .Sdr_rd_en(sdram_rd_valid), .Sdr_rd_dout(sdram_rd_data)
    );

    reg ingest_start;
    reg ingest_started;
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            ingest_start <= 1'b0;
            ingest_started <= 1'b0;
        end else begin
            ingest_start <= 1'b0;
            if (sdram_init_done && !ingest_started) begin
                ingest_start <= 1'b1;
                ingest_started <= 1'b1;
            end
        end
    end

    wire ingest_busy;
    wire ingest_done;
    wire [2:0] ingest_error_stage;
    wire [7:0] ingest_error_code;
    wire [7:0] ingest_error_detail;
    wire card_ready;
    wire card_high_capacity;
    wire [3:0] resource_valid;
    wire [3:0] image_complete;
    wire [15:0] ingest_src_w;
    wire [15:0] ingest_src_h;
    // Picture currently on screen, mirrored into mem_clk so the geometry cache
    // in the ingest chain can select the right entry.
    wire [1:0] active_index_mem;
    media_storage_ingest_top u_storage_ingest (
        .clk(mem_clk), .rst_n(mem_rst_n), .start(ingest_start),
        .busy(ingest_busy), .done(ingest_done),
        .error_stage(ingest_error_stage), .error_code(ingest_error_code),
        .error_detail(ingest_error_detail), .card_ready(card_ready),
        .card_high_capacity(card_high_capacity),
        .resource_valid(resource_valid), .image_complete(image_complete),
        .src_width(ingest_src_w), .src_height(ingest_src_h),
        .disp_index(active_index_mem),
        .controller_init_done(sdram_init_done),
        .controller_busy(sdram_busy), .refresh_active(sdram_refresh_active),
        .app_wr_en(ingest_app_wr_en), .app_wr_addr(ingest_app_wr_addr),
        .app_wr_mask(ingest_app_wr_mask), .app_wr_data(ingest_app_wr_data),
        .sd_cs_n(sd_cs_n), .sd_sclk(sd_sclk),
        .sd_mosi(sd_mosi), .sd_miso(sd_miso)
    );

    // Storage owns writes until the picture set is in place; playback then owns
    // only the read port, so the two stages never contend for SDRAM bandwidth.
    // (Starting playback after the first frame was tried and produced tearing
    // while the remaining frames were being written.)
    reg player_enable_mem;
    wire storage_error_mem;
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            player_enable_mem <= 1'b0;
        end else if (ingest_done) begin
            if (ingest_error_stage == 0 && ingest_error_code == 0 &&
                image_complete == 4'hf)
                player_enable_mem <= 1'b1;
            else if (image_complete != 4'h0)
                // Graceful degradation: play whatever committed and keep the
                // fault visible on the OSD instead of blanking the screen.
                player_enable_mem <= 1'b1;
        end
    end

    wire key2_press;
    wire key3_press;
    wire config_save_event;
    wire config_save_locked;
    wire config_ready;
    wire config_busy;
    wire config_load_done;
    wire config_save_done;
    wire config_apply_valid;
    wire config_rejected;
    wire config_fatal_error;
    wire [7:0] config_error_code;
    wire config_request_dropped;
    wire stored_config_valid;
    wire [31:0] stored_config_version;
    wire [31:0] config_period_ms;
    wire [31:0] config_period_cycles;
    wire [7:0] config_volume;
    wire [7:0] config_mode;
    wire [23:0] config_jedec_id;
    wire [7:0] config_status1, config_status2, config_status3;
    wire auto_enabled;

    // Human-operated chord contract: a single key acts on release; while the
    // first key remains held, the second key may join and start a 2 s save hold.
    wire key3_long_press;

    dual_key_chord_router #(
        .DEBOUNCE_CYCLES(1000000),
        .HOLD_CYCLES(100000000),
        .KEY3_LONG_CYCLES(50000000)
    ) u_key_router (
        .clk(sys_clk), .rst_n(ctrl_rst_n), .save_enabled(config_ready),
        .key2_n(key2_n), .key3_n(key3_n), .key2_event(key2_press),
        .key3_event(key3_press), .key3_long_event(key3_long_press),
        .save_event(config_save_event),
        .save_locked(config_save_locked), .key2_stable_n(),
        .key3_stable_n()
    );

    // Session-level brightness ladder: KEY3 held alone for one second and
    // released steps 100/75/50/25%% and back to 100%%. Not persisted, so the
    // Flash configuration record layout stays unchanged.
    reg [1:0] brightness_level;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            brightness_level <= 2'd0;
        else if (key3_long_press && config_ready)
            brightness_level <= brightness_level + 1'b1;
    end
    reg [7:0] brightness_sys;
    always @* begin
        case (brightness_level)
            2'd0: brightness_sys = 8'd255;
            2'd1: brightness_sys = 8'd192;
            2'd2: brightness_sys = 8'd128;
            default: brightness_sys = 8'd64;
        endcase
    end

    // KEY4 parameter menu: short press steps the volume ladder, a one
    // second hold steps the slideshow period ladder. The persistence service
    // samples the new value immediately; the dual-key chord still controls
    // the Flash write.
    wire menu_cfg_valid;
    wire [31:0] menu_cfg_period;
    wire [7:0] menu_cfg_volume;
    wire menu_evt;
    wire [1:0] menu_kind;
    wire [3:0] menu_level;

    param_menu_controller #(
        .CLKS_PER_MS(50000)
    ) u_param_menu (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .key4_n(key4_n), .service_ready(config_ready),
        .live_volume(config_volume), .live_period_ms(config_period_ms),
        .cfg_update_valid(menu_cfg_valid),
        .cfg_period_ms(menu_cfg_period), .cfg_volume(menu_cfg_volume),
        .menu_event(menu_evt), .menu_kind(menu_kind), .menu_level(menu_level)
    );

    // ===== Parameter feedback: transient level readout on the status OSD =====
    // The sys_clk side latches the label ('U' volume / 'P' period / 'L'
    // brightness) together with the new level and flips a toggle. The video
    // domain only has to detect that toggle, so no level bus crosses domains.
    // The parameter readout is permanent now (V3/5 B1/4), so the transient
    // banner is retired: the 5x7 font has no free code left for a 'P' label and
    // every character cell in the corner is already populated.
    wire [3:0] param_label_sys  = 4'hb;
    wire [3:0] param_digit_sys  = 4'hb;
    wire       param_toggle_sys = 1'b0;

    player_config_persistence u_player_config (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .cfg_update_valid((key3_press || menu_cfg_valid) && config_ready),
        .cfg_period_ms(menu_cfg_valid ? menu_cfg_period : config_period_ms),
        .cfg_volume(menu_cfg_valid ? menu_cfg_volume : config_volume),
        // mode[0] = auto slideshow; brightness stays session-only.
        .cfg_mode(key3_press ? {7'd0, ~auto_enabled} :
                              {7'd0, auto_enabled}),
        .save_request(config_save_event), .service_ready(config_ready),
        .busy(config_busy), .load_done(config_load_done),
        .save_done(config_save_done), .apply_valid(config_apply_valid),
        .config_rejected(config_rejected),
        .fatal_error(config_fatal_error), .error_code(config_error_code),
        .request_dropped(config_request_dropped),
        .stored_config_valid(stored_config_valid),
        .active_version(stored_config_version),
        .live_period_ms(config_period_ms),
        .live_period_cycles(config_period_cycles),
        .live_volume(config_volume), .live_mode(config_mode),
        .jedec_id(config_jedec_id), .status1(config_status1),
        .status2(config_status2), .status3(config_status3),
        .flash_miso(flash_miso), .flash_cs_n(flash_cs_n),
        .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
        .flash_wp_n(flash_wp_n), .flash_hold_n(flash_hold_n)
    );

    reg config_save_ok_latched;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            config_save_ok_latched <= 1'b0;
        else if (config_save_done && !config_fatal_error)
            config_save_ok_latched <= 1'b1;
    end

    wire [3:0] image_complete_sys;
    genvar bit_index;
    generate
        for (bit_index = 0; bit_index < 4; bit_index = bit_index + 1) begin : g_image_sync
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(image_complete[bit_index]),
                .sync_out(image_complete_sys[bit_index])
            );
        end
    endgenerate

    wire video_clk;
    wire video_hs;
    wire video_vs;
    wire video_de;
    wire [23:0] video_rgb;
    wire [9:0] video_x;
    wire [1:0] active_index;
    wire [1:0] pending_index;
    genvar disp_bit;
    generate
        for (disp_bit = 0; disp_bit < 2; disp_bit = disp_bit + 1) begin : g_disp_sync
            sync_2ff u_sync (
                .clk(mem_clk), .rst_n(mem_rst_n),
                .async_in(active_index[disp_bit]),
                .sync_out(active_index_mem[disp_bit])
            );
        end
    endgenerate
    wire frame_start;
    wire reader_busy;
    wire reader_underflow;
    wire [7:0] video_error;

    // FADE_FRAMES=16 gives roughly a 0.27 s ramp at 640x480@60 on both the
    // fade-out and fade-in halves of a manual switch.
    wire [7:0] brightness_pix;
    genvar bright_bit;
    generate
        for (bright_bit = 0; bright_bit < 8; bright_bit = bright_bit + 1) begin : g_bright_sync
            sync_2ff u_sync (
                .clk(pixel_clk), .rst_n(pix_rst_n),
                .async_in(brightness_sys[bright_bit]),
                .sync_out(brightness_pix[bright_bit])
            );
        end
    endgenerate

    media_video_pipeline #(
        .FADE_FRAMES(16)
    ) u_video (
        .brightness(brightness_pix),
        .ctrl_clk(sys_clk), .ctrl_rst_n(ctrl_rst_n),
        .next_event(key2_press), .toggle_auto_event(key3_press),
        .period_cfg_valid(config_apply_valid),
        .period_cfg_cycles(config_period_cycles),
        .auto_cfg_valid(config_apply_valid),
        .auto_cfg_enabled(config_mode[0]),
        .image_complete(image_complete_sys),
        .player_enable(player_enable_mem), .mem_clk(mem_clk),
        .mem_rst_n(mem_rst_n), .controller_init_done(sdram_init_done),
        .controller_busy(sdram_busy), .refresh_active(sdram_refresh_active),
        .app_rd_en(player_app_rd_en), .app_rd_addr(player_app_rd_addr),
        .sdram_rd_valid(sdram_rd_valid), .sdram_rd_data(sdram_rd_data),
        .pix_clk(pixel_clk), .pix_rst_n(pix_rst_n),
        .video_clk(video_clk), .video_hs(video_hs),
        .video_vs(video_vs),
        .video_de(video_de), .video_rgb(video_rgb), .video_x(video_x),
        .active_index(active_index),
        .pending_index(pending_index), .auto_enabled(auto_enabled),
        .frame_start(frame_start), .reader_busy(reader_busy),
        .reader_underflow(reader_underflow), .video_error(video_error)
    );

    // Audio-video linkage: the displayed image picks the sustained note and
    // the persisted volume scales it. Both are slow, frame-granular signals
    // crossing into the registered video clock domain.
    wire [1:0] active_index_video;
    wire [7:0] volume_video;
    genvar media_bit;
    generate
        for (media_bit = 0; media_bit < 2; media_bit = media_bit + 1) begin : g_media_sync
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(active_index[media_bit]),
                .sync_out(active_index_video[media_bit])
            );
        end
    endgenerate
    generate
        for (media_bit = 0; media_bit < 8; media_bit = media_bit + 1) begin : g_volume_sync
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(config_volume[media_bit]),
                .sync_out(volume_video[media_bit])
            );
        end
    endgenerate

    wire [9:0] audio_symbol0;
    wire [9:0] audio_symbol1;
    wire [9:0] audio_symbol2;
    wire audio_sequence_error;
    wire audio_tone_overflow;
    wire audio_pcm_contract_error;
    wire audio_phy_phase_error;
    wire audio_blip_active;

    // Audio faults surface as 8'h71 (distinct from the 0x6x reader family
    // and 0x5x control family) so the UART log and the status OSD report the
    // same failure the 4'hd LED shows.
    // Cross-domain status wires (declared before first use to keep
    // synthesis logs free of HDL-5373 used-before-declaration noise).
    wire player_enable_sys;
    wire storage_error_sys;
    wire video_fault_sys;
    wire audio_fault_sys;
    wire [3:0] video_error_sys;

    // Storage failures surface the real ingest error code (SD init
    // 0x01-0x0c, filesystem 0x10-0x22, decoder 0x31-0x38, loader 0x41-0x46)
    // so the OSD and UART distinguish "no card" (CMD0 timeout) from
    // "no FAT/no BMP"; the blanket 0xFE remains for code-less failures.
    wire [7:0] storage_reported_error;
    // Publish an immutable code and its valid flag together in sys_clk.
    // The previous direct ingest_error_code -> UART path crossed mem_clk
    // without a coherent snapshot and failed final setup timing.
    sticky_error_cdc u_storage_error_snapshot (
        .src_clk(mem_clk), .src_rst_n(mem_rst_n),
        .src_error(ingest_done &&
                   (ingest_error_stage != 0 || ingest_error_code != 0 ||
                    image_complete != 4'hf)),
        .src_code(ingest_error_code), .src_latched(storage_error_mem),
        .dst_clk(sys_clk), .dst_rst_n(ctrl_rst_n),
        .dst_valid(storage_error_sys), .dst_code(storage_reported_error)
    );
    wire [7:0] reported_error = storage_error_sys ? storage_reported_error :
                                 config_fatal_error ? config_error_code :
                                 video_fault_sys ? {4'b0, video_error_sys} :
                                 audio_fault_sys ? 8'h71 :
                                 8'd0;

    // Status OSD: bank number, auto mode and fault code rendered in the
    // top-left corner. Purely combinational, so the video_x/video_de
    // alignment feeding the HDMI Data Island scheduler is preserved.
    wire auto_enabled_video;
    wire [7:0] reported_error_video;
    sync_2ff u_auto_osd_sync (
        .clk(video_clk), .rst_n(pix_rst_n),
        .async_in(auto_enabled), .sync_out(auto_enabled_video)
    );
    genvar osd_bit;
    generate
        for (osd_bit = 0; osd_bit < 8; osd_bit = osd_bit + 1) begin : g_osd_err
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(reported_error[osd_bit]),
                .sync_out(reported_error_video[osd_bit])
            );
        end
    endgenerate

    wire [23:0] video_rgb_osd;
    wire video_hs_osd;
    wire video_vs_osd;
    wire video_de_osd;
    wire [9:0] video_x_osd;

    // Parameter readout hold: 2 s at the 25 MHz pixel clock.
    reg        param_tgl_v1, param_tgl_v2, param_tgl_seen;
    reg  [3:0] param_label_v, param_digit_v;
    reg [25:0] msg_timer;
    wire       param_new       = (param_tgl_v2 != param_tgl_seen);
    wire       osd_msg_active  = (msg_timer != 26'd0);

    // Persistent readout digits, 1-based so the customer never sees "level 0".
    wire [3:0] vol_digit_v = (volume_video >= 8'd255) ? 4'd5 :
                             (volume_video >= 8'd192) ? 4'd4 :
                             (volume_video >= 8'd128) ? 4'd3 :
                             (volume_video >=  8'd64) ? 4'd2 : 4'd1;
    wire [3:0] bright_digit_v = (brightness_pix >= 8'd255) ? 4'd1 :
                                (brightness_pix >= 8'd192) ? 4'd2 :
                                (brightness_pix >= 8'd128) ? 4'd3 : 4'd4;

    // ===== Second information line: source geometry and slideshow period =====
    // 12-bit binary to four BCD digits by double dabble. Registered once per
    // frame so the shift chain never sits in the pixel path.
    function [15:0] bin2bcd;
        input [11:0] value;
        integer i;
        reg [27:0] shift;
        begin
            shift = {16'd0, value};
            for (i = 0; i < 12; i = i + 1) begin
                if (shift[27:24] >= 4'd5) shift[27:24] = shift[27:24] + 4'd3;
                if (shift[23:20] >= 4'd5) shift[23:20] = shift[23:20] + 4'd3;
                if (shift[19:16] >= 4'd5) shift[19:16] = shift[19:16] + 4'd3;
                if (shift[15:12] >= 4'd5) shift[15:12] = shift[15:12] + 4'd3;
                shift = shift << 1;
            end
            bin2bcd = shift[27:12];
        end
    endfunction

    // Slideshow period in whole seconds. The four ladder steps are
    // 2 s / 5 s / 15 s / 30 s, so a compare ladder beats a divider here.
    wire [7:0] per_sec_sys = (config_period_ms >= 32'd30000) ? 8'd30 :
                             (config_period_ms >= 32'd15000) ? 8'd15 :
                             (config_period_ms >= 32'd5000)  ? 8'd5  : 8'd2;

    wire [39:0] info_bus_sys = {ingest_src_w, ingest_src_h, per_sec_sys};
    wire [39:0] info_bus_vid;
    genvar info_bit;
    generate
        for (info_bit = 0; info_bit < 40; info_bit = info_bit + 1) begin : g_info_sync
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(info_bus_sys[info_bit]),
                .sync_out(info_bus_vid[info_bit])
            );
        end
    endgenerate

    wire [15:0] src_w_vid  = info_bus_vid[39:24];
    wire [15:0] src_h_vid  = info_bus_vid[23:8];
    wire [7:0]  per_sec_vid = info_bus_vid[7:0];

    wire [3:0] per_tens = (per_sec_vid >= 8'd30) ? 4'd3 :
                          (per_sec_vid >= 8'd20) ? 4'd2 :
                          (per_sec_vid >= 8'd10) ? 4'd1 : 4'd0;
    // Tens digit of the slideshow period, and the units digit as value - tens*10.
    // The multiply is done with shifts ({t,3'b000} = t*8, {2'b00,t,1'b0} = t*2)
    // so no DSP is used. The previous expression only subtracted t*2+1, which
    // turned 30 s into 23 and printed a bogus "7" next to the "3".
    wire [7:0] per_tens_x10 = {1'b0, per_tens, 3'b000} + {2'b00, per_tens, 1'b0};
    wire [7:0] per_units_w  = per_sec_vid - per_tens_x10;
    wire [3:0] per_units    = per_units_w[3:0];

    reg [15:0] w_bcd_v, h_bcd_v;
    reg [7:0]  per_bcd_v;
    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            w_bcd_v <= 16'd0;
            h_bcd_v <= 16'd0;
            per_bcd_v <= 8'd0;
        end else if (frame_start) begin
            w_bcd_v <= bin2bcd(src_w_vid[11:0]);
            h_bcd_v <= bin2bcd(src_h_vid[11:0]);
            per_bcd_v <= {per_tens, per_units};
        end
    end

    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            param_tgl_v1   <= 1'b0;
            param_tgl_v2   <= 1'b0;
            param_tgl_seen <= 1'b0;
            param_label_v  <= 4'hb;
            param_digit_v  <= 4'hb;
            msg_timer      <= 26'd0;
        end else begin
            param_tgl_v1 <= param_toggle_sys;
            param_tgl_v2 <= param_tgl_v1;
            if (param_new) begin
                param_tgl_seen <= param_tgl_v2;
                param_label_v  <= param_label_sys;
                param_digit_v  <= param_digit_sys;
                msg_timer      <= 26'd49_999_999;
            end else if (msg_timer != 26'd0) begin
                msg_timer <= msg_timer - 1'b1;
            end
        end
    end

    // Status strip: everything on one line at y=24.
    //   x   8..104 : 图片号 / 自动轮播标志 / 错误码（8x5 字模）
    //   x 112..288 : 音量、亮度（12x12 中文点阵，11 个字符）
    //   x 288..528 : 分辨率、轮播周期（12x12 点阵，15 个字符）
    osd_overlay_24 #(.OSD_X(8), .OSD_Y(26)) u_osd (
        .clk(video_clk), .rst_n(pix_rst_n),
        .hsync(video_hs), .vsync(video_vs), .de(video_de),
        .rgb_in(video_rgb), .pixel_x(video_x),
        .active_index(active_index_video), .auto_enabled(auto_enabled_video),
        .error_code(reported_error_video),
        .msg_active(osd_msg_active),
        .msg_char0(param_label_v), .msg_char1(param_digit_v),
        .msg_char2(4'hb), .msg_char3(4'hb),
        .vol_digit(vol_digit_v), .bright_digit(bright_digit_v),
        .rgb_out(video_rgb_osd),
        .hsync_out(video_hs_osd), .vsync_out(video_vs_osd),
        .de_out(video_de_osd), .pixel_x_out(video_x_osd),
        .osd_active()
    );

    // E1 extension: scrolling subtitle banner. Registered stage - rgb and the
    // whole timing bundle shift by the same clock.
    // Chinese parameter banner sits under the status OSD: 音量3/5 亮度1/4.
    wire [23:0] video_rgb_par;
    wire video_hs_par;
    wire video_vs_par;
    wire video_de_par;
    wire [9:0] video_x_par;
    param_osd_12 #(
        .OSD_X(112), .OSD_Y(24), .TEXT_COLOR(24'h00E5FF)
    ) u_param_osd (
        .clk(video_clk), .rst_n(pix_rst_n),
        .hsync(video_hs_osd), .vsync(video_vs_osd), .de(video_de_osd),
        .pixel_x(video_x_osd), .rgb_in(video_rgb_osd),
        .vol_digit(vol_digit_v), .bright_digit(bright_digit_v),
        .hsync_out(video_hs_par), .vsync_out(video_vs_par),
        .de_out(video_de_par), .pixel_x_out(video_x_par),
        .rgb_out(video_rgb_par)
    );

    // Second information line: 640x480 and the slideshow period.
    wire [23:0] video_rgb_info;
    wire video_hs_info;
    wire video_vs_info;
    wire video_de_info;
    wire [9:0] video_x_info;
    info_osd_12 #(
        .OSD_X(288), .OSD_Y(24), .TEXT_COLOR(24'h00E5FF)
    ) u_info_osd (
        .clk(video_clk), .rst_n(pix_rst_n),
        .hsync(video_hs_par), .vsync(video_vs_par), .de(video_de_par),
        .pixel_x(video_x_par), .rgb_in(video_rgb_par),
        .w_bcd(w_bcd_v), .h_bcd(h_bcd_v), .per_bcd(per_bcd_v),
        .hsync_out(video_hs_info), .vsync_out(video_vs_info),
        .de_out(video_de_info), .pixel_x_out(video_x_info),
        .rgb_out(video_rgb_info)
    );

    // ===== Scrolling banner, plain text typed into the serial terminal =====
    // A serial assistant can only send bytes, so this path renders printable
    // ASCII with the 8x12 font ROM: type HELLO in the terminal and it shows up.
    // CR/LF clears the line (send with "new line" enabled), BS deletes, and
    // 0x0C clears everything. Chinese needs the preset/glyph paths instead,
    // because a runtime-typed Chinese string would need a full GB2312 font.
    wire [23:0] video_rgb_line;
    wire        video_hs_line;
    wire        video_vs_line;
    wire        video_de_line;
    wire [9:0]  video_x_line;
    wire        serial_line_active;
    // Both receive lines idle high, so a wired-AND lets either source drive the
    // receiver: F12 (on-board CH340 TX) and N4 (J2 header pin 2, for an external
    // USB-TTL module when the CH340 socket is blocked by the HDMI cable).
    wire uart_rx_combined = uart_rx_i & uart_rx_ext_i;
    serial_text_line #(
        .TEXT_Y(438), .SCROLL_STEP(2), .H_ACT(640), .LINE_CHARS(40),
        .CLKS_PER_BIT(217), .TEXT_COLOR(24'hFFFFFF)
    ) u_serial_line (
        .clk(video_clk), .rst_n(pix_rst_n), .uart_rx(uart_rx_combined),
        .hsync(video_hs_info), .vsync(video_vs_info), .de(video_de_info),
        .pixel_x(video_x_info), .rgb_in(video_rgb_info),
        .hsync_out(video_hs_line), .vsync_out(video_vs_line),
        .de_out(video_de_line), .pixel_x_out(video_x_line),
        .rgb_out(video_rgb_line), .text_active(serial_line_active)
    );

    // The vendor HDMI core owns the TMDS/audio path now: audio goes straight
    // from the AV-linked tone source below into I_audio_*, so the legacy
    // bridge/packetizers stay unused and their fault flags are tied off.
    assign audio_sequence_error = 1'b0;
    assign audio_tone_overflow = 1'b0;
    assign audio_pcm_contract_error = 1'b0;
    assign audio_phy_phase_error = 1'b0;

    // PCM source: 48 kHz, per-image melody, scaled by the persisted volume.
    wire               tone_valid;
    wire signed [15:0] tone_left;
    wire signed [15:0] tone_right;
    wire               tone_overflow;
    wire               tone_blip;
    pcm_media_tone #(.CLOCK_HZ(25000000)) u_pcm_tone (
        .clk(video_clk), .rst_n(pix_rst_n),
        .media_id(active_index_video), .volume(volume_video),
        .sample_valid(tone_valid), .sample_ready(1'b1),
        .sample_left(tone_left), .sample_right(tone_right),
        .overflow(tone_overflow), .blip_active(tone_blip)
    );

    // ACR rate: N=6144, CTS=25000 for 48 kHz at a 25 MHz pixel clock. One
    // pulse per ~1 ms of pixel clocks.
    reg [14:0] acr_period_cnt;
    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) acr_period_cnt <= 15'd0;
        else if (acr_period_cnt == 15'd24999) acr_period_cnt <= 15'd0;
        else acr_period_cnt <= acr_period_cnt + 1'b1;
    end
    wire acr_valid = (acr_period_cnt == 15'd0);

    // NOTE: the legacy HDMI audio bridge was removed for resource headroom;
    // audio can be fed to the vendor core later if the competition needs it.

    // ===== E5 extension: live audio waveform + level meter =====
    // Taps the 48 kHz PCM stream straight out of the tone generator (the same
    // stream that feeds the vendor core's I_audio_* port) and renders a 32-point
    // waveform with a peak bar. Everything here is a combinational RGB mux, so
    // the de/hs/vs/pixel_x bundle handed to the HDMI core is bit-identical to
    // the version without the overlay.
    wire [23:0] video_rgb_wave;
    wire        waveform_active;
    promax_audio_waveform #(
        .WAVE_X(64), .WAVE_Y(344),
        .WAVE_WIDTH(512), .WAVE_HEIGHT(64), .DECIMATE(6)
    ) u_waveform (
        .clk(video_clk), .rst_n(pix_rst_n),
        .vsync(video_vs_line), .de(video_de_line),
        .rgb_in(video_rgb_line), .pixel_x(video_x_line),
        .enable(1'b1), .volume(volume_video),
        .pcm_tap_valid(tone_valid), .pcm_tap_left(tone_left),
        .rgb_out(video_rgb_wave), .waveform_active(waveform_active)
    );

    // ===== Lightweight left-to-right reveal on every picture switch =====
    // Kept multiplier-free so it fits the remaining slice budget: the new
    // frame is revealed while the not-yet-revealed part stays black.
    reg  [1:0] wipe_index_q;
    reg  [9:0] wipe_pos;
    reg        wipe_on;
    reg        wipe_vs_q;

    wire wipe_switch = (active_index_video != wipe_index_q);
    wire wipe_frame  = wipe_vs_q && !video_vs_line;

    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            wipe_index_q <= 2'd0;
            wipe_pos     <= 10'd0;
            wipe_on      <= 1'b0;
            wipe_vs_q    <= 1'b1;
        end else begin
            wipe_index_q <= active_index_video;
            wipe_vs_q    <= video_vs_line;
            if (wipe_switch) begin
                wipe_pos <= 10'd0;
                wipe_on  <= 1'b1;
            end else if (wipe_frame && wipe_on) begin
                if (wipe_pos >= 10'd624)
                    wipe_on <= 1'b0;
                else
                    wipe_pos <= wipe_pos + 10'd16;
            end
        end
    end

    // Transition selection: media_video_pipeline already implements both
    // effects internally (alpha fade on manual switch, left-to-right reveal on
    // automatic slideshow, driven by FADE_FRAMES). Running the hand-written
    // reveal below on top of that produced two competing boundaries, so it is
    // kept switchable but disabled.
    localparam integer WIPE_ENABLE = 1'b0;
    wire [23:0] video_rgb_fx =
        (WIPE_ENABLE && wipe_on && (video_x_line >= wipe_pos)) ? 24'h000000 : video_rgb_wave;

    // ===== Vendor HDMI 1.4b transmitter (proven on this board) =====
    // The overlay pipeline below still produces one RGB pixel per pixel_clk;
    // this block converts that stream to AXI-Stream and lets the vendor core
    // do TMDS encoding, data islands and AVI infoframes.
    reg        axis_user;
    reg        axis_valid;
    reg        axis_last;
    reg [23:0] axis_data;
    reg        axis_vs_1d;
    reg        axis_de_1d;
    reg        axis_frame_arm;
    reg [10:0] axis_x_cnt;

    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            axis_vs_1d      <= 1'b0;
            axis_de_1d      <= 1'b0;
            axis_frame_arm  <= 1'b1;
            axis_x_cnt      <= 11'd0;
            axis_user       <= 1'b0;
            axis_valid      <= 1'b0;
            axis_last       <= 1'b0;
            axis_data       <= 24'd0;
        end else begin
            axis_vs_1d <= video_vs_line;
            axis_de_1d <= video_de_line;
            axis_user  <= 1'b0;
            axis_valid <= video_de_line;
            axis_last  <= 1'b0;
            axis_data  <= video_rgb_fx;

            if (axis_vs_1d != video_vs_line)
                axis_frame_arm <= 1'b1;

            if (video_de_line) begin
                if (!axis_de_1d) begin
                    axis_x_cnt <= 11'd0;
                    if (axis_frame_arm) begin
                        axis_user      <= 1'b1;
                        axis_frame_arm <= 1'b0;
                    end
                end else begin
                    axis_x_cnt <= axis_x_cnt + 11'd1;
                end

                if (axis_x_cnt == 11'd639)
                    axis_last <= 1'b1;
            end
        end
    end

    // One-shot EDID read request once the pixel domain is out of reset.
    reg [3:0] edid_trig_cnt;
    always @(posedge video_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) edid_trig_cnt <= 4'd0;
        else if (edid_trig_cnt != 4'hf) edid_trig_cnt <= edid_trig_cnt + 1'b1;
    end
    wire edid_trig = (edid_trig_cnt == 4'h8);

    wire [9:0] tmds_ch0_data;
    wire [9:0] tmds_ch1_data;
    wire [9:0] tmds_ch2_data;
    wire [9:0] tmds_clk_data;

    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE            ("EG"     ),
        .HTOTAL            (800      ),
        .HSA               (96       ),
        .HFP               (16       ),
        .HBP               (48       ),
        .HACTIVE           (640      ),
        .VTOTAL            (525      ),
        .VSA               (2        ),
        .VFP               (10       ),
        .VBP               (33       ),
        .VACTIVE           (480      ),
        .VIDEO_VIC         (1        ),
        .VIDEO_TPG         ("Disable"),
        .VIDEO_FORMAT      ("RGB"    ),
        .AUDIO_SAMPLE_RATE ("48K"    ),
        .IIC_SCL_DIV       (250      )
    ) u_hdmi_tx_core (
        .I_pixel_clk        (video_clk),
        .I_rst              (~pix_rst_n),
        .I_edid_read_trig   (edid_trig),
        .O_edid_read_valid  (),
        .O_edid_read_data   (),
        .I_axis_s_user      (axis_user),
        .I_axis_s_valid     (axis_valid),
        .I_axis_s_last      (axis_last),
        .I_axis_s_data      (axis_data),
        .O_axis_s_ready     (),
        .I_audio_valid      (tone_valid),
        .I_audio_left_data  ({{8{tone_left[15]}}, tone_left}),
        .I_audio_right_data ({{8{tone_right[15]}}, tone_right}),
        .I_acr_valid        (acr_valid),
        .I_acr_cts          (20'd25000),
        .I_acr_n            (20'd6144),
        .O_video_locked     (),
        .O_ddc_scl          (HDMI_DDC_SCL),
        .IO_ddc_sda         (HDMI_DDC_SDA),
        .O_ch0_tmds_data    (tmds_ch0_data),
        .O_ch1_tmds_data    (tmds_ch1_data),
        .O_ch2_tmds_data    (tmds_ch2_data),
        .O_clk_tmds_data    (tmds_clk_data)
    );

    hdmi_phy_wrapper #(.DEVICE("EG")) u_hdmi_phy (
        .I_pixel_clk        (video_clk),
        .I_serial_clk       (pixel_clk_5x),
        .I_rst              (~pix_rst_n),
        .I_tmds_channel_0   (tmds_ch0_data),
        .I_tmds_channel_1   (tmds_ch1_data),
        .I_tmds_channel_2   (tmds_ch2_data),
        .I_tmds_channel_clk (tmds_clk_data),
        .O_tmds_ch0_p       (HDMI_D0_P),
        .O_tmds_ch1_p       (HDMI_D1_P),
        .O_tmds_ch2_p       (HDMI_D2_P),
        .O_tmds_clk_p       (HDMI_CLK_P)
    );

    // The vendor HDMI core drives the TMDS lanes directly.
    sync_2ff u_player_enable_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(player_enable_mem), .sync_out(player_enable_sys)
    );
    sync_2ff u_video_fault_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(video_error != 0), .sync_out(video_fault_sys)
    );
    sync_2ff u_audio_fault_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(audio_sequence_error || audio_tone_overflow ||
                  audio_pcm_contract_error ||
                  audio_phy_phase_error),
        .sync_out(audio_fault_sys)
    );

    genvar error_bit_index;
    generate
        for (error_bit_index = 0; error_bit_index < 4;
             error_bit_index = error_bit_index + 1) begin : g_video_error_sync
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(video_error[error_bit_index]),
                .sync_out(video_error_sys[error_bit_index])
            );
        end
    endgenerate

    // UART status reporter: boot banner, image switch lines, parameter
    // menu changes and fault codes on the board USB-UART at 115200 8N1.
    wire [1:0] active_index_sys;
    genvar uart_bit;
    generate
        for (uart_bit = 0; uart_bit < 2; uart_bit = uart_bit + 1) begin : g_media_sys
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(active_index[uart_bit]),
                .sync_out(active_index_sys[uart_bit])
            );
        end
    endgenerate

    reg [1:0] active_index_sys_q;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            active_index_sys_q <= 2'd0;
        else
            active_index_sys_q <= active_index_sys;
    end
    wire switch_evt = (active_index_sys != active_index_sys_q);


    uart_status_reporter #(.CLKS_PER_BIT(434)) u_uart_report (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .enable(player_enable_sys), .media_id(active_index_sys),
        .volume(config_volume), .period_ms(config_period_ms),
        .auto_enabled(auto_enabled),
        .switch_event(switch_evt), .menu_event(menu_evt),
        .menu_kind(menu_kind), .error_code(reported_error),
        .bright_event(key3_long_press && config_ready),
        .bright_value(brightness_sys),
        .uart_tx(uart_tx_o)
    );

    // Competition-facing playback indicators with fault-code priority:
    // storage all-on, config code, audio 4'hD, video code, save-ok 4'b0101,
    // then the running bank/auto/auto state.

    // TEMPORARY clock diagnostic (DEBUG_CLK_LEDS=1):
    //   led[0] blinks from the 25 MHz pixel domain
    //   led[1] blinks from the 125 MHz clk_5x domain
    // If led[1] never blinks, clk_5x never reaches the HDMI PHY, which keeps
    // the PHY in reset and produces no TMDS clock on the connector.
    localparam DEBUG_CLK_LEDS = 1'b0;
    reg [24:0] dbg_pix_cnt;
    reg [26:0] dbg_x5_cnt;
    always @(posedge pixel_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) dbg_pix_cnt <= 25'd0;
        else            dbg_pix_cnt <= dbg_pix_cnt + 1'b1;
    end
    always @(posedge pixel_clk_5x or negedge pix_5x_rst_n) begin
        if (!pix_5x_rst_n) dbg_x5_cnt <= 27'd0;
        else               dbg_x5_cnt <= dbg_x5_cnt + 1'b1;
    end

    assign led = DEBUG_CLK_LEDS ? {1'b0, 1'b0, dbg_x5_cnt[26], dbg_pix_cnt[24]} :
                 storage_error_sys ? 4'hf :
                 config_fatal_error ? {1'b1,config_error_code[2:0]} :
                 audio_fault_sys ? 4'hd :
                 video_fault_sys ? video_error_sys :
                 config_save_ok_latched ? 4'b0101 :
                 player_enable_sys ? {1'b0, auto_enabled, active_index} :
                 {1'b0, 1'b0, &image_complete_sys,
                  mem_locked & video_locked};

endmodule
