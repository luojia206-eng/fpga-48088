`timescale 1ns/1ps

// Minimal 8N1 UART receiver.
//
// CLKS_PER_BIT must be the frequency of `clk` divided by the baud rate. The
// board link is 115200 8N1; this module is clocked from the 25 MHz pixel clock
// so CLKS_PER_BIT = 25000000 / 115200 = 217 (0.006 % baud error, well inside
// the 2 % budget).
//
// The input is double-registered first (the UART line is asynchronous to the
// pixel clock) and every bit is sampled at its centre by keeping a counter that
// reloads from CLKS_PER_BIT. rx_valid is a single-cycle pulse.
module uart_rx_byte #(
    parameter integer CLKS_PER_BIT = 217
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg        rx_valid,
    output reg  [7:0] rx_data
);
    localparam integer HALF = CLKS_PER_BIT / 2;

    // Two-flop synchroniser; the line idles high.
    reg [1:0] sync_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync_q <= 2'b11;
        else        sync_q <= {sync_q[0], rx};
    end
    wire rx_bit = sync_q[1];

    reg [15:0] cnt;
    reg [3:0]  state;
    reg [7:0]  shreg;

    localparam [3:0] ST_IDLE  = 4'd0;
    localparam [3:0] ST_START = 4'd1;
    localparam [3:0] ST_DATA  = 4'd2;   // 4'd2 .. 4'd9 = data bits 0..7
    localparam [3:0] ST_STOP  = 4'd10;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 16'd0;
            state    <= ST_IDLE;
            shreg    <= 8'd0;
            rx_valid <= 1'b0;
            rx_data  <= 8'd0;
        end else begin
            rx_valid <= 1'b0;
            case (state)
            ST_IDLE: begin
                if (!rx_bit) begin
                    // Jump straight to the middle of the start bit.
                    cnt   <= HALF - 1;
                    state <= ST_START;
                end
            end
            ST_START: begin
                if (cnt != 16'd0) begin
                    cnt <= cnt - 1'b1;
                end else if (rx_bit) begin
                    state <= ST_IDLE;               // false start, resync
                end else begin
                    cnt   <= CLKS_PER_BIT - 1;
                    state <= ST_DATA;
                end
            end
            // Data bits 0..7 live in states 2..9. A bare `ST_DATA:` label would
            // only match state 2, so every state must be listed explicitly.
            4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9: begin
                if (cnt != 16'd0) begin
                    cnt <= cnt - 1'b1;
                end else begin
                    shreg <= {rx_bit, shreg[7:1]};  // LSB first
                    cnt   <= CLKS_PER_BIT - 1;
                    state <= state + 1'b1;
                end
            end
            default: begin                          // centre of the stop bit
                if (cnt != 16'd0) begin
                    cnt <= cnt - 1'b1;
                end else begin
                    rx_valid <= 1'b1;
                    rx_data  <= shreg;
                    state    <= ST_IDLE;
                end
            end
            endcase
        end
    end
endmodule
