module oled_nilam (
    input  wire clk,
    input  wire reset_n,
    inout  wire oled_sda,
    inout  wire oled_scl,
	output wire [3:0] LED,
    output reg  done
);

assign LED = div_count[7:4];

    localparam integer DIVIDER = 125;

    reg [7:0] div_count;
    reg tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            div_count <= 0;
            tick      <= 0;
        end else begin
            if (div_count == DIVIDER-1) begin
                div_count <= 0;
                tick      <= 1;
            end else begin
                div_count <= div_count + 1'b1;
                tick      <= 0;
            end
        end
    end

    reg scl_low, sda_low;
    assign oled_scl = scl_low ? 1'b0 : 1'bz;
    assign oled_sda = sda_low ? 1'b0 : 1'bz;

    localparam
        I_IDLE       = 4'd0,
        I_START1     = 4'd1,
        I_START2     = 4'd2,
        I_BIT_LOW    = 4'd3,
        I_BIT_HIGH   = 4'd4,
        I_BIT_END    = 4'd5,
        I_ACK_LOW    = 4'd6,
        I_ACK_HIGH   = 4'd7,
        I_ACK_END    = 4'd8,
        I_STOP1      = 4'd9,
        I_STOP2      = 4'd10,
        I_STOP3      = 4'd11;

    reg [3:0] i2c_state;
    reg send, i2c_done;
    reg [7:0] control_byte, data_byte, shift;
    reg [2:0] bit_count;
    reg [1:0] byte_count;

    always @(posedge clk) begin
        i2c_done <= 1'b0;

        if (!reset_n) begin
            i2c_state <= I_IDLE;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            shift <= 8'h00;
            bit_count <= 3'd7;
            byte_count <= 2'd0;
        end else if (tick) begin
            case (i2c_state)
                I_IDLE: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    if (send) begin
                        shift <= 8'h78; // 0x3C << 1, write
                        bit_count <= 3'd7;
                        byte_count <= 2'd0;
                        i2c_state <= I_START1;
                    end
                end
                I_START1: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    i2c_state <= I_START2;
                end
                I_START2: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b1;
                    i2c_state <= I_BIT_LOW;
                end
                I_BIT_LOW: begin
                    scl_low <= 1'b1;
                    sda_low <= (shift[bit_count] == 1'b0);
                    i2c_state <= I_BIT_HIGH;
                end
                I_BIT_HIGH: begin
                    scl_low <= 1'b0;
                    i2c_state <= I_BIT_END;
                end
                I_BIT_END: begin
                    scl_low <= 1'b1;
                    if (bit_count == 0) begin
                        sda_low <= 1'b0;
                        i2c_state <= I_ACK_LOW;
                    end else begin
                        bit_count <= bit_count - 1'b1;
                        i2c_state <= I_BIT_LOW;
                    end
                end
                I_ACK_LOW: begin
                    scl_low <= 1'b1;
                    sda_low <= 1'b0;
                    i2c_state <= I_ACK_HIGH;
                end
                I_ACK_HIGH: begin
                    scl_low <= 1'b0;
                    i2c_state <= I_ACK_END;
                end
                I_ACK_END: begin
                    scl_low <= 1'b1;
                    if (byte_count == 0) begin
                        shift <= control_byte;
                        bit_count <= 3'd7;
                        byte_count <= 2'd1;
                        i2c_state <= I_BIT_LOW;
                    end else if (byte_count == 1) begin
                        shift <= data_byte;
                        bit_count <= 3'd7;
                        byte_count <= 2'd2;
                        i2c_state <= I_BIT_LOW;
                    end else begin
                        sda_low <= 1'b1;
                        i2c_state <= I_STOP1;
                    end
                end
                I_STOP1: begin
                    scl_low <= 1'b1;
                    sda_low <= 1'b1;
                    i2c_state <= I_STOP2;
                end
                I_STOP2: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b1;
                    i2c_state <= I_STOP3;
                end
                I_STOP3: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    i2c_done <= 1'b1;
                    i2c_state <= I_IDLE;
                end
                default: i2c_state <= I_IDLE;
            endcase
        end
    end

    localparam
        S_POWER_WAIT = 5'd0,
        S_INIT       = 5'd1,
        S_INIT_WAIT  = 5'd2,
        S_PAGE       = 5'd3,
        S_PAGE_WAIT  = 5'd4,
        S_COL_LOW    = 5'd5,
        S_COL_LOW_W  = 5'd6,
        S_COL_HIGH   = 5'd7,
        S_COL_HIGH_W = 5'd8,
        S_TEXT       = 5'd9,
        S_TEXT_WAIT  = 5'd10,
        S_DONE       = 5'd11;

    reg [4:0] state;
    reg [22:0] power_counter;
    reg [4:0] init_index;
    reg [5:0] text_index;

    function [7:0] init_command;
        input [4:0] index;
        begin
            case(index)
                 0: init_command = 8'hAE;
                 1: init_command = 8'hA8;
                 2: init_command = 8'h1F;
                 3: init_command = 8'hD3;
                 4: init_command = 8'h00;
                 5: init_command = 8'h40;
                 6: init_command = 8'hA1;
                 7: init_command = 8'hC8;
                 8: init_command = 8'hDA;
                 9: init_command = 8'h02;
                10: init_command = 8'h81;
                11: init_command = 8'h7F;
                12: init_command = 8'hA4;
                13: init_command = 8'hA6;
                14: init_command = 8'hD5;
                15: init_command = 8'h80;
                16: init_command = 8'h8D;
                17: init_command = 8'h14;
                18: init_command = 8'h20;
                19: init_command = 8'h02;
                20: init_command = 8'hD9;
                21: init_command = 8'hF1;
                22: init_command = 8'hDB;
                23: init_command = 8'h40;
                24: init_command = 8'hAF;
                default: init_command = 8'h00;
            endcase
        end
    endfunction

    function [7:0] text_data;
        input [5:0] index;
        begin
            case(index)
                 0: text_data = 8'h7F;  1: text_data = 8'h02;
                 2: text_data = 8'h04;  3: text_data = 8'h08;
                 4: text_data = 8'h7F;  5: text_data = 8'h00;
                 6: text_data = 8'h00;  7: text_data = 8'h41;
                 8: text_data = 8'h7F;  9: text_data = 8'h41;
                10: text_data = 8'h00; 11: text_data = 8'h00;
                12: text_data = 8'h7F; 13: text_data = 8'h40;
                14: text_data = 8'h40; 15: text_data = 8'h40;
                16: text_data = 8'h40; 17: text_data = 8'h00;
                18: text_data = 8'h7E; 19: text_data = 8'h11;
                20: text_data = 8'h11; 21: text_data = 8'h11;
                22: text_data = 8'h7E; 23: text_data = 8'h00;
                24: text_data = 8'h7F; 25: text_data = 8'h02;
                26: text_data = 8'h0C; 27: text_data = 8'h02;
                28: text_data = 8'h7F; 29: text_data = 8'h00;
                default: text_data = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        send <= 1'b0;

        if (!reset_n) begin
            state <= S_POWER_WAIT;
            power_counter <= 0;
            init_index <= 0;
            text_index <= 0;
            control_byte <= 8'h00;
            data_byte <= 8'h00;
            done <= 1'b0;
        end else begin
            case(state)
                S_POWER_WAIT: begin
                    done <= 1'b0;
                    if (power_counter == 5_000_000-1) begin
                        power_counter <= 0;
                        init_index <= 0;
                        state <= S_INIT;
                    end else
                        power_counter <= power_counter + 1'b1;
                end

                S_INIT: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        data_byte <= init_command(init_index);
                        send <= 1'b1;
                        state <= S_INIT_WAIT;
                    end
                end

                S_INIT_WAIT: begin
                    if (i2c_done) begin
                        if (init_index == 24)
                            state <= S_PAGE;
                        else begin
                            init_index <= init_index + 1'b1;
                            state <= S_INIT;
                        end
                    end
                end

                S_PAGE: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        data_byte <= 8'hB0;
                        send <= 1'b1;
                        state <= S_PAGE_WAIT;
                    end
                end

                S_PAGE_WAIT: if (i2c_done) state <= S_COL_LOW;

                S_COL_LOW: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        data_byte <= 8'h00;
                        send <= 1'b1;
                        state <= S_COL_LOW_W;
                    end
                end

                S_COL_LOW_W: if (i2c_done) state <= S_COL_HIGH;

                S_COL_HIGH: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        data_byte <= 8'h10;
                        send <= 1'b1;
                        state <= S_COL_HIGH_W;
                    end
                end

                S_COL_HIGH_W: begin
                    if (i2c_done) begin
                        text_index <= 0;
                        state <= S_TEXT;
                    end
                end

                S_TEXT: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h40;
                        data_byte <= text_data(text_index);
                        send <= 1'b1;
                        state <= S_TEXT_WAIT;
                    end
                end

                S_TEXT_WAIT: begin
                    if (i2c_done) begin
                        if (text_index == 29) begin
                            done <= 1'b1;
                            state <= S_DONE;
                        end else begin
                            text_index <= text_index + 1'b1;
                            state <= S_TEXT;
                        end
                    end
                end

                S_DONE: done <= 1'b1;
                default: state <= S_POWER_WAIT;
            endcase
        end
    end
endmodule
