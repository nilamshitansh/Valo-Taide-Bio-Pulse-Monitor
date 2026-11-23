module oled_nilam (
    input  wire       clk,       // DE10-Nano 50 MHz (PIN_V11)
    input  wire       reset_n,   // KEY0, active low
    input  wire       key1_n,    // KEY1, active low (Screen switch button - PIN_AH16)
    inout  wire       oled_sda,
    inout  wire       oled_scl,
    output wire [3:0] LED,
    output reg        done
);

    // ============================================================
    // 1. 50 MHz -> I2C timing tick (125 clocks = 2.5 us -> ~100 kHz I2C)
    // ============================================================
    localparam integer DIVIDER = 125;

    reg [7:0] div_count;
    reg       tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            div_count <= 8'd0;
            tick      <= 1'b0;
        end else begin
            if (div_count == DIVIDER - 1) begin
                div_count <= 8'd0;
                tick      <= 1'b1;
            end else begin
                div_count <= div_count + 1'b1;
                tick      <= 1'b0;
            end
        end
    end

    // ============================================================
    // 2. Pushbutton KEY1 Debouncer & Latched Screen Switch Request
    // ============================================================
    reg [19:0] debounce_cnt;
    reg        key1_sync0, key1_sync1;
    reg        key1_stable, key1_prev;
    reg [1:0]  screen_sel;
    reg        refresh_pending;
    reg        clear_phase;
    reg        clear_done;

    always @(posedge clk) begin
        if (!reset_n) begin
            key1_sync0      <= 1'b1;
            key1_sync1      <= 1'b1;
            key1_stable     <= 1'b1;
            key1_prev       <= 1'b1;
            debounce_cnt    <= 20'd0;
            screen_sel      <= 2'd0;
            refresh_pending <= 1'b0;
            clear_phase     <= 1'b0;
        end else begin
            key1_sync0 <= key1_n;
            key1_sync1 <= key1_sync0;

            if (key1_sync1 != key1_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt == 20'd1_000_000) begin // ~20 ms debounce at 50 MHz
                    key1_stable  <= key1_sync1;
                    debounce_cnt <= 20'd0;
                end
            end else begin
                debounce_cnt <= 20'd0;
            end

            key1_prev <= key1_stable;

            // Detect falling edge (KEY1 pressed) at ANY time, even mid-frame!
            if (key1_prev == 1'b1 && key1_stable == 1'b0) begin
                if (screen_sel == 2'd2)
                    screen_sel <= 2'd0;
                else
                    screen_sel <= screen_sel + 1'b1;

                // Latch request to wipe GDDRAM clean first, then draw new screen
                refresh_pending <= 1'b1;
                clear_phase     <= 1'b1;
            end else if (clear_done) begin
                // Once blank wipe completes, transition to drawing the new screen
                clear_phase     <= 1'b0;
                refresh_pending <= 1'b0;
            end
        end
    end

    // ============================================================
    // 3. ECG Scroll Timer (Scrolls waveform smoothly when on Screen 2)
    // ============================================================
    reg [20:0] scroll_timer;
    reg [6:0]  ecg_offset;
    reg        ecg_scroll_req;

    always @(posedge clk) begin
        if (!reset_n) begin
            scroll_timer   <= 21'd0;
            ecg_offset     <= 7'd0;
            ecg_scroll_req <= 1'b0;
        end else begin
            ecg_scroll_req <= 1'b0;
            if ((screen_sel == 2'd2) && !refresh_pending && !clear_phase) begin
                if (scroll_timer >= 21'd4_000_000) begin // ~80 ms per step (slower scroll)
                    scroll_timer   <= 21'd0;
                    ecg_offset     <= ecg_offset + 7'd1;
                    ecg_scroll_req <= 1'b1;
                end else begin
                    scroll_timer <= scroll_timer + 1'b1;
                end
            end else begin
                scroll_timer <= 21'd0;
                ecg_offset   <= 7'd0;
            end
        end
    end

    // ============================================================
    // 4. Open-Drain I2C Physical Lines
    // ============================================================
    reg scl_low;
    reg sda_low;

    assign oled_scl = scl_low ? 1'b0 : 1'bz;
    assign oled_sda = sda_low ? 1'b0 : 1'bz;

    // ============================================================
    // 5. I2C Transmitter (100 kHz Master FSM)
    // ============================================================
    localparam
        I_IDLE      = 4'd0,
        I_START1    = 4'd1,
        I_START2    = 4'd2,
        I_BIT_LOW   = 4'd3,
        I_BIT_HIGH  = 4'd4,
        I_BIT_END   = 4'd5,
        I_ACK_LOW   = 4'd6,
        I_ACK_HIGH  = 4'd7,
        I_ACK_END   = 4'd8,
        I_STOP1     = 4'd9,
        I_STOP2     = 4'd10,
        I_STOP3     = 4'd11;

    reg [3:0] i2c_state;
    reg       send;
    reg       i2c_done;
    reg [7:0] control_byte;
    reg [7:0] tx_byte;
    reg [7:0] shift;
    reg [2:0] bit_count;
    reg [1:0] byte_count;
    reg       address_ack_seen;
    reg       nack_seen;

    always @(posedge clk) begin
        i2c_done <= 1'b0;

        if (!reset_n) begin
            i2c_state        <= I_IDLE;
            scl_low          <= 1'b0;
            sda_low          <= 1'b0;
            shift            <= 8'h00;
            bit_count        <= 3'd7;
            byte_count       <= 2'd0;
            address_ack_seen <= 1'b0;
            nack_seen        <= 1'b0;
        end else begin
            if ((i2c_state == I_IDLE) && send) begin
                shift      <= 8'h78; // 0x3C << 1
                bit_count  <= 3'd7;
                byte_count <= 2'd0;
                i2c_state  <= I_START1;
            end else if (tick) begin
                case (i2c_state)
                    I_IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                    end
                    I_START1: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                        i2c_state <= I_START2;
                    end
                    I_START2: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b1;
                        i2c_state <= I_BIT_LOW;
                    end
                    I_BIT_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= (shift[bit_count] == 1'b0);
                        i2c_state <= I_BIT_HIGH;
                    end
                    I_BIT_HIGH: begin
                        scl_low   <= 1'b0;
                        i2c_state <= I_BIT_END;
                    end
                    I_BIT_END: begin
                        scl_low <= 1'b1;
                        if (bit_count == 0) begin
                            sda_low   <= 1'b0;
                            i2c_state <= I_ACK_LOW;
                        end else begin
                            bit_count <= bit_count - 1'b1;
                            i2c_state <= I_BIT_LOW;
                        end
                    end
                    I_ACK_LOW: begin
                        scl_low   <= 1'b1;
                        sda_low   <= 1'b0;
                        i2c_state <= I_ACK_HIGH;
                    end
                    I_ACK_HIGH: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                        i2c_state <= I_ACK_END;
                    end
                    I_ACK_END: begin
                        if (oled_sda == 1'b1)
                            nack_seen <= 1'b1;
                        if ((byte_count == 0) && (oled_sda == 1'b0))
                            address_ack_seen <= 1'b1;

                        scl_low <= 1'b1;
                        if (byte_count == 0) begin
                            shift      <= control_byte;
                            bit_count  <= 3'd7;
                            byte_count <= 2'd1;
                            i2c_state  <= I_BIT_LOW;
                        end else if (byte_count == 1) begin
                            shift      <= tx_byte;
                            bit_count  <= 3'd7;
                            byte_count <= 2'd2;
                            i2c_state  <= I_BIT_LOW;
                        end else begin
                            sda_low   <= 1'b1;
                            i2c_state <= I_STOP1;
                        end
                    end
                    I_STOP1: begin
                        scl_low   <= 1'b1;
                        sda_low   <= 1'b1;
                        i2c_state <= I_STOP2;
                    end
                    I_STOP2: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b1;
                        i2c_state <= I_STOP3;
                    end
                    I_STOP3: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                        i2c_done  <= 1'b1;
                        i2c_state <= I_IDLE;
                    end
                    default: begin
                        i2c_state <= I_IDLE;
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                    end
                endcase
            end
        end
    end

    // ============================================================
    // 6. SSD1306 128x32 Initialization ROM
    // ============================================================
    function [7:0] init_command;
        input [4:0] index;
        begin
            case (index)
                 0: init_command = 8'hAE; // Display OFF
                 1: init_command = 8'hD5;
                 2: init_command = 8'h80;
                 3: init_command = 8'hA8;
                 4: init_command = 8'h1F; // 32 rows
                 5: init_command = 8'hD3;
                 6: init_command = 8'h00;
                 7: init_command = 8'h40;
                 8: init_command = 8'h8D;
                 9: init_command = 8'h14; // Charge pump ON
                10: init_command = 8'h20;
                11: init_command = 8'h00; // Horizontal addressing
                12: init_command = 8'hA1;
                13: init_command = 8'hC8;
                14: init_command = 8'hDA;
                15: init_command = 8'h02; // 128x32 COM pins
                16: init_command = 8'h81;
                17: init_command = 8'h7F;
                18: init_command = 8'hD9;
                19: init_command = 8'hF1;
                20: init_command = 8'hDB;
                21: init_command = 8'h40;
                22: init_command = 8'hA4;
                23: init_command = 8'hA6;
                24: init_command = 8'hAF; // Display ON
                default: init_command = 8'hAE;
            endcase
        end
    endfunction

    // ============================================================
    // 7. Complete 5x7 ASCII Font ROM
    // ============================================================
    function [7:0] glyph;
        input [7:0] ch;
        input [2:0] col;
        begin
            glyph = 8'h00;
            case (ch)
                "A": case(col) 0:glyph=8'h7E; 1:glyph=8'h09; 2:glyph=8'h09; 3:glyph=8'h09; 4:glyph=8'h7E; endcase
                "B": case(col) 0:glyph=8'h7F; 1:glyph=8'h49; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h36; endcase
                "C": case(col) 0:glyph=8'h3E; 1:glyph=8'h41; 2:glyph=8'h41; 3:glyph=8'h41; 4:glyph=8'h22; endcase
                "D": case(col) 0:glyph=8'h7F; 1:glyph=8'h41; 2:glyph=8'h41; 3:glyph=8'h22; 4:glyph=8'h1C; endcase
                "E": case(col) 0:glyph=8'h7F; 1:glyph=8'h49; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h41; endcase
                "G": case(col) 0:glyph=8'h3E; 1:glyph=8'h41; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h7A; endcase
                "H": case(col) 0:glyph=8'h7F; 1:glyph=8'h08; 2:glyph=8'h08; 3:glyph=8'h08; 4:glyph=8'h7F; endcase
                "I": case(col) 0:glyph=8'h41; 1:glyph=8'h41; 2:glyph=8'h7F; 3:glyph=8'h41; 4:glyph=8'h41; endcase
                "K": case(col) 0:glyph=8'h7F; 1:glyph=8'h08; 2:glyph=8'h14; 3:glyph=8'h22; 4:glyph=8'h41; endcase
                "L": case(col) 0:glyph=8'h7F; 1:glyph=8'h40; 2:glyph=8'h40; 3:glyph=8'h40; 4:glyph=8'h40; endcase
                "M": case(col) 0:glyph=8'h7F; 1:glyph=8'h02; 2:glyph=8'h0C; 3:glyph=8'h02; 4:glyph=8'h7F; endcase
                "N": case(col) 0:glyph=8'h7F; 1:glyph=8'h06; 2:glyph=8'h0C; 3:glyph=8'h30; 4:glyph=8'h7F; endcase
                "O": case(col) 0:glyph=8'h3E; 1:glyph=8'h41; 2:glyph=8'h41; 3:glyph=8'h41; 4:glyph=8'h3E; endcase
                "P": case(col) 0:glyph=8'h7F; 1:glyph=8'h09; 2:glyph=8'h09; 3:glyph=8'h09; 4:glyph=8'h06; endcase
                "R": case(col) 0:glyph=8'h7F; 1:glyph=8'h09; 2:glyph=8'h19; 3:glyph=8'h29; 4:glyph=8'h46; endcase
                "S": case(col) 0:glyph=8'h46; 1:glyph=8'h49; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h31; endcase
                "T": case(col) 0:glyph=8'h01; 1:glyph=8'h01; 2:glyph=8'h7F; 3:glyph=8'h01; 4:glyph=8'h01; endcase
                "U": case(col) 0:glyph=8'h3F; 1:glyph=8'h40; 2:glyph=8'h40; 3:glyph=8'h40; 4:glyph=8'h3F; endcase
                "V": case(col) 0:glyph=8'h1F; 1:glyph=8'h20; 2:glyph=8'h40; 3:glyph=8'h20; 4:glyph=8'h1F; endcase
                "W": case(col) 0:glyph=8'h3F; 1:glyph=8'h40; 2:glyph=8'h38; 3:glyph=8'h40; 4:glyph=8'h3F; endcase
                "X": case(col) 0:glyph=8'h63; 1:glyph=8'h14; 2:glyph=8'h08; 3:glyph=8'h14; 4:glyph=8'h63; endcase
                "Y": case(col) 0:glyph=8'h07; 1:glyph=8'h08; 2:glyph=8'h70; 3:glyph=8'h08; 4:glyph=8'h07; endcase
                "a": case(col) 0:glyph=8'h20; 1:glyph=8'h54; 2:glyph=8'h54; 3:glyph=8'h54; 4:glyph=8'h78; endcase
                "b": case(col) 0:glyph=8'h7F; 1:glyph=8'h48; 2:glyph=8'h48; 3:glyph=8'h48; 4:glyph=8'h30; endcase
                "e": case(col) 0:glyph=8'h38; 1:glyph=8'h54; 2:glyph=8'h54; 3:glyph=8'h54; 4:glyph=8'h18; endcase
                "i": case(col) 0:glyph=8'h00; 1:glyph=8'h44; 2:glyph=8'h7D; 3:glyph=8'h40; 4:glyph=8'h00; endcase
                "l": case(col) 0:glyph=8'h00; 1:glyph=8'h41; 2:glyph=8'h7F; 3:glyph=8'h40; 4:glyph=8'h00; endcase
                "m": case(col) 0:glyph=8'h7C; 1:glyph=8'h04; 2:glyph=8'h18; 3:glyph=8'h04; 4:glyph=8'h78; endcase
                "n": case(col) 0:glyph=8'h7C; 1:glyph=8'h04; 2:glyph=8'h04; 3:glyph=8'h04; 4:glyph=8'h78; endcase
                "0": case(col) 0:glyph=8'h3E; 1:glyph=8'h51; 2:glyph=8'h49; 3:glyph=8'h45; 4:glyph=8'h3E; endcase
                "1": case(col) 0:glyph=8'h00; 1:glyph=8'h42; 2:glyph=8'h7F; 3:glyph=8'h40; 4:glyph=8'h00; endcase
                "2": case(col) 0:glyph=8'h62; 1:glyph=8'h51; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h46; endcase
                "3": case(col) 0:glyph=8'h22; 1:glyph=8'h41; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h36; endcase
                "4": case(col) 0:glyph=8'h18; 1:glyph=8'h14; 2:glyph=8'h12; 3:glyph=8'h7F; 4:glyph=8'h10; endcase
                "6": case(col) 0:glyph=8'h3C; 1:glyph=8'h4A; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h30; endcase
                "7": case(col) 0:glyph=8'h01; 1:glyph=8'h71; 2:glyph=8'h09; 3:glyph=8'h05; 4:glyph=8'h03; endcase
                "8": case(col) 0:glyph=8'h36; 1:glyph=8'h49; 2:glyph=8'h49; 3:glyph=8'h49; 4:glyph=8'h36; endcase
                ".": case(col) 0:glyph=8'h00; 1:glyph=8'h60; 2:glyph=8'h60; 3:glyph=8'h00; 4:glyph=8'h00; endcase
                "-": case(col) 0:glyph=8'h08; 1:glyph=8'h08; 2:glyph=8'h08; 3:glyph=8'h08; 4:glyph=8'h08; endcase
                ":": case(col) 0:glyph=8'h00; 1:glyph=8'h36; 2:glyph=8'h36; 3:glyph=8'h00; 4:glyph=8'h00; endcase
                "#": case(col) 0:glyph=8'h14; 1:glyph=8'h7F; 2:glyph=8'h14; 3:glyph=8'h7F; 4:glyph=8'h14; endcase
                ">": case(col) 0:glyph=8'h00; 1:glyph=8'h41; 2:glyph=8'h22; 3:glyph=8'h14; 4:glyph=8'h08; endcase
                "[": case(col) 0:glyph=8'h00; 1:glyph=8'h7F; 2:glyph=8'h41; 3:glyph=8'h41; 4:glyph=8'h00; endcase
                "]": case(col) 0:glyph=8'h00; 1:glyph=8'h41; 2:glyph=8'h41; 3:glyph=8'h7F; 4:glyph=8'h00; endcase
                default: glyph = 8'h00;
            endcase
        end
    endfunction

    // ============================================================
    // 8. Text Character Lookup
    //    Screen 0 = Valo-Taide Welcome
    //    Screen 1 = Patient Details & Vitals
    // ============================================================
    function [7:0] text_char;
        input [1:0] scr;
        input [1:0] page;
        input [4:0] index;
        begin
            text_char = " ";
            if (scr == 2'd0) begin
                // SCREEN 0: Valo-Taide Event & Interactive Welcome Screen
                case (page)
                    0: case (index) // "TEAM: VALO-TAIDE" (16 chars)
                         0:text_char="T";  1:text_char="E";  2:text_char="A";  3:text_char="M";
                         4:text_char=":";  5:text_char=" ";  6:text_char="V";  7:text_char="A";
                         8:text_char="L";  9:text_char="O"; 10:text_char="-"; 11:text_char="T";
                        12:text_char="A"; 13:text_char="I"; 14:text_char="D"; 15:text_char="E";
                        default:text_char=" ";
                    endcase
                    1: case (index) // "REUSE. REPURPOSE." (17 chars)
                         0:text_char="R";  1:text_char="E";  2:text_char="U";  3:text_char="S";
                         4:text_char="E";  5:text_char=".";  6:text_char=" ";  7:text_char="R";
                         8:text_char="E";  9:text_char="P"; 10:text_char="U"; 11:text_char="R";
                        12:text_char="P"; 13:text_char="O"; 14:text_char="S"; 15:text_char="E";
                        16:text_char="."; default:text_char=" ";
                    endcase
                    2: case (index) // "BIO-PULSE MONITOR" (17 chars)
                         0:text_char="B";  1:text_char="I";  2:text_char="O";  3:text_char="-";
                         4:text_char="P";  5:text_char="U";  6:text_char="L";  7:text_char="S";
                         8:text_char="E";  9:text_char=" "; 10:text_char="M"; 11:text_char="O";
                        12:text_char="N"; 13:text_char="I"; 14:text_char="T"; 15:text_char="O";
                        16:text_char="R"; default:text_char=" ";
                    endcase
                    3: case (index) // "[PRESS KEY1 -> NEXT]" (20 chars)
                         0:text_char="[";  1:text_char="P";  2:text_char="R";  3:text_char="E";
                         4:text_char="S";  5:text_char="S";  6:text_char=" ";  7:text_char="K";
                         8:text_char="E";  9:text_char="Y"; 10:text_char="1"; 11:text_char=" ";
                        12:text_char="-"; 13:text_char=">"; 14:text_char=" "; 15:text_char="N";
                        16:text_char="E"; 17:text_char="X"; 18:text_char="T"; 19:text_char="]";
                        default:text_char=" ";
                    endcase
                endcase
            end else begin
                // SCREEN 1: Patient Info & Vitals Layout
                case (page)
                    0: case (index) // "PATIENT ID: #042" (16 chars)
                         0:text_char="P";  1:text_char="A";  2:text_char="T";  3:text_char="I";
                         4:text_char="E";  5:text_char="N";  6:text_char="T";  7:text_char=" ";
                         8:text_char="I";  9:text_char="D"; 10:text_char=":"; 11:text_char=" ";
                        12:text_char="#"; 13:text_char="0"; 14:text_char="4"; 15:text_char="2";
                        default:text_char=" ";
                    endcase
                    1: case (index) // "NAME: NILAMBEN" (14 chars)
                         0:text_char="N";  1:text_char="A";  2:text_char="M";  3:text_char="E";
                         4:text_char=":";  5:text_char=" ";  6:text_char="N";  7:text_char="I";
                         8:text_char="L";  9:text_char="A"; 10:text_char="M"; 11:text_char="B";
                        12:text_char="E"; 13:text_char="N"; default:text_char=" ";
                    endcase
                    2: case (index) // "HR: 72 BPM  OK" (14 chars)
                         0:text_char="H";  1:text_char="R";  2:text_char=":";  3:text_char=" ";
                         4:text_char="7";  5:text_char="2";  6:text_char=" ";  7:text_char="B";
                         8:text_char="P";  9:text_char="M"; 10:text_char=" "; 11:text_char=" ";
                        12:text_char="O"; 13:text_char="K"; default:text_char=" ";
                    endcase
                    3: case (index) // "TEMP: 21.0 C" (12 chars)
                         0:text_char="T";  1:text_char="E";  2:text_char="M";  3:text_char="P";
                         4:text_char=":";  5:text_char=" ";  6:text_char="2";  7:text_char="1";
                         8:text_char=".";  9:text_char="0"; 10:text_char=" "; 11:text_char="C";
                        default:text_char=" ";
                    endcase
                endcase
            end
        end
    endfunction

    // ============================================================
    // 9. ECG 128x32 Waveform Generator (Screen 2) — original compact
    // ============================================================
    function [7:0] ecg_column_byte;
        input [1:0] page;
        input [6:0] col;
        reg [5:0] phase;
        reg [4:0] y_curr, y_prev, y_min, y_max;
        reg [31:0] col_mask;
        integer r;
        begin
            phase = col[5:0];

            case (phase)
                6'd6:  y_curr = 5'd19;
                6'd7:  y_curr = 5'd17;
                6'd8:  y_curr = 5'd16;
                6'd9:  y_curr = 5'd17;
                6'd10: y_curr = 5'd19;
                6'd16: y_curr = 5'd23;
                6'd17: y_curr = 5'd12;
                6'd18: y_curr = 5'd2;
                6'd19: y_curr = 5'd14;
                6'd20: y_curr = 5'd29;
                6'd21: y_curr = 5'd24;
                6'd28: y_curr = 5'd18;
                6'd29: y_curr = 5'd15;
                6'd30: y_curr = 5'd14;
                6'd31: y_curr = 5'd14;
                6'd32: y_curr = 5'd16;
                6'd33: y_curr = 5'd19;
                default: y_curr = 5'd20;
            endcase

            case (phase - 6'd1)
                6'd6:  y_prev = 5'd19;
                6'd7:  y_prev = 5'd17;
                6'd8:  y_prev = 5'd16;
                6'd9:  y_prev = 5'd17;
                6'd10: y_prev = 5'd19;
                6'd16: y_prev = 5'd23;
                6'd17: y_prev = 5'd12;
                6'd18: y_prev = 5'd2;
                6'd19: y_prev = 5'd14;
                6'd20: y_prev = 5'd29;
                6'd21: y_prev = 5'd24;
                6'd28: y_prev = 5'd18;
                6'd29: y_prev = 5'd15;
                6'd30: y_prev = 5'd14;
                6'd31: y_prev = 5'd14;
                6'd32: y_prev = 5'd16;
                6'd33: y_prev = 5'd19;
                default: y_prev = 5'd20;
            endcase

            y_min = (y_curr < y_prev) ? y_curr : y_prev;
            y_max = (y_curr > y_prev) ? y_curr : y_prev;

            col_mask = 32'd0;
            for (r = 0; r < 32; r = r + 1) begin
                if ((r >= y_min) && (r <= y_max))
                    col_mask = col_mask | (32'd1 << r);
            end

            case (page)
                2'd0: ecg_column_byte = col_mask[7:0];
                2'd1: ecg_column_byte = col_mask[15:8];
                2'd2: ecg_column_byte = col_mask[23:16];
                2'd3: ecg_column_byte = col_mask[31:24];
            endcase
        end
    endfunction

    // ============================================================
    // 10. Unified Framebuffer Generator
    //     When clear_phase == 1, outputs 0x00 to wipe GDDRAM completely!
    // ============================================================
    function [7:0] framebuffer;
        input       clr;
        input [1:0] scr;
        input [1:0] page;
        input [6:0] column;
        input [6:0] scroll_off;

        integer pos;
        integer char_index;
        integer char_column;
        integer offset;
        integer char_count;
        begin
            framebuffer = 8'h00;

            if (clr) begin
                // Full-screen blanking pass
                framebuffer = 8'h00;
            end else if (scr == 2'd2) begin
                // Screen 2: Scrolling ECG Waveform
                framebuffer = ecg_column_byte(page, column + scroll_off);
            end else begin
                if (scr == 2'd0) begin
                    // Screen 0 centering offsets
                    case (page)
                        0: begin offset = 16; char_count = 16; end // TEAM: VALO-TAIDE
                        1: begin offset = 13; char_count = 17; end // REUSE. REPURPOSE.
                        2: begin offset = 13; char_count = 17; end // BIO-PULSE MONITOR
                        3: begin offset = 4;  char_count = 20; end // [PRESS KEY1 -> NEXT]
                    endcase
                end else begin
                    // Screen 1 centering offsets
                    case (page)
                        0: begin offset = 16; char_count = 16; end // PATIENT ID: #042
                        1: begin offset = 22; char_count = 14; end // NAME: NILAMBEN
                        2: begin offset = 22; char_count = 14; end // HR: 72 BPM  OK
                        3: begin offset = 28; char_count = 12; end // TEMP: 21.0 C
                    endcase
                end

                if ((column >= offset) && (column < (offset + char_count * 6))) begin
                    pos         = column - offset;
                    char_index  = pos / 6;
                    char_column = pos % 6;

                    if (char_column < 5)
                        framebuffer = glyph(text_char(scr, page, char_index), char_column);
                    else
                        framebuffer = 8'h00;
                end
            end
        end
    endfunction

    // ============================================================
    // 11. Main Controller FSM (With Clean Blank Wipe on Screen Switch)
    // ============================================================
    localparam
        S_POWER     = 4'd0,
        S_INIT_SEND = 4'd1,
        S_INIT_WAIT = 4'd2,
        S_ADDR_SEND = 4'd3,
        S_ADDR_WAIT = 4'd4,
        S_DATA_SEND = 4'd5,
        S_DATA_WAIT = 4'd6,
        S_DONE      = 4'd7;

    reg [3:0]  state;
    reg [22:0] power_counter;
    reg [4:0]  init_index;
    reg [2:0]  addr_index;
    reg [8:0]  pixel_index;

    function [7:0] address_command;
        input [2:0] index;
        begin
            case (index)
                0: address_command = 8'h21;
                1: address_command = 8'h00;
                2: address_command = 8'h7F;
                3: address_command = 8'h22;
                4: address_command = 8'h00;
                5: address_command = 8'h03;
                default: address_command = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        clear_done <= 1'b0;

        if (!reset_n) begin
            state         <= S_POWER;
            power_counter <= 23'd0;
            init_index    <= 5'd0;
            addr_index    <= 3'd0;
            pixel_index   <= 9'd0;
            control_byte  <= 8'h00;
            tx_byte       <= 8'h00;
            send          <= 1'b0;
            done          <= 1'b0;
        end else begin
            send <= 1'b0;

            case (state)
                S_POWER: begin
                    done <= 1'b0;
                    if (power_counter >= 23'd5_000_000) begin
                        init_index <= 5'd0;
                        state      <= S_INIT_SEND;
                    end else begin
                        power_counter <= power_counter + 1'b1;
                    end
                end

                S_INIT_SEND: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        tx_byte      <= init_command(init_index);
                        send         <= 1'b1;
                        state        <= S_INIT_WAIT;
                    end
                end

                S_INIT_WAIT: begin
                    if (i2c_done) begin
                        if (init_index == 5'd24) begin
                            addr_index <= 3'd0;
                            state      <= S_ADDR_SEND;
                        end else begin
                            init_index <= init_index + 1'b1;
                            state      <= S_INIT_SEND;
                        end
                    end
                end

                S_ADDR_SEND: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h00;
                        tx_byte      <= address_command(addr_index);
                        send         <= 1'b1;
                        state        <= S_ADDR_WAIT;
                    end
                end

                S_ADDR_WAIT: begin
                    if (i2c_done) begin
                        if (addr_index == 3'd5) begin
                            pixel_index <= 9'd0;
                            state       <= S_DATA_SEND;
                        end else begin
                            addr_index <= addr_index + 1'b1;
                            state      <= S_ADDR_SEND;
                        end
                    end
                end

                S_DATA_SEND: begin
                    if (i2c_state == I_IDLE) begin
                        control_byte <= 8'h40;
                        tx_byte      <= framebuffer(clear_phase, screen_sel, pixel_index[8:7], pixel_index[6:0], ecg_offset);
                        send         <= 1'b1;
                        state        <= S_DATA_WAIT;
                    end
                end

                S_DATA_WAIT: begin
                    if (i2c_done) begin
                        if (pixel_index == 9'd511) begin
                            if (clear_phase) begin
                                // Finished wiping screen blank -> now immediately draw new screen from (0,0)
                                clear_done <= 1'b1;
                                addr_index <= 3'd0;
                                state      <= S_ADDR_SEND;
                            end else if (refresh_pending) begin
                                // KEY1 was pressed mid-frame -> restart from (0,0) to wipe & redraw cleanly
                                addr_index <= 3'd0;
                                state      <= S_ADDR_SEND;
                            end else begin
                                state <= S_DONE;
                            end
                        end else begin
                            pixel_index <= pixel_index + 1'b1;
                            state       <= S_DATA_SEND;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    if (refresh_pending || ecg_scroll_req) begin
                        done       <= 1'b0;
                        addr_index <= 3'd0;
                        state      <= S_ADDR_SEND;
                    end
                end

                default: state <= S_POWER;
            endcase
        end
    end

    // ============================================================
    // 12. Debug & Heartbeat LEDs
    // ============================================================
    assign LED[0] = (i2c_state != I_IDLE);
    assign LED[1] = address_ack_seen;
    assign LED[2] = screen_sel[0];
    assign LED[3] = (screen_sel == 2'd2) ? (ecg_offset[5:2] == 4'd4) : done;

endmodule

