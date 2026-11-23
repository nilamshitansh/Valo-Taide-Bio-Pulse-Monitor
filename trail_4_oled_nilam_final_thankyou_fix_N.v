module oled_nilam (
    input  wire       clk,       // DE10-Nano 50 MHz
    input  wire       reset_n,   // active low
    inout  wire       oled_sda,
    inout  wire       oled_scl,
    output wire [3:0] LED,
    output reg        done
);

    // ============================================================
    // 50 MHz -> I2C timing tick
    // ============================================================

    localparam integer DIVIDER = 125;

    reg [7:0] div_count;
    reg       tick;

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


    // ============================================================
    // OPEN-DRAIN I2C
    // ============================================================

    reg scl_low;
    reg sda_low;

    assign oled_scl = scl_low ? 1'b0 : 1'bz;
    assign oled_sda = sda_low ? 1'b0 : 1'bz;


    // ============================================================
    // I2C TRANSMITTER
    //
    // START
    // 0x78       SSD1306 address (0x3C << 1)
    // control
    // data
    // STOP
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

    reg send;
    reg i2c_done;

    reg [7:0] control_byte;
    reg [7:0] tx_byte;

    reg [7:0] shift;
    reg [2:0] bit_count;
    reg [1:0] byte_count;

    reg address_ack_seen;
    reg nack_seen;


    always @(posedge clk) begin

        i2c_done <= 1'b0;

        if (!reset_n) begin

            i2c_state        <= I_IDLE;
            scl_low          <= 0;
            sda_low          <= 0;

            shift            <= 0;
            bit_count        <= 7;
            byte_count       <= 0;

            address_ack_seen <= 0;
            nack_seen        <= 0;

        end else begin

            if ((i2c_state == I_IDLE) && send) begin

                shift      <= 8'h78;
                bit_count  <= 7;
                byte_count <= 0;

                i2c_state  <= I_START1;

            end

            else if (tick) begin

                case (i2c_state)

                    I_IDLE: begin
                        scl_low <= 0;
                        sda_low <= 0;
                    end


                    I_START1: begin
                        scl_low <= 0;
                        sda_low <= 0;
                        i2c_state <= I_START2;
                    end


                    I_START2: begin
                        scl_low <= 0;
                        sda_low <= 1;
                        i2c_state <= I_BIT_LOW;
                    end


                    I_BIT_LOW: begin

                        scl_low <= 1;

                        if (shift[bit_count] == 0)
                            sda_low <= 1;
                        else
                            sda_low <= 0;

                        i2c_state <= I_BIT_HIGH;

                    end


                    I_BIT_HIGH: begin
                        scl_low <= 0;
                        i2c_state <= I_BIT_END;
                    end


                    I_BIT_END: begin

                        scl_low <= 1;

                        if (bit_count == 0) begin

                            sda_low <= 0;
                            i2c_state <= I_ACK_LOW;

                        end else begin

                            bit_count <= bit_count - 1'b1;
                            i2c_state <= I_BIT_LOW;

                        end
                    end


                    I_ACK_LOW: begin
                        scl_low <= 1;
                        sda_low <= 0;
                        i2c_state <= I_ACK_HIGH;
                    end


                    I_ACK_HIGH: begin
                        scl_low <= 0;
                        sda_low <= 0;
                        i2c_state <= I_ACK_END;
                    end


                    I_ACK_END: begin

                        if (oled_sda == 1'b1)
                            nack_seen <= 1'b1;

                        if ((byte_count == 0) &&
                            (oled_sda == 1'b0))
                            address_ack_seen <= 1'b1;

                        scl_low <= 1;

                        if (byte_count == 0) begin

                            shift      <= control_byte;
                            bit_count  <= 7;
                            byte_count <= 1;

                            i2c_state <= I_BIT_LOW;

                        end else if (byte_count == 1) begin

                            shift      <= tx_byte;
                            bit_count  <= 7;
                            byte_count <= 2;

                            i2c_state <= I_BIT_LOW;

                        end else begin

                            sda_low <= 1;
                            i2c_state <= I_STOP1;

                        end
                    end


                    I_STOP1: begin
                        scl_low <= 1;
                        sda_low <= 1;
                        i2c_state <= I_STOP2;
                    end


                    I_STOP2: begin
                        scl_low <= 0;
                        sda_low <= 1;
                        i2c_state <= I_STOP3;
                    end


                    I_STOP3: begin

                        scl_low <= 0;
                        sda_low <= 0;

                        i2c_done <= 1;

                        i2c_state <= I_IDLE;

                    end


                    default: begin
                        i2c_state <= I_IDLE;
                        scl_low   <= 0;
                        sda_low   <= 0;
                    end

                endcase
            end
        end
    end


    // ============================================================
    // SSD1306 INITIALIZATION
    // 128 x 32
    // ============================================================

    function [7:0] init_command;

        input [4:0] index;

        begin
            case(index)

                 0: init_command = 8'hAE;

                 1: init_command = 8'hD5;
                 2: init_command = 8'h80;

                 3: init_command = 8'hA8;
                 4: init_command = 8'h1F;

                 5: init_command = 8'hD3;
                 6: init_command = 8'h00;

                 7: init_command = 8'h40;

                 8: init_command = 8'h8D;
                 9: init_command = 8'h14;

                10: init_command = 8'h20;
                11: init_command = 8'h00;

                12: init_command = 8'hA1;
                13: init_command = 8'hC8;

                14: init_command = 8'hDA;
                15: init_command = 8'h02;

                16: init_command = 8'h81;
                17: init_command = 8'h7F;

                18: init_command = 8'hD9;
                19: init_command = 8'hF1;

                20: init_command = 8'hDB;
                21: init_command = 8'h40;

                22: init_command = 8'hA4;
                23: init_command = 8'hA6;
                24: init_command = 8'hAF;

                default:
                    init_command = 8'hAE;

            endcase
        end
    endfunction


    // ============================================================
    // 5x7 FONT
    // Each byte is one vertical column.
    // ============================================================

    function [7:0] glyph;

        input [7:0] ch;
        input [2:0] col;

        begin

            glyph = 8'h00;

            case(ch)

                "A":
                    case(col)
                        0:glyph=8'h7E;
                        1:glyph=8'h09;
                        2:glyph=8'h09;
                        3:glyph=8'h09;
                        4:glyph=8'h7E;
                    endcase

                "B":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h49;
                        2:glyph=8'h49;
                        3:glyph=8'h49;
                        4:glyph=8'h36;
                    endcase

                "D":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h41;
                        2:glyph=8'h41;
                        3:glyph=8'h22;
                        4:glyph=8'h1C;
                    endcase

                "E":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h49;
                        2:glyph=8'h49;
                        3:glyph=8'h49;
                        4:glyph=8'h41;
                    endcase

                "H":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h08;
                        2:glyph=8'h08;
                        3:glyph=8'h08;
                        4:glyph=8'h7F;
                    endcase

                "I":
                    case(col)
                        0:glyph=8'h41;
                        1:glyph=8'h41;
                        2:glyph=8'h7F;
                        3:glyph=8'h41;
                        4:glyph=8'h41;
                    endcase

                "K":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h08;
                        2:glyph=8'h14;
                        3:glyph=8'h22;
                        4:glyph=8'h41;
                    endcase

                "L":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h40;
                        2:glyph=8'h40;
                        3:glyph=8'h40;
                        4:glyph=8'h40;
                    endcase

                "M":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h02;
                        2:glyph=8'h0C;
                        3:glyph=8'h02;
                        4:glyph=8'h7F;
                    endcase
// Fix N below code
"N":
    case(col)
        0:glyph=8'h7F;
        1:glyph=8'h06;
        2:glyph=8'h0C;
        3:glyph=8'h30;
        4:glyph=8'h7F;
    endcase

                "O":
                    case(col)
                        0:glyph=8'h3E;
                        1:glyph=8'h41;
                        2:glyph=8'h41;
                        3:glyph=8'h41;
                        4:glyph=8'h3E;
                    endcase

                "P":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h09;
                        2:glyph=8'h09;
                        3:glyph=8'h09;
                        4:glyph=8'h06;
                    endcase

                "R":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h09;
                        2:glyph=8'h19;
                        3:glyph=8'h29;
                        4:glyph=8'h46;
                    endcase

                "S":
                    case(col)
                        0:glyph=8'h46;
                        1:glyph=8'h49;
                        2:glyph=8'h49;
                        3:glyph=8'h49;
                        4:glyph=8'h31;
                    endcase

                "T":
                    case(col)
                        0:glyph=8'h01;
                        1:glyph=8'h01;
                        2:glyph=8'h7F;
                        3:glyph=8'h01;
                        4:glyph=8'h01;
                    endcase

                "U":
                    case(col)
                        0:glyph=8'h3F;
                        1:glyph=8'h40;
                        2:glyph=8'h40;
                        3:glyph=8'h40;
                        4:glyph=8'h3F;
                    endcase

                "V":
                    case(col)
                        0:glyph=8'h1F;
                        1:glyph=8'h20;
                        2:glyph=8'h40;
                        3:glyph=8'h20;
                        4:glyph=8'h1F;
                    endcase

                "X":
                    case(col)
                        0:glyph=8'h63;
                        1:glyph=8'h14;
                        2:glyph=8'h08;
                        3:glyph=8'h14;
                        4:glyph=8'h63;
                    endcase

                "Y":
                    case(col)
                        0:glyph=8'h07;
                        1:glyph=8'h08;
                        2:glyph=8'h70;
                        3:glyph=8'h08;
                        4:glyph=8'h07;
                    endcase

                "2":
                    case(col)
                        0:glyph=8'h62;
                        1:glyph=8'h51;
                        2:glyph=8'h49;
                        3:glyph=8'h49;
                        4:glyph=8'h46;
                    endcase

                "&":
                    case(col)
                        0:glyph=8'h36;
                        1:glyph=8'h49;
                        2:glyph=8'h55;
                        3:glyph=8'h22;
                        4:glyph=8'h50;
                    endcase

                ".":
                    case(col)
                        0:glyph=8'h00;
                        1:glyph=8'h60;
                        2:glyph=8'h60;
                        3:glyph=8'h00;
                        4:glyph=8'h00;
                    endcase

                "-":
                    case(col)
                        0:glyph=8'h08;
                        1:glyph=8'h08;
                        2:glyph=8'h08;
                        3:glyph=8'h08;
                        4:glyph=8'h08;
                    endcase

                "B":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h48;
                        2:glyph=8'h48;
                        3:glyph=8'h48;
                        4:glyph=8'h30;
                    endcase

                "y":
                    case(col)
                        0:glyph=8'h0F;
                        1:glyph=8'h50;
                        2:glyph=8'h50;
                        3:glyph=8'h50;
                        4:glyph=8'h3F;
                    endcase

                "i":
                    case(col)
                        0:glyph=8'h00;
                        1:glyph=8'h44;
                        2:glyph=8'h7D;
                        3:glyph=8'h40;
                        4:glyph=8'h00;
                    endcase

                "l":
                    case(col)
                        0:glyph=8'h00;
                        1:glyph=8'h41;
                        2:glyph=8'h7F;
                        3:glyph=8'h40;
                        4:glyph=8'h00;
                    endcase

                "a":
                    case(col)
                        0:glyph=8'h20;
                        1:glyph=8'h54;
                        2:glyph=8'h54;
                        3:glyph=8'h54;
                        4:glyph=8'h78;
                    endcase

                "m":
                    case(col)
                        0:glyph=8'h7C;
                        1:glyph=8'h04;
                        2:glyph=8'h18;
                        3:glyph=8'h04;
                        4:glyph=8'h78;
                    endcase

                "b":
                    case(col)
                        0:glyph=8'h7F;
                        1:glyph=8'h48;
                        2:glyph=8'h48;
                        3:glyph=8'h48;
                        4:glyph=8'h30;
                    endcase

                "e":
                    case(col)
                        0:glyph=8'h38;
                        1:glyph=8'h54;
                        2:glyph=8'h54;
                        3:glyph=8'h54;
                        4:glyph=8'h18;
                    endcase

                "n":
                    case(col)
                        0:glyph=8'h7C;
                        1:glyph=8'h04;
                        2:glyph=8'h04;
                        3:glyph=8'h04;
                        4:glyph=8'h78;
                    endcase

                default:
                    glyph = 8'h00;

            endcase

        end
    endfunction


    // ============================================================
    // TEXT CHARACTER LOOKUP
    //
    // PAGE 0 : OULU UNI. & NOKIA
    // PAGE 1 : THANK YOU
    // PAGE 2 : ASSP & DT2 EXPERIMENT
    // PAGE 3 : -By Nilamben
    // ============================================================

    function [7:0] text_char;

        input [1:0] page;
        input [4:0] index;

        begin

            text_char = " ";

            case(page)

                // PAGE 0 : OULU UNI. & NOKIA
                0:
                    case(index)
                         0:text_char="O";
                         1:text_char="U";
                         2:text_char="L";
                         3:text_char="U";
                         4:text_char=" ";
                         5:text_char="U";
                         6:text_char="N";
                         7:text_char="I";
                         8:text_char=".";
                         9:text_char=" ";
                        10:text_char="&";
                        11:text_char=" ";
                        12:text_char="N";
                        13:text_char="O";
                        14:text_char="K";
                        15:text_char="I";
                        16:text_char="A";
                        default:text_char=" ";
                    endcase

                // PAGE 1 : THANK YOU
                1:
                    case(index)
                         0:text_char="T";
                         1:text_char="H";
                         2:text_char="A";
                         3:text_char="N";
                         4:text_char="K";
                         5:text_char=" ";
                         6:text_char="Y";
                         7:text_char="O";
                         8:text_char="U";
                        default:text_char=" ";
                    endcase

                // PAGE 2 : ASSP & DT2 EXPERIMENT
                2:
                    case(index)
                         0:text_char="A";
                         1:text_char="S";
                         2:text_char="S";
                         3:text_char="P";
                         4:text_char=" ";
                         5:text_char="&";
                         6:text_char=" ";
                         7:text_char="D";
                         8:text_char="T";
                         9:text_char="2";
                        10:text_char=" ";
                        11:text_char="E";
                        12:text_char="X";
                        13:text_char="P";
                        14:text_char="E";
                        15:text_char="R";
                        16:text_char="I";
                        17:text_char="M";
                        18:text_char="E";
                        19:text_char="N";
                        20:text_char="T";
                        default:text_char=" ";
                    endcase

                // PAGE 3 : -By Nilamben
                3:
                    case(index)
                         0:text_char="-";
                         1:text_char="B";
                         2:text_char="y";
                         3:text_char=" ";
                         4:text_char="N";
                         5:text_char="i";
                         6:text_char="l";
                         7:text_char="a";
                         8:text_char="m";
                         9:text_char="b";
                        10:text_char="e";
                        11:text_char="n";
                        default:text_char=" ";
                    endcase

                default:
                    text_char=" ";

            endcase
        end
    endfunction


    // ============================================================
    // FRAMEBUFFER GENERATOR
    //
    // Full 128-pixel width is used for text.
    // 5 pixel glyph + 1 blank pixel per character.
    // Each line is centered independently.
    // ============================================================

    function [7:0] framebuffer;

        input [1:0] page;
        input [6:0] column;

        integer pos;
        integer char_index;
        integer char_column;
        integer offset;
        integer char_count;

        begin

            framebuffer = 8'h00;

            case(page)
                // 17 chars -> 102 pixels, centered
                0: begin
                    offset     = 13;
                    char_count = 17;
                end

                // 9 chars -> 54 pixels, centered
                1: begin
                    offset     = 37;
                    char_count = 9;
                end

                // 21 chars -> 126 pixels, centered with 1-pixel margin
                2: begin
                    offset     = 1;
                    char_count = 21;
                end

                // 12 chars -> 72 pixels, centered
                3: begin
                    offset     = 28;
                    char_count = 12;
                end

                default: begin
                    offset     = 0;
                    char_count = 0;
                end
            endcase

            if ((column >= offset) &&
                (column < (offset + char_count*6))) begin

                pos = column - offset;

                char_index  = pos / 6;
                char_column = pos % 6;

                if (char_column < 5)
                    framebuffer =
                        glyph(
                            text_char(page,char_index),
                            char_column
                        );
                else
                    framebuffer = 8'h00;

            end

        end
    endfunction


    // ============================================================
    // MAIN CONTROLLER
    // ============================================================

    localparam
        S_POWER       = 4'd0,
        S_INIT_SEND   = 4'd1,
        S_INIT_WAIT   = 4'd2,
        S_ADDR_SEND   = 4'd3,
        S_ADDR_WAIT   = 4'd4,
        S_DATA_SEND   = 4'd5,
        S_DATA_WAIT   = 4'd6,
        S_DONE        = 4'd7;


    reg [3:0] state;

    reg [22:0] power_counter;

    reg [4:0] init_index;

    reg [2:0] addr_index;

    reg [8:0] pixel_index;


    // ============================================================
    // Set full display address:
    //
    // columns 0..127
    // pages   0..3
    // ============================================================

    function [7:0] address_command;

        input [2:0] index;

        begin

            case(index)

                0:address_command=8'h21;
                1:address_command=8'h00;
                2:address_command=8'h7F;

                3:address_command=8'h22;
                4:address_command=8'h00;
                5:address_command=8'h03;

                default:
                    address_command=8'h00;

            endcase

        end
    endfunction


    // ============================================================
    // MAIN FSM
    // ============================================================

    always @(posedge clk) begin

        if (!reset_n) begin

            state         <= S_POWER;

            power_counter <= 0;

            init_index    <= 0;
            addr_index    <= 0;
            pixel_index   <= 0;

            control_byte  <= 0;
            tx_byte       <= 0;

            send           <= 0;
            done           <= 0;

        end

        else begin

            send <= 0;


            case(state)

                // ------------------------------------------------
                // 100 ms power-up wait
                // ------------------------------------------------

                S_POWER: begin

                    done <= 0;

                    if (power_counter >= 23'd5_000_000) begin

                        init_index <= 0;

                        state <= S_INIT_SEND;

                    end else begin

                        power_counter <=
                            power_counter + 1'b1;

                    end
                end


                // ------------------------------------------------
                // SSD1306 initialization
                // ------------------------------------------------

                S_INIT_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h00;

                        tx_byte <=
                            init_command(init_index);

                        send <= 1;

                        state <= S_INIT_WAIT;

                    end
                end


                S_INIT_WAIT: begin

                    if (i2c_done) begin

                        if (init_index == 24) begin

                            addr_index <= 0;

                            state <= S_ADDR_SEND;

                        end else begin

                            init_index <=
                                init_index + 1'b1;

                            state <= S_INIT_SEND;

                        end
                    end
                end


                // ------------------------------------------------
                // Address complete framebuffer
                // ------------------------------------------------

                S_ADDR_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h00;

                        tx_byte <=
                            address_command(addr_index);

                        send <= 1;

                        state <= S_ADDR_WAIT;

                    end
                end


                S_ADDR_WAIT: begin

                    if (i2c_done) begin

                        if (addr_index == 5) begin

                            pixel_index <= 0;

                            state <= S_DATA_SEND;

                        end else begin

                            addr_index <=
                                addr_index + 1'b1;

                            state <= S_ADDR_SEND;

                        end
                    end
                end


                // ------------------------------------------------
                // Write ALL 512 framebuffer bytes
                // ------------------------------------------------

                S_DATA_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h40;

                        tx_byte <=
                            framebuffer(
                                pixel_index[8:7],
                                pixel_index[6:0]
                            );

                        send <= 1;

                        state <= S_DATA_WAIT;

                    end
                end


                S_DATA_WAIT: begin

                    if (i2c_done) begin

                        if (pixel_index == 9'd511) begin

                            state <= S_DONE;

                        end else begin

                            pixel_index <=
                                pixel_index + 1'b1;

                            state <= S_DATA_SEND;

                        end
                    end
                end


                // ------------------------------------------------
                // Complete
                // ------------------------------------------------

                S_DONE: begin
                    done <= 1;
                end


                default:
                    state <= S_POWER;

            endcase
        end
    end


    // ============================================================
    // DEBUG LEDs
    // ============================================================

    // LED0 = I2C active
    assign LED[0] =
        (i2c_state != I_IDLE);

    // LED1 = SSD1306 ACK received
    assign LED[1] =
        address_ack_seen;

    // LED2 = NACK detected
    assign LED[2] =
        nack_seen;

    // LED3 = framebuffer completed
    assign LED[3] =
        done;

endmodule