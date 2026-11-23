module oled_nilam (
    input  wire       clk,       // DE10-Nano 50 MHz
    input  wire       reset_n,   // KEY0, active low
    inout  wire       oled_sda,
    inout  wire       oled_scl,
    output wire [3:0] LED,
    output reg        done
);

    // ============================================================
    // 50 MHz -> I2C timing tick
    // 125 clocks = 2.5 us
    // ~100 kHz I2C using multiple phases per bit
    // ============================================================

    localparam integer DIVIDER = 125;

    reg [7:0] div_count;
    reg       tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            div_count <= 8'd0;
            tick      <= 1'b0;
        end
        else begin
            if (div_count == DIVIDER-1) begin
                div_count <= 8'd0;
                tick      <= 1'b1;
            end
            else begin
                div_count <= div_count + 1'b1;
                tick      <= 1'b0;
            end
        end
    end


    // ============================================================
    // Open-drain I2C
    // ============================================================

    reg scl_low;
    reg sda_low;

    assign oled_scl = scl_low ? 1'b0 : 1'bz;
    assign oled_sda = sda_low ? 1'b0 : 1'bz;


    // ============================================================
    // I2C transmitter
    //
    // Transaction:
    //
    // START
    // 0x78            SSD1306 address 0x3C + WRITE
    // control_byte    0x00 command / 0x40 display data
    // tx_byte
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

    reg       send;
    reg       i2c_done;

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

            scl_low          <= 1'b0;
            sda_low          <= 1'b0;

            shift            <= 8'h00;
            bit_count        <= 3'd7;
            byte_count       <= 2'd0;

            address_ack_seen <= 1'b0;
            nack_seen        <= 1'b0;

        end
        else begin

            // Capture transmit request at full clock speed
            if ((i2c_state == I_IDLE) && send) begin

                shift      <= 8'h78;      // 0x3C << 1
                bit_count  <= 3'd7;
                byte_count <= 2'd0;

                i2c_state <= I_START1;

            end

            else if (tick) begin

                case (i2c_state)

                    I_IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                    end


                    I_START1: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        i2c_state <= I_START2;
                    end


                    I_START2: begin
                        // START condition
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        i2c_state <= I_BIT_LOW;
                    end


                    I_BIT_LOW: begin

                        scl_low <= 1'b1;

                        if (shift[bit_count] == 1'b0)
                            sda_low <= 1'b1;
                        else
                            sda_low <= 1'b0;

                        i2c_state <= I_BIT_HIGH;

                    end


                    I_BIT_HIGH: begin
                        scl_low <= 1'b0;
                        i2c_state <= I_BIT_END;
                    end


                    I_BIT_END: begin

                        scl_low <= 1'b1;

                        if (bit_count == 0) begin

                            // Release SDA for ACK
                            sda_low <= 1'b0;
                            i2c_state <= I_ACK_LOW;

                        end
                        else begin

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

                        // SCL high, SDA released
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                        i2c_state <= I_ACK_END;

                    end


                    I_ACK_END: begin

                        // Slave ACK = SDA low
                        if (oled_sda == 1'b1)
                            nack_seen <= 1'b1;

                        if ((byte_count == 0) &&
                            (oled_sda == 1'b0))
                            address_ack_seen <= 1'b1;

                        scl_low <= 1'b1;


                        if (byte_count == 0) begin

                            // Send control byte
                            shift      <= control_byte;
                            bit_count  <= 3'd7;
                            byte_count <= 2'd1;

                            i2c_state <= I_BIT_LOW;

                        end

                        else if (byte_count == 1) begin

                            // Send command/data byte
                            shift      <= tx_byte;
                            bit_count  <= 3'd7;
                            byte_count <= 2'd2;

                            i2c_state <= I_BIT_LOW;

                        end

                        else begin

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

                        // STOP condition
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                        i2c_done <= 1'b1;

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
    // SSD1306 initialization
    // 128 x 32 OLED
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
                15: init_command = 8'h02;

                16: init_command = 8'h81;
                17: init_command = 8'h7F;

                18: init_command = 8'hD9;
                19: init_command = 8'hF1;

                20: init_command = 8'hDB;
                21: init_command = 8'h40;

                22: init_command = 8'hA4; // Follow GDDRAM
                23: init_command = 8'hA6; // Normal display
                24: init_command = 8'hAF; // Display ON

                default:
                    init_command = 8'hAE;

            endcase
        end

    endfunction


    // ============================================================
    // Font ROM
    //
    // 5x7 characters:
    //
    // N I L A M
    //
    // Each character = 5 bytes + 1 blank column.
    // Data is vertical-column format for SSD1306.
    // ============================================================

    function [7:0] name_data;

        input [5:0] index;

        begin

            case (index)

                // N
                 0: name_data = 8'h7F;
                 1: name_data = 8'h02;
                 2: name_data = 8'h04;
                 3: name_data = 8'h08;
                 4: name_data = 8'h7F;
                 5: name_data = 8'h00;

                // I
                 6: name_data = 8'h41;
                 7: name_data = 8'h41;
                 8: name_data = 8'h7F;
                 9: name_data = 8'h41;
                10: name_data = 8'h41;
                11: name_data = 8'h00;

                // L
                12: name_data = 8'h7F;
                13: name_data = 8'h40;
                14: name_data = 8'h40;
                15: name_data = 8'h40;
                16: name_data = 8'h40;
                17: name_data = 8'h00;

                // A
                18: name_data = 8'h7E;
                19: name_data = 8'h09;
                20: name_data = 8'h09;
                21: name_data = 8'h09;
                22: name_data = 8'h7E;
                23: name_data = 8'h00;

                // M
                24: name_data = 8'h7F;
                25: name_data = 8'h02;
                26: name_data = 8'h0C;
                27: name_data = 8'h02;
                28: name_data = 8'h7F;
                29: name_data = 8'h00;

                default:
                    name_data = 8'h00;

            endcase

        end

    endfunction


    // ============================================================
    // Main controller
    // ============================================================

    localparam
        S_POWER_WAIT = 4'd0,
        S_INIT_SEND  = 4'd1,
        S_INIT_WAIT  = 4'd2,

        S_COL_CMD    = 4'd3,
        S_COL_WAIT   = 4'd4,

        S_PAGE_CMD   = 4'd5,
        S_PAGE_WAIT  = 4'd6,

        S_TEXT_SEND  = 4'd7,
        S_TEXT_WAIT  = 4'd8,

        S_DONE       = 4'd9;


    reg [3:0] state;

    reg [22:0] power_counter;

    reg [4:0] init_index;

    reg [2:0] setup_index;

    reg [5:0] text_index;


    // ============================================================
    // Address setup command ROM
    //
    // Column range 0..127
    // Page range   1..1
    //
    // Page 1 means vertical pixels 8..15.
    // This places NILAM roughly around the upper-middle area.
    // ============================================================

    function [7:0] setup_command;

        input [2:0] index;

        begin

            case (index)

                0: setup_command = 8'h21; // Set column address
                1: setup_command = 8'd48; // Start column
                2: setup_command = 8'd77; // End column

                3: setup_command = 8'h22; // Set page address
                4: setup_command = 8'h01; // Start page = 1
                5: setup_command = 8'h01; // End page   = 1

                default:
                    setup_command = 8'h00;

            endcase

        end

    endfunction


    // ============================================================
    // Main FSM
    // ============================================================

    always @(posedge clk) begin

        if (!reset_n) begin

            state         <= S_POWER_WAIT;

            power_counter <= 23'd0;

            init_index    <= 5'd0;
            setup_index   <= 3'd0;
            text_index    <= 6'd0;

            control_byte  <= 8'h00;
            tx_byte       <= 8'h00;

            send           <= 1'b0;
            done           <= 1'b0;

        end

        else begin

            // Normally one-clock pulse
            send <= 1'b0;


            case (state)

                // ------------------------------------------------
                // ~100 ms startup delay
                // ------------------------------------------------

                S_POWER_WAIT: begin

                    done <= 1'b0;

                    if (power_counter >= 23'd5_000_000) begin

                        init_index <= 5'd0;

                        state <= S_INIT_SEND;

                    end
                    else begin

                        power_counter <=
                            power_counter + 1'b1;

                    end

                end


                // ------------------------------------------------
                // Initialization commands
                // ------------------------------------------------

                S_INIT_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h00; // COMMAND

                        tx_byte <=
                            init_command(init_index);

                        send <= 1'b1;

                        state <= S_INIT_WAIT;

                    end

                end


                S_INIT_WAIT: begin

                    if (i2c_done) begin

                        if (init_index == 5'd24) begin

                            setup_index <= 3'd0;

                            state <= S_COL_CMD;

                        end
                        else begin

                            init_index <=
                                init_index + 1'b1;

                            state <= S_INIT_SEND;

                        end

                    end

                end


                // ------------------------------------------------
                // Set OLED column/page address
                // ------------------------------------------------

                S_COL_CMD: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h00; // COMMAND

                        tx_byte <=
                            setup_command(setup_index);

                        send <= 1'b1;

                        state <= S_COL_WAIT;

                    end

                end


                S_COL_WAIT: begin

                    if (i2c_done) begin

                        if (setup_index == 3'd5) begin

                            text_index <= 6'd0;

                            state <= S_TEXT_SEND;

                        end
                        else begin

                            setup_index <=
                                setup_index + 1'b1;

                            state <= S_COL_CMD;

                        end

                    end

                end


                // ------------------------------------------------
                // Send NILAM bitmap
                //
                // control byte 0x40 = display DATA
                // ------------------------------------------------

                S_TEXT_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        control_byte <= 8'h40;

                        tx_byte <=
                            name_data(text_index);

                        send <= 1'b1;

                        state <= S_TEXT_WAIT;

                    end

                end


                S_TEXT_WAIT: begin

                    if (i2c_done) begin

                        if (text_index == 6'd29) begin

                            state <= S_DONE;

                        end
                        else begin

                            text_index <=
                                text_index + 1'b1;

                            state <= S_TEXT_SEND;

                        end

                    end

                end


                // ------------------------------------------------
                // Finished
                // ------------------------------------------------

                S_DONE: begin

                    done <= 1'b1;

                end


                default: begin

                    state <= S_POWER_WAIT;

                end

            endcase

        end

    end


    // ============================================================
    // Debug LEDs
    //
    // LED0 = I2C activity
    // LED1 = SSD1306 ACK received
    // LED2 = NACK detected
    // LED3 = finished
    //
    // done = physical LED7
    // ============================================================

    assign LED[0] =
        (i2c_state != I_IDLE);

    assign LED[1] =
        address_ack_seen;

    assign LED[2] =
        nack_seen;

    assign LED[3] =
        done;


endmodule