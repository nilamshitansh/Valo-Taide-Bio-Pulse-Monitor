module oled_nilam (
    input  wire       clk,       // 50 MHz
    input  wire       reset_n,   // active-low
    inout  wire       oled_sda,
    inout  wire       oled_scl,
    output wire [3:0] LED,
    output reg        done
);

    // ============================================================
    // Slow timing tick
    // 50 MHz / 125 = 400 kHz tick
    // I2C state machine uses multiple ticks per bit,
    // resulting in approximately 100 kHz I2C.
    // ============================================================

    reg [7:0] div_count;
    reg       tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            div_count <= 8'd0;
            tick      <= 1'b0;
        end
        else begin
            if (div_count == 8'd124) begin
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
    // I2C open-drain outputs
    // ============================================================

    reg scl_low;
    reg sda_low;

    assign oled_scl = scl_low ? 1'b0 : 1'bz;
    assign oled_sda = sda_low ? 1'b0 : 1'bz;


    // ============================================================
    // I2C transmitter
    //
    // Every transaction sends:
    //
    // START
    // 0x78       SSD1306 address 0x3C + WRITE
    // 0x00       command control byte
    // command
    // STOP
    //
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

    reg [7:0] command_byte;
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

            // Capture request at full 50 MHz clock speed
            if ((i2c_state == I_IDLE) && send) begin

                shift      <= 8'h78;       // 0x3C << 1
                bit_count  <= 3'd7;
                byte_count <= 2'd0;

                i2c_state  <= I_START1;

            end

            else if (tick) begin

                case (i2c_state)

                    // --------------------------------------------
                    // IDLE
                    // --------------------------------------------

                    I_IDLE: begin

                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                    end


                    // --------------------------------------------
                    // START
                    // --------------------------------------------

                    I_START1: begin

                        // SCL high, SDA high
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                        i2c_state <= I_START2;

                    end


                    I_START2: begin

                        // SDA falls while SCL remains high
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;

                        i2c_state <= I_BIT_LOW;

                    end


                    // --------------------------------------------
                    // Send 8 data bits
                    // --------------------------------------------

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


                    // --------------------------------------------
                    // ACK
                    // --------------------------------------------

                    I_ACK_LOW: begin

                        scl_low <= 1'b1;
                        sda_low <= 1'b0;

                        i2c_state <= I_ACK_HIGH;

                    end


                    I_ACK_HIGH: begin

                        // Raise SCL
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                        i2c_state <= I_ACK_END;

                    end


                    I_ACK_END: begin

                        // Sample SDA after high period
                        if (oled_sda == 1'b1)
                            nack_seen <= 1'b1;

                        // Most important debug:
                        // Did SSD1306 ACK address 0x3C?
                        if ((byte_count == 0) &&
                            (oled_sda == 1'b0))
                            address_ack_seen <= 1'b1;

                        scl_low <= 1'b1;


                        // Next byte
                        if (byte_count == 0) begin

                            // Command control byte
                            shift      <= 8'h00;

                            bit_count  <= 3'd7;
                            byte_count <= 2'd1;

                            i2c_state <= I_BIT_LOW;

                        end

                        else if (byte_count == 1) begin

                            // Actual SSD1306 command
                            shift      <= command_byte;

                            bit_count  <= 3'd7;
                            byte_count <= 2'd2;

                            i2c_state <= I_BIT_LOW;

                        end

                        else begin

                            // All 3 bytes sent
                            sda_low <= 1'b1;

                            i2c_state <= I_STOP1;

                        end

                    end


                    // --------------------------------------------
                    // STOP
                    // --------------------------------------------

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

                        // SDA rises while SCL is high
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                        i2c_done <= 1'b1;

                        i2c_state <= I_IDLE;

                    end


                    default: begin

                        i2c_state <= I_IDLE;

                        scl_low <= 1'b0;
                        sda_low <= 1'b0;

                    end

                endcase
            end
        end
    end


    // ============================================================
    // SSD1306 Initialization
    // ============================================================

    function [7:0] init_command;

        input [4:0] index;

        begin

            case (index)

                 0: init_command = 8'hAE; // Display OFF

                 1: init_command = 8'hD5; // Clock divide
                 2: init_command = 8'h80;

                 3: init_command = 8'hA8; // Multiplex
                 4: init_command = 8'h1F; // 32 rows

                 5: init_command = 8'hD3; // Display offset
                 6: init_command = 8'h00;

                 7: init_command = 8'h40; // Start line

                 8: init_command = 8'h8D; // Charge pump
                 9: init_command = 8'h14; // Enable

                10: init_command = 8'h20; // Memory mode
                11: init_command = 8'h00; // Horizontal

                12: init_command = 8'hA1; // Segment remap
                13: init_command = 8'hC8; // COM scan direction

                14: init_command = 8'hDA; // COM pins
                15: init_command = 8'h02; // 128x32

                16: init_command = 8'h81; // Contrast
                17: init_command = 8'h7F;

                18: init_command = 8'hD9; // Precharge
                19: init_command = 8'hF1;

                20: init_command = 8'hDB; // VCOM detect
                21: init_command = 8'h40;

                22: init_command = 8'hA4; // RAM display
                23: init_command = 8'hA6; // Normal display

                24: init_command = 8'hAF; // Display ON

                // ---------------------------------------------
                // TEST COMMAND
                //
                // A5 = Entire Display ON
                //
                // This ignores GDDRAM and forces every OLED
                // pixel ON.
                // ---------------------------------------------

                25: init_command = 8'hA5;

                default:
                    init_command = 8'hAE;

            endcase
        end

    endfunction


    // ============================================================
    // Main controller
    // ============================================================

    localparam
        S_POWER_WAIT = 3'd0,
        S_SEND       = 3'd1,
        S_WAIT       = 3'd2,
        S_DONE       = 3'd3;

    reg [2:0] state;

    reg [22:0] power_counter;

    reg [4:0] command_index;


    always @(posedge clk) begin

        if (!reset_n) begin

            state         <= S_POWER_WAIT;

            power_counter <= 23'd0;

            command_index <= 5'd0;

            command_byte  <= 8'h00;

            send           <= 1'b0;

            done           <= 1'b0;

        end

        else begin

            // send is normally only one 50-MHz clock pulse
            send <= 1'b0;


            case (state)

                // --------------------------------------------
                // Wait approximately 100 ms after power-up
                // --------------------------------------------

                S_POWER_WAIT: begin

                    done <= 1'b0;

                    if (power_counter >= 23'd5_000_000) begin

                        command_index <= 5'd0;

                        state <= S_SEND;

                    end
                    else begin

                        power_counter <= power_counter + 1'b1;

                    end

                end


                // --------------------------------------------
                // Start command transaction
                // --------------------------------------------

                S_SEND: begin

                    if (i2c_state == I_IDLE) begin

                        command_byte <=
                            init_command(command_index);

                        send <= 1'b1;

                        state <= S_WAIT;

                    end

                end


                // --------------------------------------------
                // Wait until command transaction completed
                // --------------------------------------------

                S_WAIT: begin

                    if (i2c_done) begin

                        if (command_index == 5'd25) begin

                            state <= S_DONE;

                        end
                        else begin

                            command_index <=
                                command_index + 1'b1;

                            state <= S_SEND;

                        end

                    end

                end


                // --------------------------------------------
                // Finished
                // --------------------------------------------

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
    // ============================================================
    //
    // LED0 : I2C currently active
    //
    // LED1 : SSD1306 ACK received at address 0x3C
    //        THIS IS THE IMPORTANT DEBUG LED
    //
    // LED2 : any NACK was detected
    //
    // LED3 : initialization completed
    //
    // done : also becomes 1 at completion
    //        Your QSF maps done to physical LED7.
    // ============================================================

    assign LED[0] = (i2c_state != I_IDLE);

    assign LED[1] = address_ack_seen;

    assign LED[2] = nack_seen;

    assign LED[3] = done;


endmodule