module i2c_master (
    input wire sys_clk,

    input wire start_sig,
    input wire stop_sig,
    input wire send_data_sig,
    input wire read_data_sig,

    input wire [7:0] data_in,
    output reg [7:0] data_out,

    output reg busy,
    output reg ack,

    output reg scl,
    inout wire sda_out
);
    reg sda;
    assign sda_out = (sda == 1'b0) ? 1'b0 : 1'bz;

    localparam CLK_FREQ = 50000000;
    localparam CLK_SCL = CLK_FREQ / 100000;
    localparam CLK_SDA = CLK_SCL / 2;

    localparam COUNTER_BIT_WIDTH = $clog2(CLK_SCL);

    reg [COUNTER_BIT_WIDTH:0] counter;
    reg [3:0] byte_counter;
    

    localparam IDLE = 4'd0;
    localparam START = 4'd1;
    localparam SENDING = 4'd2;
    localparam READING = 4'd3;
    localparam STOP = 4'd4;

    reg [3:0] state;
    reg start_sig_d;
    reg stop_sig_d;
    reg read_data_sig_d;
    reg send_data_sig_d;
    
    initial begin
        state = IDLE;
        scl <= 1;
        sda <= 1'b1;

        busy <= 0;
        ack <= 1;
        counter <= 0;
        byte_counter <= 0;

        data_out <= 8'hFF;
    end

    always @(posedge sys_clk) begin
        start_sig_d <= start_sig;
        stop_sig_d <= stop_sig;
        read_data_sig_d <= read_data_sig;
        send_data_sig_d <= send_data_sig;

        if (state == IDLE && start_sig_d) begin
            state = START;
            busy <= 1;
        end

        if (state == IDLE && send_data_sig_d) begin
            state = SENDING;
            busy <= 1;
        end

        if (state == IDLE && read_data_sig_d) begin
            state = READING;
            busy <= 1;
        end

        if (state == IDLE && stop_sig_d) begin
            state = STOP;
            busy <= 1;
        end

        // 100kHz SCL & SDA LOOPS

        
        if (state != IDLE) begin
            counter <= counter + 1;
        end

        if (counter == CLK_SDA) begin

            if (scl && state == START) begin
                sda <= 1'b0;
            end

            if (scl && state == STOP) begin
                sda <= 1'b1;
            end
            if(!scl && state == STOP) begin
                sda <= 1'b0;
            end

            if (state == SENDING && !scl && byte_counter < 8) begin
                sda <= data_in[7 - byte_counter];
                byte_counter <= byte_counter + 1;
            end

            if (state == SENDING && scl && byte_counter == 9) begin
                ack <= sda_out; // odbieram ack
            end

            if (state == READING && scl && byte_counter < 8) begin
                data_out[7 - byte_counter] <= sda_out;
                byte_counter <= byte_counter + 1;
            end

            if (state == READING && !scl && byte_counter == 8) begin
                sda <= 1'b0; // wysylam ack
                byte_counter <= byte_counter + 1;
            end
        end

        if (counter == CLK_SCL) begin
            if (scl && state == SENDING && byte_counter == 8) begin
                sda <= 1'b1;
                byte_counter <= byte_counter + 1;
            end
            else if (scl && state == SENDING && byte_counter == 9) begin
                state <= IDLE;
                sda <= 1'b1;
                byte_counter <= 0;
                busy <= 0;
            end

            if (scl && state == READING && byte_counter == 9) begin
                state <= IDLE;
                sda <= 1'b1;
                byte_counter <= 0;
                busy <= 0;
            end

            if (scl && (state == START || state == STOP)) begin
                state <= IDLE;
                sda <= 1'b1;
                byte_counter <= 0;
                busy <= 0;
            end

            scl <= ~scl;
            counter <= 0;
        end
    end
endmodule