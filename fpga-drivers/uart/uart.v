module uart (
    input wire sys_clk,

    input wire send_data_sig, // impuls jak na wejsciu jest poprawne data_in
    output reg tx_busy,

    output reg read_data_sig, // impuls jak odczytano poprawne data_out
    output reg error_data_flag, // staly sygnal jak odczytano zly bit zamykajacy ramke


    input wire [7:0] data_in,
    output reg [7:0] data_out,

    output reg tx,
    input wire rx
);
    localparam CLK_FREQ = 50000000;

    localparam BAUD_RATE = 100000;
    localparam BAUD_RATEx16 = BAUD_RATE * 16;

    localparam CLK_RX = CLK_FREQ / BAUD_RATEx16;
    localparam CLK_TX = CLK_FREQ / BAUD_RATE;

    localparam COUNTER_BIT_WIDTH_RX = $clog2(CLK_RX);
    localparam COUNTER_BIT_WIDTH_TX = $clog2(CLK_TX);

    localparam IDLE = 1'b0;
    localparam ACTIVE = 1'b1;

    reg [COUNTER_BIT_WIDTH_RX:0] counter_rx;
    reg [COUNTER_BIT_WIDTH_TX:0] counter_tx;

    reg [0:0] rx_state;
    reg [0:0] rx_buf;
    reg [3:0] sampling_counter;
    reg [3:0] rx_byte_counter;

    reg [0:0] tx_state;
    reg [3:0] tx_byte_counter;

    
    reg read_data_sig_d;
    reg send_data_sig_d;
    
    initial begin
        tx <= 1;
        tx_busy <= 0;

        rx_state <= IDLE;
        sampling_counter <= 0;
        rx_byte_counter <= 0;
        rx_buf <= 1;

        tx_state <= IDLE;
        tx_byte_counter <= 0;
        error_data_flag <= 0;

        counter_rx <= 0;
        counter_tx <= 0;

        data_out <= 8'hFF;
    end

    always @(posedge sys_clk) begin
        rx_buf <= rx;
        if (read_data_sig == 1) read_data_sig <= 0;
        error_data_flag <= 0;
        send_data_sig_d <= send_data_sig;

        if (tx_state == IDLE && !tx_busy && send_data_sig_d) begin
            tx_state <= ACTIVE;
            tx_byte_counter <= 0;
            tx_busy <= 1;
            send_data_sig_d <= 0;
        end

        
        counter_rx <= counter_rx + 1;
        counter_tx <= counter_tx + 1;

        if (counter_rx == CLK_RX) begin

            if (rx_state == IDLE && !rx_buf) begin
                rx_state <= ACTIVE;

                sampling_counter <= 0;
                rx_byte_counter <= 0;
            end

            if (rx_state == ACTIVE && sampling_counter == 7) begin

                if (rx_byte_counter == 0 && !rx_buf) begin
                    rx_byte_counter <= rx_byte_counter + 1;
                end
                else if(rx_byte_counter == 0) begin
                    rx_state <= IDLE;
                end
                else if(rx_byte_counter == 9 && rx_buf) begin
                    rx_state <= IDLE;
                    read_data_sig <= 1;
                end
                else if(rx_byte_counter == 9) begin
                    error_data_flag <= 1;
                    rx_state <= IDLE;
                end
                else begin
                    data_out[rx_byte_counter - 1] <= rx_buf;
                    rx_byte_counter <= rx_byte_counter + 1;
                end
            end

            if (rx_state == ACTIVE) begin
                if (sampling_counter < 15) begin
                    sampling_counter <= sampling_counter + 1;
                end
                else begin
                    sampling_counter <= 0;
                end
            end

            counter_rx <= 0;
        end

        if (counter_tx == CLK_TX) begin
            if (tx_state == ACTIVE) begin
                if (tx_byte_counter == 0) begin
                    tx <= 0;
                    tx_byte_counter <= tx_byte_counter + 1;
                end
                else if (tx_byte_counter == 9) begin
                    tx <= 1;
                    tx_busy <= 0;
                    tx_state <= IDLE;
                    tx_byte_counter <= 0;
                end
                else begin
                    tx <= data_in[tx_byte_counter - 1];
                    tx_byte_counter <= tx_byte_counter + 1;
                end
            end

            counter_tx <= 0;
        end
    end
endmodule