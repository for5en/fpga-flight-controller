`timescale 1ns / 1ps

module tb_uart();
    reg sys_clk;
    reg send_data_sig;
    reg [7:0] data_in;
    reg rx;
    wire tx;
    wire tx_busy;
    wire read_data_sig;
    wire error_data_flag;
    wire [7:0] data_out;
    reg [7:0] val;
    reg data_captured;
    reg [7:0] captured_data;
    reg error_latched;

    uart uut (
        .sys_clk(sys_clk),
        .send_data_sig(send_data_sig),
        .tx_busy(tx_busy),
        .read_data_sig(read_data_sig),
        .error_data_flag(error_data_flag),
        .data_in(data_in),
        .data_out(data_out),
        .tx(tx),
        .rx(rx)
    );

    initial sys_clk = 0;
    always #10 sys_clk = ~sys_clk;

    localparam CLK_FREQ = 50000000;
    localparam RX_N = 50;
    localparam RX_ERROR_N = 10;
    localparam TX_N = 50;

    localparam BAUD_RATE = 100000;
    localparam BIT_PERIOD = BAUD_RATE / 10;

    initial begin
        data_captured = 0;
        error_latched = 0;
        forever @(posedge sys_clk) begin
            if (error_data_flag) error_latched = 1;
            if (read_data_sig) begin
                data_captured = 1;
                captured_data = data_out;
            end
        end
    end

    initial begin
        $dumpfile("test_uart.vcd");
        $dumpvars(0, tb_uart);

        #1000;

        $display("--- TX TEST BEGIN ---");
        test_tx();
        $display("--- TX TEST END ---");

        send_data_sig = 0;
        data_in = 0;
        rx = 1;
        #1000;

        $display("--- RX TEST BEGIN---");
        test_rx();

        $display("--- RX FRAME ERROR TEST ---");
        test_errors_rx();
        repeat(5000) @(posedge sys_clk);
        
        $display("--- RX TEST END ---");
        $finish;
    end

    task test_rx;
        repeat(RX_N) begin
            val = $urandom_range(0, 255);
            send_byte(val);
            #(BIT_PERIOD);
        end
    endtask

    task test_errors_rx;
        repeat(RX_ERROR_N) begin
            val = $urandom_range(0, 255);
            simulate_framing_error(val);
            #(BIT_PERIOD);
        end
    endtask

    task send_byte;
        input [7:0] val;
        integer j;
        begin
            data_captured = 0;
            
            rx = 0; #(BIT_PERIOD);
            for(j=0; j<8; j=j+1) begin
                rx = val[j]; #(BIT_PERIOD);
            end
            rx = 1; #(BIT_PERIOD);
            
            repeat(500) @(posedge sys_clk); 
            
            if (data_captured) begin
                if (captured_data === val) 
                    $display("RX TEST: OK! SENT: %h, RECEIVED: %h", val, captured_data);
                else 
                    $display("RX TEST: ERROR! SENT: %h, RECEIVED: %h", val, captured_data);
            end else begin
                $display("RX TEST: ERROR! FLAG read_data_sig MISSING FOR BYTE %h", val);
            end
        end
    endtask

    task simulate_framing_error;
        input [7:0] val;
        integer j;
        begin            
            error_latched = 0;
            
            rx = 0; #(BIT_PERIOD);
            for(j=0; j<8; j=j+1) begin
                rx = val[j]; #(BIT_PERIOD);
            end
            rx = 0; #(BIT_PERIOD);
            
            if (error_latched) begin
                $display("RX TEST: OK! ERROR FLAG DETECTED");
            end else begin
                $display("RX TEST: ERROR! NO ERROR FLAG DETECTED");
            end
            
            rx = 1;
            #(BIT_PERIOD);
        end
    endtask

    task test_tx_byte(input [7:0] val);
        integer j;
        begin
            data_in = val;
            send_data_sig = 1;
            #20;
            send_data_sig = 0;

            @(negedge tx);
            
            #(BIT_PERIOD / 2);
            if(tx !== 0) $display("TX TEST: ERROR! START BIT MISSING");
            #(BIT_PERIOD);

            for(j=0; j<8; j=j+1) begin
                if(tx !== val[j]) 
                    $display("TX TEST: ERROR! BIT %d: EXPECTED %b, ACTUAL %b", j, val[j], tx);
                #(BIT_PERIOD);
            end
            
            if(tx !== 1) $display("TX TEST: ERROR! STOP BIT MISSING", tx);
            #(BIT_PERIOD);
            
            $display("TX TEST: OK! BYTE %h CORRECT", val);
        end
    endtask

    task test_tx();
        repeat(TX_N) begin
            val = $urandom_range(0, 255);
            test_tx_byte(val);
            #(BIT_PERIOD);
        end
    endtask
endmodule