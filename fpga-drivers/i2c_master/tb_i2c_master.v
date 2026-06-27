`timescale 1ns / 1ps

module tb_i2c_master;

    reg sys_clk;
    reg start_sig, stop_sig, send_data_sig, read_data_sig;
    reg [7:0] data_in;
    wire [7:0] data_out;
    wire busy, ack, scl;
    tri1 sda_out;

    reg slave_drive_sda;
    reg slave_sda_bit;
    assign sda_out = slave_drive_sda ? (slave_sda_bit ? 1'bz : 1'b0) : 1'bz;

    i2c_master uut (
        .sys_clk(sys_clk),
        .start_sig(start_sig),
        .stop_sig(stop_sig),
        .send_data_sig(send_data_sig),
        .read_data_sig(read_data_sig),
        .data_in(data_in),
        .data_out(data_out),
        .busy(busy),
        .ack(ack),
        .scl(scl),
        .sda_out(sda_out)
    );

    always #10 sys_clk = ~sys_clk;

    task do_start;
        begin
            start_sig = 1; #40; start_sig = 0;
            wait(busy == 0); #200;
        end
    endtask

    task do_stop;
        begin
            stop_sig = 1; #40; stop_sig = 0;
            wait(busy == 0); #200;
        end
    endtask

    task do_send(input [7:0] val);
        begin
            data_in = val;
            send_data_sig = 1; #40; send_data_sig = 0;
            wait(busy == 0); #200;
        end
    endtask

    task do_read;
        begin
            slave_drive_sda = 1;
            read_data_sig = 1; #40; read_data_sig = 0;
            repeat(8) begin
                wait(scl == 0);
                slave_sda_bit = $random; // Symulacja różnych bitów
                wait(scl == 1);
            end
            wait(busy == 0); #200;
            slave_drive_sda = 0;
        end
    endtask

    initial begin
        $dumpfile("test_i2c_master.vcd");
        $dumpvars(0, tb_i2c_master);
        
        sys_clk = 0; start_sig = 0; stop_sig = 0; send_data_sig = 0; 
        read_data_sig = 0; slave_drive_sda = 0;

        #200;
        
        $display("--- ROZPOCZĘCIE MEGA SEKWENCJI ---");
        
        // Sekwencja 1: START -> SEND -> SEND -> READ -> STOP
        do_start;
        do_send(8'hAA);
        do_send(8'h55);
        do_read;
        do_stop;

        // Sekwencja 2: START -> READ -> READ -> SEND -> STOP
        do_start;
        do_read;
        do_read;
        do_send(8'hFF);
        do_stop;

        $display("--- KONIEC MEGA SEKWENCJI ---");
        $finish;
    end
endmodule