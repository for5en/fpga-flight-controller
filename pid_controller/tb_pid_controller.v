`timescale 1ns / 1ps

module tb_pid_controller;
    reg sys_clk;
    reg update_sig;
    reg signed [15:0] error_in;
    wire signed [47:0] pid_out;

    reg signed [15:0] gyro_raw;
    localparam signed CENTER = 16'b0100000000000000;
    
    // Tablica "wiatru" (zamiast $sin)
    reg signed [15:0] wind_data [0:99]; 
    integer i;

    reg signed [15:0] error_wind;
    reg signed [15:0] angle;
    reg signed [15:0] wind_real;
    reg signed [15:0] error_max = 0;
    reg signed [15:0] error_div = 0;

    pid_controller #(
        .KP(16'd50),  // Zamiast 256
        .KI(16'd15),   // Zamiast 26
        .KD(16'd2)    // Zamiast 50
    ) uut (
        .sys_clk(sys_clk),
        .update_sig(update_sig),
        .error_in(error_in),
        .pid_out(pid_out)
    );

    always #5 sys_clk = ~sys_clk; 

    reg signed [15:0] sin_lut [0:63];
    integer k;

    reg [31:0] fixed_angle = 0;

    initial begin
        for (k = 0; k < 64; k = k + 1) begin
            // Wypełniamy tablicę wartościami od -1000 do 1000
            sin_lut[k] = $rtoi(1000.0 * $sin(2.0 * 3.14159 * k / 64.0));
        end
    end

    initial begin
        // Wypełniamy tablicę "wiatrem" - po prostu ciąg wartości
        for (i = 0; i < 100; i = i + 1) begin
            wind_data[i] = (i < 50) ? (i * 100) : ((100-i) * 100);
        end

        $dumpfile("test_pid_controller.vcd");
        $dumpvars(0, tb_pid_controller);

        sys_clk = 0; 
        error_in = 0;
        #50;

        // Pętla przez 4000 taktów
        for (i = 0; i < 4000; i = i + 1) begin
            // Bierzemy wartość z tablicy w pętli
            fixed_angle = fixed_angle + 32'd65536; // Teraz to jest płynne!
    
            // Indeks tablicy = wycięcie 6 bitów z kąta
            wind_real = sin_lut[fixed_angle[31:26]];
            gyro_raw = CENTER + sin_lut[fixed_angle[31:26]] - (pid_out >>> 8);
            error_in = gyro_raw - CENTER; 
            error_div = (error_div * i + error_in * 10000) / (i + 1);

            if (error_max >= 0 && (error_in > error_max || error_in < -error_max)) error_max = error_in;
            if (error_max < 0 && (error_in < error_max || error_in > -error_max)) error_max = error_in;

            
            update_sig = (i % 20 == 0); // Próbkowanie co 20 taktów
            #10;
        end

        $display("MAXIMUM ERROR: ", error_max);
        $display("MEAN ERROR: ", error_div);
        $finish;
    end
endmodule