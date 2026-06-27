`timescale 1ns / 1ps

// =====================================================================
//  Testbench dla pid_controller — jedna os drona w ciaglym wietrze
// =====================================================================
//  Idea (prosta):
//   - jedna zmienna `gyro` = mierzona wielkosc, ktora PID ma trzymac
//     przy zadaniu (SETPOINT),
//   - `wind_at(t)` = ladna, CIAGLA funkcja wiatru (dryf + kolysanie +
//     szybkie podmuchy) — bez skokow,
//   - w petli co cykl: zmierz blad -> policz PID -> przesun os.
//
//  Os jest modelem 1. rzedu (petla predkosci): pid przeciwdziala,
//  wiatr zaklocenie, DAMPING to tlumienie. Petla chodzi z DT (5 kHz).
//
//  STROJENIE: zmieniaj KP/KI/KD ponizej i patrz na blad RMS/max oraz
//  przebiegi (gtkwave test_pid_controller.vcd: gyro_mdeg vs setpoint_mdeg).
//
//  UWAGA o 5-10 kHz: DUT akumuluje I i D "na probke", bez /dt. Przy tak
//  szybkiej petli calka narasta b. szybko i dobija do limitu +-500000
//  (windup), a pochodna (error-prev_error) kwantuje sie ~do 0. Dlatego
//  przy duzym KI widac cykl graniczny — to do poprawy w RTL (skalowanie
//  I/D przez dt), nie wina modelu.
// =====================================================================

module tb_pid_controller;

    // --- NASTAWY PID (stroj tutaj) ---
    localparam signed [15:0] KP = 16'd50;
    localparam signed [15:0] KI = 16'd15;
    localparam signed [15:0] KD = 16'd2;

    // --- DUT ---
    reg                sys_clk    = 1'b0;
    reg                update_sig = 1'b0;
    reg  signed [15:0] error_in   = 16'sd0;
    wire signed [47:0] pid_out;

    pid_controller #(.KP(KP), .KI(KI), .KD(KD)) uut (
        .sys_clk(sys_clk), .update_sig(update_sig),
        .error_in(error_in), .pid_out(pid_out)
    );

    always #10 sys_clk = ~sys_clk;   // zegar DUT 50 MHz

    // --- model osi + wiatr ---
    real DT       = 0.0002;   // krok regulacji [s] -> 5 kHz
    real SCALE    = 100.0;    // county na jednostke (0.01 / count)
    real K_ACT    = 0.02;     // wplyw pid_out na os
    real DAMPING  = 0.5;      // tlumienie osi
    real gyro     = 0.0;      // <<< mierzona wielkosc (to reguluje PID)
    real setpoint = 0.0;      // zadanie (chcemy, by gyro = setpoint)
    real wind     = 0.0;
    real t        = 0.0;

    // Ladna, CIAGLA funkcja wiatru: stala skladowa + dryf + kolysanie + podmuchy.
    // Skladowa STALA (20) to np. boczny wiatr / niewywazenie / opor przy opadaniu
    // — to ona wymaga czlonu I (samo P zostawia tu staly blad).
    function real wind_at(input real tt);
        real w;
    begin
        w =  20.0                                           // skladowa STALA
           + 30.0 * $sin(2.0 * 3.14159 * 0.3 * tt)          // powolny dryf
           + 15.0 * $sin(2.0 * 3.14159 * 1.7 * tt + 0.7)    // kolysanie
           +  8.0 * $sin(2.0 * 3.14159 * 5.0 * tt + 2.1);   // szybkie podmuchy
        wind_at = w;
    end
    endfunction

    // Jeden krok PID. DUT ma maszyne stanow (FIRST_RUN/ACTIVE/POSTACTIVE),
    // wiec potrzebuje impulsu update_sig, by policzyc nowe pid_out.
    task pid_step(input signed [15:0] err);
    begin
        @(negedge sys_clk); error_in = err; update_sig = 1'b1;
        @(posedge sys_clk);                  // ACTIVE: calka + pochodna
        @(negedge sys_clk); update_sig = 1'b0;
        @(posedge sys_clk); #1;              // POSTACTIVE: liczy pid_out
    end
    endtask

    // Sygnaly do gtkwave (x1000)
    reg signed [31:0] gyro_mdeg = 0, setpoint_mdeg = 0, wind_mdeg = 0, pid_out_dbg = 0;

    integer i, err_i, NSTEP;
    real e, err_sum, err_sq, err_max, mean, rms;

    initial begin
        $dumpfile("test_pid_controller.vcd");
        $dumpvars(0, tb_pid_controller);

        NSTEP   = $rtoi(5.0 / DT);   // 5 s symulacji
        err_sum = 0.0; err_sq = 0.0; err_max = 0.0;

        for (i = 0; i < NSTEP; i = i + 1) begin
            t        = i * DT;
            setpoint = (t < 0.5) ? 0.0 : 10.0;   // po 0.5 s: trzymaj 10
            wind     = wind_at(t);

            // zmierz blad i podaj do PID (county, klamrowanie do 16 bit)
            err_i = $rtoi((setpoint - gyro) * SCALE);
            if (err_i >  32767) err_i =  32767;
            if (err_i < -32768) err_i = -32768;
            pid_step(err_i[15:0]);

            // os 1. rzedu: pid przeciwdziala, wiatr zakloca
            gyro = gyro + (K_ACT * pid_out + wind - DAMPING * gyro) * DT;

            // podglad + metryka (liczona po ustaleniu, t > 1 s)
            gyro_mdeg     = $rtoi(gyro     * 1000.0);
            setpoint_mdeg = $rtoi(setpoint * 1000.0);
            wind_mdeg     = $rtoi(wind     * 1000.0);
            pid_out_dbg   = pid_out;
            if (t > 1.0) begin
                e       = gyro - setpoint;
                err_sum = err_sum + e;
                err_sq  = err_sq  + e * e;
                if (e > err_max)  err_max =  e;
                if (-e > err_max) err_max = -e;
            end
        end

        mean = err_sum / ($rtoi(4.0 / DT));
        rms  = $sqrt(err_sq / ($rtoi(4.0 / DT)));
        $display("======================================================");
        $display(" PID w ciaglym wietrze   KP=%0d KI=%0d KD=%0d  (%.0f Hz)",
                  KP, KI, KD, 1.0/DT);
        $display("   sredni blad : %.3f  (bias -> czy P wystarcza / czy I kasuje)", mean);
        $display("   blad RMS    : %.3f", rms);
        $display("   blad max    : %.3f", err_max);
        $display(" gtkwave: gyro_mdeg vs setpoint_mdeg (x1000)");
        $display("======================================================");
        $finish;
    end

endmodule
