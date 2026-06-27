`timescale 1ns / 1ps

// =====================================================================
//  Testbench dla pid_controller — realistyczny model jednej osi drona
// =====================================================================
//  DLACZEGO POWSTAL:
//   - Stary test wpinal pid_out z powrotem w gyro NATYCHMIAST (petla
//     algebraiczna), bez bezwladnosci -> blad "skakal" i nie odpowiadal
//     realnemu dronowi. Calosc trwala ~40 us.
//   - Tutaj os drona jest STALYM modelem 1. rzedu (petla predkosci):
//        I * d(theta)/dt = moment - DAMPING*theta + zaklocenie
//     gdzie theta = mierzona wielkosc (np. predkosc katowa z zyroskopu),
//     a petla regulacji chodzi z realna czestotliwoscia (DT -> 5 kHz).
//   - Model jest STALY: parametry PID stroimy DO modelu, nie odwrotnie.
//
//  CO MIERZY (3 scenariusze, metryki wypisywane na koniec):
//   1) SKOK ZADANIA   -> przeregulowanie, czas narastania, czas
//                        ustalania, blad ustalony.
//   2) PODMUCH (gust) -> jak mocno zbije z toru i jak szybko wroci.
//   3) SZUM CZUJNIKA  -> drzenie wyjscia (RMS). Pokazuje, czy czlon D
//                        wzmacnia szum i czy potrzebny jest filtr.
//
//  STROJENIE PID: zmieniaj KP / KI / KD ponizej, uruchom ponownie,
//  patrz na metryki i przebiegi (gtkwave test_pid_controller.vcd).
//
//  KALIBRACJA DO TWOJEGO DRONA: sekcja "MODEL FIZYCZNY" — ustaw
//  INERTIA (bezwladnosc osi), DAMPING (tlumienie aero) i ACT_GAIN
//  (moment na jednostke pid_out) tak, by odpowiadaly Twojemu platowcowi.
//  Model jest celowo prosty (1 os, sztywne cialo) — sluzy do strojenia
//  i sprawdzenia stabilnosci, nie do certyfikacji lotu.
// =====================================================================

module tb_pid_controller;

    // ---------------------------------------------------------------
    //  STROJENIE REGULATORA  (ZMIEN TUTAJ)
    // ---------------------------------------------------------------
    localparam signed [15:0] KP = 16'd50;
    localparam signed [15:0] KI = 16'd15;
    localparam signed [15:0] KD = 16'd2;

    // ---------------------------------------------------------------
    //  Sygnaly DUT
    // ---------------------------------------------------------------
    reg                sys_clk    = 1'b0;
    reg                update_sig = 1'b0;
    reg  signed [15:0] error_in   = 16'sd0;
    wire signed [47:0] pid_out;

    pid_controller #(.KP(KP), .KI(KI), .KD(KD)) uut (
        .sys_clk   (sys_clk),
        .update_sig(update_sig),
        .error_in  (error_in),
        .pid_out   (pid_out)
    );

    always #10 sys_clk = ~sys_clk;   // 50 MHz -> okres 20 ns

    // ---------------------------------------------------------------
    //  MODEL FIZYCZNY (real — tylko symulacja, NIE syntezowalne)
    // ---------------------------------------------------------------
    real DT          = 0.0002;   // krok regulacji [s] -> 5 kHz (cel: 5-10 kHz)
    real ANGLE_SCALE = 100.0;    // county na jednostke pomiaru (10 -> 1000 cnt)
    // --- STALY model fizyczny (NIE dobieramy go do nastaw PID!) ---
    real INERTIA     = 1.0;      // bezwladnosc osi
    real DAMPING     = 0.5;      // tlumienie (aero) w petli predkosci
    real ACT_GAIN    = 0.02;     // moment na jednostke pid_out

    // WAZNE przy 5-10 kHz: ten DUT liczy calke i pochodna "na probke",
    // BEZ skalowania przez dt:
    //   - calka: integral += error co probke -> przy 5-10 kHz narasta
    //     ~5000-10000x/s i blyskawicznie dobija do limitu +-500000 (windup).
    //     Efektywne KI ~ KI * f_probkowania -> nawet KI=1 jest "ogromne".
    //   - pochodna: (error - prev_error) -> przy tak szybkiej petli zmiana
    //     bledu na probke jest mala i kwantuje sie do ~0 (czlon D slabnie).
    // Wniosek praktyczny: albo KI bardzo male/0, albo (lepiej) przeskaluj
    // czlony I/D przez dt w RTL, by nastawy nie zalezaly od czestotliwosci.

    real theta        = 0.0;     // kat [deg]
    real omega        = 0.0;     // predkosc katowa [deg/s]
    real dist_torque  = 0.0;     // moment zaklocenia (podmuch)
    real noise_amp    = 0.0;     // amplituda szumu czujnika [deg]
    real setpoint_deg = 0.0;     // zadany kat [deg]
    real noise        = 0.0;

    integer seed = 32'h0BADF00D;

    // Lustrzane wersje calkowite -> dobrze widoczne w gtkwave
    reg signed [31:0] theta_mdeg    = 0;   // kat   * 1000
    reg signed [31:0] omega_mdeg    = 0;   // omega * 1000
    reg signed [31:0] setpoint_mdeg = 0;   // zadanie * 1000
    reg signed [31:0] pid_out_dbg   = 0;
    reg signed [31:0] dist_mdeg     = 0;

    // Metryki
    real target, max_theta, t_rise10, t_rise90, t_settle;
    real ss_sum, ss_err, peak_dev, t_recover, t_now, ss_min, ss_max;
    real sum_out, sumsq_out, out_rms, theta_rms, sum_th, sumsq_th, pr_out;
    integer n_acc;

    // ---------------------------------------------------------------
    //  Jeden krok DUT: jedno wyzwolenie update -> jedna akumulacja
    //  calki + policzenie pid_out (FIRST_RUN/ACTIVE -> POSTACTIVE).
    // ---------------------------------------------------------------
    task pid_step(input signed [15:0] err);
    begin
        @(negedge sys_clk); error_in = err; update_sig = 1'b1;
        @(posedge sys_clk);                  // ACTIVE: calka + pochodna
        @(negedge sys_clk); update_sig = 1'b0;
        @(posedge sys_clk);                  // POSTACTIVE: liczy pid_out
        #1;                                  // ustalenie przypisan NBA
    end
    endtask

    // ---------------------------------------------------------------
    //  Jeden krok swiata: ZMIERZ -> REGULUJ -> SCALKUJ FIZYKE (Euler)
    // ---------------------------------------------------------------
    task world_step;
        real    meas_deg, ctrl, accel, pr;
        integer err_i, gyro_cnt, sp_cnt;
    begin
        // szum czujnika (jesli wlaczony) -> symetryczny w [-noise_amp, +noise_amp]
        if (noise_amp > 0.0)
            noise = noise_amp * (($random(seed) % 1000) / 1000.0);
        else
            noise = 0.0;

        // pomiar kata -> county
        meas_deg = theta + noise;
        gyro_cnt = $rtoi(meas_deg     * ANGLE_SCALE);
        sp_cnt   = $rtoi(setpoint_deg * ANGLE_SCALE);

        // blad w countach, ograniczenie do 16 bit
        err_i = sp_cnt - gyro_cnt;
        if (err_i >  32767) err_i =  32767;
        if (err_i < -32768) err_i = -32768;

        // policz wyjscie regulatora (DUT)
        pid_step(err_i[15:0]);

        // pid_out -> moment; obiekt PIERWSZEGO RZEDU (petla predkosci):
        //   I * d(theta)/dt = moment - DAMPING*theta + zaklocenie
        // theta = mierzona wielkosc (np. predkosc katowa z zyroskopu).
        // Model 1. rzedu jest stabilny i ladnie zbiezny (czlon I kasuje
        // blad ustalony), zgodnie z petla regulacji opartą na zyroskopie.
        pr    = pid_out;                 // signed [47:0] -> real (ze znakiem)
        ctrl  = pr * ACT_GAIN;
        accel = (ctrl + dist_torque - DAMPING * theta) / INERTIA;
        theta = theta + accel * DT;
        omega = accel;                   // pochodna (do podgladu w VCD)

        // sygnaly debug do VCD
        theta_mdeg    = $rtoi(theta        * 1000.0);
        omega_mdeg    = $rtoi(omega        * 1000.0);
        setpoint_mdeg = $rtoi(setpoint_deg * 1000.0);
        dist_mdeg     = $rtoi(dist_torque  * 1000.0);
        pid_out_dbg   = pid_out;
    end
    endtask

    integer i;
    integer N_PRE, N_STEP, N_SS, N_GUST, N_REC, N_NOISE;

    initial begin
        // Dlugosci faz w SEKUNDACH -> liczba krokow (odporne na zmiane DT)
        N_PRE   = $rtoi(0.4 / DT);   // ustalenie przy 0
        N_STEP  = $rtoi(2.0 / DT);   // analiza skoku
        N_SS    = $rtoi(0.4 / DT);   // okno bledu ustalonego
        N_GUST  = $rtoi(0.2 / DT);   // czas wiania podmuchu
        N_REC   = $rtoi(1.5 / DT);   // czas na powrot
        N_NOISE = $rtoi(1.0 / DT);   // okno pomiaru szumu

        $dumpfile("test_pid_controller.vcd");
        $dumpvars(0, tb_pid_controller);

        $display("==================================================================");
        $display(" PID drone-axis testbench   KP=%0d  KI=%0d  KD=%0d", KP, KI, KD);
        $display(" Model: INERTIA=%.3f DAMPING=%.3f ACT_GAIN=%.5f  DT=%.4fs (%.0f Hz)",
                  INERTIA, DAMPING, ACT_GAIN, DT, 1.0/DT);
        $display("==================================================================");

        // ---------------- Faza 0: ustalenie przy 0 deg ----------------
        setpoint_deg = 0.0; dist_torque = 0.0; noise_amp = 0.0;
        for (i = 0; i < N_PRE; i = i + 1) world_step;

        // ---------------- Faza 1: SKOK ZADANIA 0 -> +10 deg -----------
        target    = 10.0;
        max_theta = -1.0e9;
        t_rise10  = -1.0; t_rise90 = -1.0; t_settle = -1.0;
        setpoint_deg = target;

        for (i = 0; i < N_STEP; i = i + 1) begin
            world_step;
            t_now = (i + 1) * DT;
            if (theta > max_theta) max_theta = theta;
            if (t_rise10 < 0.0 && theta >= 0.10 * target) t_rise10 = t_now;
            if (t_rise90 < 0.0 && theta >= 0.90 * target) t_rise90 = t_now;
            // czas ustalania: ostatnia chwila poza pasmem +-2%
            if ((theta > target * 1.02) || (theta < target * 0.98)) t_settle = t_now;
        end

        // blad ustalony: srednia z ostatnich 100 krokow
        ss_sum = 0.0; ss_min = 1.0e9; ss_max = -1.0e9;
        for (i = 0; i < N_SS; i = i + 1) begin
            world_step;
            ss_sum = ss_sum + theta;
            if (theta < ss_min) ss_min = theta;
            if (theta > ss_max) ss_max = theta;
        end
        ss_err = target - (ss_sum / N_SS);

        $display("");
        $display("--- 1) SKOK ZADANIA  (0 -> %.1f deg) ---", target);
        $display("  Przeregulowanie : %.1f %%",
                  (max_theta > target) ? (max_theta - target) / target * 100.0 : 0.0);
        $display("  Czas narastania : %.3f s (10%%->90%%)",
                  (t_rise90 > 0.0 && t_rise10 > 0.0) ? (t_rise90 - t_rise10) : -1.0);
        if (t_settle >= (N_STEP - 2) * DT)
            $display("  Czas ustalania  : NIE ustalil sie w oknie %.1f s (pasmo +-2%%)", N_STEP * DT);
        else
            $display("  Czas ustalania  : %.3f s (pasmo +-2%%)", t_settle);
        $display("  Blad ustalony   : %.3f deg (srednia)", ss_err);
        $display("  Oscylacja reszt.: %.3f deg p-p (cykl graniczny w oknie ustalenia)",
                  ss_max - ss_min);

        // ---------------- Faza 2: PODMUCH (zaklocenie) ----------------
        peak_dev = 0.0; t_recover = -1.0;
        dist_torque = 60.0;                       // staly moment podmuchu
        for (i = 0; i < N_GUST; i = i + 1) begin
            world_step;
            if ((theta - target > peak_dev)) peak_dev = theta - target;
            if ((target - theta > peak_dev)) peak_dev = target - theta;
        end
        dist_torque = 0.0;
        for (i = 0; i < N_REC; i = i + 1) begin
            world_step;
            t_now = (i + 1) * DT;
            if ((theta > target * 1.02) || (theta < target * 0.98)) t_recover = t_now;
        end

        $display("");
        $display("--- 2) PODMUCH  (moment=%.0f przez %.2f s) ---", 60.0, N_GUST * DT);
        $display("  Max odchylenie  : %.3f deg", peak_dev);
        if (t_recover < 0.0)
            $display("  Czas powrotu    : < %.4f s (powrot natychmiastowy)", DT);
        else if (t_recover >= (N_REC - 2) * DT)
            $display("  Czas powrotu    : NIE wrocil w %.1f s (do pasma +-2%%)", N_REC * DT);
        else
            $display("  Czas powrotu    : %.3f s (do pasma +-2%%)", t_recover);

        // ---------------- Faza 3: SZUM CZUJNIKA -----------------------
        noise_amp = 0.3;                           // +-0.3 deg szumu
        sum_out = 0.0; sumsq_out = 0.0; sum_th = 0.0; sumsq_th = 0.0; n_acc = 0;
        for (i = 0; i < N_NOISE; i = i + 1) begin
            world_step;
            pr_out    = pid_out_dbg;               // -> real PRZED kwadratem (bez przepelnienia)
            sum_out   = sum_out   + pr_out;
            sumsq_out = sumsq_out + pr_out * pr_out;
            sum_th    = sum_th    + theta;
            sumsq_th  = sumsq_th  + theta * theta;
            n_acc     = n_acc + 1;
        end
        out_rms   = $sqrt(sumsq_out / n_acc - (sum_out / n_acc) * (sum_out / n_acc));
        theta_rms = $sqrt(sumsq_th  / n_acc - (sum_th  / n_acc) * (sum_th  / n_acc));
        noise_amp = 0.0;

        $display("");
        $display("--- 3) SZUM CZUJNIKA  (+-%.2f deg) ---", 0.3);
        $display("  Drzenie wyjscia : %.0f (RMS pid_out)", out_rms);
        $display("  Drzenie kata    : %.4f deg (RMS)", theta_rms);
        $display("  Wskazowka: duze drzenie wyjscia = czlon D wzmacnia szum");
        $display("             -> rozwaz filtr LPF na D lub mniejsze KD.");

        $display("");
        $display("==================================================================");
        $display(" Przebiegi: gtkwave test_pid_controller.vcd");
        $display(" Sygnaly:   setpoint_mdeg, theta_mdeg, omega_mdeg, pid_out_dbg");
        $display("            (wartosci *1000; np. theta_mdeg=10000 => 10.000 deg)");
        $display("==================================================================");
        $finish;
    end

endmodule
