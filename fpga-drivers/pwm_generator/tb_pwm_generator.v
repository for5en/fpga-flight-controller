`timescale 1ns / 1ps

module tb_pwm_generator;

    // Sygnały dla modułu
    reg clk;
    reg [7:0] power;
    wire pwm_out;

    // Podpięcie modułu (Unit Under Test)
    pwm_generator uut (
        .clk(clk),
        .power(power),
        .pwm_out(pwm_out)
    );

    // Generowanie zegara 50MHz (okres 20ns)
    initial clk = 0;
    always #10 clk = ~clk;

    // Scenariusz testowy
    initial begin
        // Plik do podglądu w GTKWave
        $dumpfile("test_pwm_generator.vcd");
        $dumpvars(0, tb_pwm_generator);

        // Ustawienia początkowe
        power = 0;
        
        // Czekaj 40ms (dwa pełne cykle po 20ms)
        #40000000;
        
        // Zmień moc na połowę (128)
        power = 128;
        #40000000;
        
        // Zmień moc na maksimum (255)
        power = 255;
        #40000000;
        
        $finish;
    end
endmodule