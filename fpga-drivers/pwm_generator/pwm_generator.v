module pwm_generator (
    input wire clk, 
    input wire [7:0] power,
    output reg pwm_out
);
    localparam CLK_FREQ = 50000000;
    localparam COUNTER_BIT_WIDTH = $clog2(CLK_FREQ);
    localparam ONE_MS = CLK_FREQ / 1000;
    localparam POWER_SCALE = ONE_MS / 255;

    reg [COUNTER_BIT_WIDTH:0] counter;
    
    initial begin
        counter = 0;
        pwm_out = 1;
    end

    always @(posedge clk) begin
        counter <= counter + 1;
        if (counter < ((32'd0 + power) * POWER_SCALE) + ONE_MS) begin
            pwm_out <= 1;
        end
        else begin
            pwm_out <= 0;
        end
        if (counter == 20 * ONE_MS) begin
            counter <= 0;
        end
        
    end
endmodule