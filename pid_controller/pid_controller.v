module pid_controller #(
    parameter signed [15:0] KP = 16'sd256,
    parameter signed [15:0] KI = 16'sd26,
    parameter signed [15:0] KD = 16'sd50
)(
    input wire sys_clk,

    input wire update_sig,
    input wire signed [15:0] error_in,

    output reg signed [47:0] pid_out
);
    reg signed [15:0] prev_error = 0;

    reg signed [47:0] p_term = 0;
    reg signed [47:0] i_term = 0;
    reg signed [47:0] d_term = 0;

    reg signed [47:0] integral = 0;
    reg signed [47:0] derivative = 0;

    localparam FIRST_RUN = 2'd0;
    localparam ACTIVE = 2'd1;
    localparam POSTACTIVE = 2'd2;
    
    reg [1:0] state = FIRST_RUN;

    parameter integer       KP_SHIFT = 0;
    parameter integer       KI_SHIFT = 5;
    parameter integer       KD_SHIFT = 0;
    parameter signed [47:0] I_LIMIT  = 48'sd80000000; // anti-windup NA CALCE


    initial begin
        pid_out <= 0;
    end

    always @(posedge sys_clk) begin
        if (update_sig) begin
            if (state == FIRST_RUN) begin
                integral <= $signed({{32{error_in[15]}}, error_in});
                i_term <= $signed({{32{error_in[15]}}, error_in}) * $signed({{32{KI[15]}}, KI});

                prev_error <= error_in;
                state <= ACTIVE;
            end
            else if (state == ACTIVE) begin
                if (error_in > 0 && integral > I_LIMIT) 
                    integral <= I_LIMIT;
                else if (error_in < 0 && integral < -I_LIMIT) 
                    integral <= -I_LIMIT;
                else 
                    integral <= integral + $signed({{32{error_in[15]}}, error_in});
                derivative <= error_in - prev_error;

                state <= POSTACTIVE;
            end
        end

        if (state == POSTACTIVE) begin
            p_term <= (error_in * KP) >>> KP_SHIFT;
            i_term <= ($signed(integral) * KI) >>> KI_SHIFT;
            d_term <= ($signed(derivative) * KD) >>> KD_SHIFT;

            pid_out <= ((error_in * KP) >>> KP_SHIFT) + 
                       (($signed(integral) * KI) >>> KI_SHIFT) + 
                       (($signed(derivative) * KD) >>> KD_SHIFT);

            prev_error <= error_in;
            state <= ACTIVE;
        end
    end

endmodule