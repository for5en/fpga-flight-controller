module pid_controller #(
    parameter KP = 16'd256,
    parameter KI = 16'd26,
    parameter KD = 16'd50
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
                if (error_in > 0 && integral > 48'd500000) 
                    integral <= 48'd500000;
                else if (error_in < 0 && integral < -48'd500000) 
                    integral <= -48'd500000;
                else 
                    integral <= integral + $signed({{32{error_in[15]}}, error_in});
                derivative <= error_in - prev_error;

                state <= POSTACTIVE;
            end
        end

        if (state == POSTACTIVE) begin
            p_term <= $signed({{32{error_in[15]}}, error_in}) * $signed({{32{KP[15]}}, KP});
            i_term <= $signed(integral) * $signed({{32{KI[15]}}, KI});
            d_term <= $signed({{32{derivative[15]}}, derivative}) * $signed({{32{KD[15]}}, KD});

            pid_out <= $signed({{32{error_in[15]}}, error_in}) * $signed({{32{KP[15]}}, KP}) + 
                       $signed(integral) * $signed({{32{KI[15]}}, KI}) + 
                       $signed({{32{KD[15]}}, KD}) * $signed({{32{derivative[15]}}, derivative});

            prev_error <= error_in;
            state <= ACTIVE;
        end
    end

endmodule