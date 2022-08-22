`timescale 1ns / 1ps

module mac_m #(

    parameter integer A_BITWIDTH = 8,
    parameter integer B_BITWIDTH = A_BITWIDTH,
    parameter integer OUT_BITWIDTH = 19,
    parameter integer C_BITWIDTH = OUT_BITWIDTH - 1
)
(
    
    input                                   clk,
    input                                   en,
    input                                   rstn,
    input [A_BITWIDTH-1:0]                  data_a, 
    input [B_BITWIDTH-1:0]                  data_b,
    input [C_BITWIDTH-1:0]                  data_c,
    output reg [OUT_BITWIDTH-1:0]           mout,
    output reg                              done
    );

localparam 
    STATE_IDLE = 2'b00, 
    STATE_MULT = 2'b01, 
    STATE_Addx = 2'b10,
    STATE_Done = 2'b11;
    
reg [OUT_BITWIDTH-1:0]          mout_temp;
reg [1:0]                       m_state;
reg [C_BITWIDTH-1:0]			c_temp;

always @( posedge clk or negedge rstn) begin
    if(!rstn) begin
        m_state <= 2'b00;
    end
    else begin
        case(m_state)
            STATE_IDLE: begin
                if(en && !done) begin
                     m_state <= STATE_MULT;
                end
                else begin
                     m_state <= STATE_IDLE;
                end
            end
            STATE_MULT: begin
                m_state <= STATE_Addx;
            end
            STATE_Addx: begin
                m_state <= STATE_Done;
            end
            STATE_Done: begin
                m_state <= STATE_IDLE;
            end
            default:;
           
        endcase
    end
end
                    

always @ (posedge clk or negedge rstn) begin
    if(!rstn) begin
        mout_temp <={OUT_BITWIDTH{1'b0}};
        mout <={OUT_BITWIDTH{1'b0}};
        done <= 1'b0;
    end
    else begin
        case(m_state)
            STATE_IDLE: begin
                done <=1'b0;
                mout_temp <= mout_temp;
            end
            
            STATE_MULT: begin
                mout_temp[OUT_BITWIDTH-1] <= data_a[A_BITWIDTH-1]^data_b[B_BITWIDTH-1];
                mout_temp[OUT_BITWIDTH-2:0] <= data_a[A_BITWIDTH-2:0] * data_b[B_BITWIDTH-2:0];
				c_temp <= data_c;
            end
            
            STATE_Addx: begin
                if(mout_temp[OUT_BITWIDTH-1]^data_c[C_BITWIDTH-1]) begin
                    if(mout_temp[OUT_BITWIDTH-2:0] > c_temp[C_BITWIDTH-2:0]) begin
                        mout_temp[OUT_BITWIDTH-2:0] <= mout_temp[OUT_BITWIDTH-2:0] - c_temp[C_BITWIDTH-2:0];
                    end
                    else begin 
                        mout_temp[OUT_BITWIDTH-1] <= c_temp[C_BITWIDTH-1];
                        mout_temp[OUT_BITWIDTH-2:0] <= c_temp[C_BITWIDTH-2:0] - mout_temp[OUT_BITWIDTH-2:0]; 
                    end
                end
                else begin 
                    mout_temp[OUT_BITWIDTH-2:0] <= c_temp[C_BITWIDTH-2:0] + mout_temp[OUT_BITWIDTH-2:0]; 
                end
            end
            
            STATE_Done: begin
                 done <= 1'b1;
                 mout <= mout_temp;
                 
            end
            default:;
       endcase
   end
end
endmodule
