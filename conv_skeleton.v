module conv #
(
	parameter integer C_S00_AXIS_TDATA_WIDTH	 = 32
)
(	 //AXI-STREAM
	input wire										clk,
	input wire										rstn,
	output wire										S_AXIS_TREADY,
	input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0]		S_AXIS_TDATA,
	input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0]	S_AXIS_TKEEP,
	input wire										S_AXIS_TUSER,
	input wire										S_AXIS_TLAST,
	input wire										S_AXIS_TVALID,
	input wire										M_AXIS_TREADY,
	output wire										M_AXIS_TUSER,
	output wire [C_S00_AXIS_TDATA_WIDTH-1 : 0]		M_AXIS_TDATA,
	output wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0]	M_AXIS_TKEEP,
	output wire										M_AXIS_TLAST,
	output wire										M_AXIS_TVALID,

	 //Control
	input											conv_start,
	output reg 										start_response,
    output reg         								F_writedone,
    output reg         								B_writedone,
    output reg         								W_writedone,
	output reg										cal_done,
	output reg										conv_done,
	input wire [7:0] 								data_length,
	input wire [8:0]								channel_length,
	input [2:0] 									COMMAND
		
    ,output reg [2:0] state
	,output reg [7:0] result_data_0
	,output reg [7:0] result_data_1
	,output reg [7:0] result_data_2
	,output reg [7:0] result_data_3
);
	localparam STATE_IDLE = 0;
	localparam STATE_GETI = 1;	//get input data
	localparam STATE_GETB = 2;	//get bias
	localparam STATE_GETW = 3;	//get weight
	localparam STATE_CALC = 4;	//calculate convolution
	localparam STATE_ACTV = 5;	//calculate ReLU
	localparam STATE_SEND = 6;	//send data to top
	localparam STATE_DONE = 7;	//conv end

		
	reg												m_axis_tuser;
	reg [C_S00_AXIS_TDATA_WIDTH-1 : 0]				m_axis_tdata;
	reg [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0]			m_axis_tkeep;
	reg												m_axis_tlast;
	reg												m_axis_tvalid;
	reg												s_axis_tready_i;
	reg												s_axis_tready_w;
	reg 											s_axis_tready_b;
	
	wire [11:0] input_channel_size;
	wire [11:0] data_row_size;
	
	//reg	[3:0]										state;
	reg [2:0]										n_state;
	
	reg [7:0] 										data_a[8:0];
	reg [7:0] 										data_b[8:0];
	reg [22:0] 										data_c[8:0];
	wire [22:0] 									psum[8:0];
	reg 											en_mac;
	wire											done_mac_i[8:0];
	
	wire 											done_mac;
	
	reg												en_sum;
	wire [23:0]										sum_out_1[3:0];
	wire [24:0]										sum_out_2[1:0];
	wire [25:0]										sum_out_3;
	wire [26:0]										sum_out_4;
	
	
	wire [7:0] next_in_ch_count;
	
	reg weight_write;
	reg [11:0] weight_addr_32;
	reg [31:0] weight_din_32;
	
	reg [7:0] weight_addr[8:0];
	reg [7:0] weight_din[8:0];
	wire [7:0] weight_dout[8:0];
	wire weight_en;
	reg weight_we[8:0];
	reg weight_en_w, weight_en_c;
	
	
    reg feature_write;
    reg [10:0] feature_addr_in[2:0];
    
    reg [10:0] feature_addra[2:0];
    reg [10:0] feature_addrb[2:0];
    reg [31:0] feature_din[2:0];
    //wire [31:0] feature_douta[2:0];
    //wire [31:0] feature_doutb[2:0];
    wire feature_en;
    reg feature_we;
    reg feature_en_i, feature_en_c;
	
	
    wire [63:0] feature_cache[2:0];
    reg [7:0] input_width;
    reg [8:0] input_ch;
    reg [15:0] counter_x;
    reg [7:0] counter_y;
    
    reg [7:0] bias[255:0];
    reg [8:0] output_ch;
    reg [7:0] counter_b;    
    reg [1:0] B_state;
    
    
    reg [11:0] counter_w;
    reg [7:0] out_ch_count;    
    reg [1:0] W_state;
        
    
    reg          to_send;
    reg    [1:0] sram_delay;
    reg [2:0] calc_state;
    reg [7:0] in_ch_count;
    reg [4:0] row_count;
    reg [4:0] col_count;
    reg [3:0] sum_count;
    reg [7:0] result_data[3:0];
    reg calc_done;
    
    
    reg [10:0] data_index[2:0];
    reg [12:0] data_index_buff[2:0];
    
    
    reg send_state;
    

	assign next_in_ch_count = in_ch_count + 1;
	
	
	
	assign weight_en = weight_en_w || weight_en_c;
	
	
	sram_8x256 weightmem_0(
		.addra(weight_addr[0]),
		.clka(clk),
		.dina(weight_din[0]),
		.douta(weight_dout[0]),
		.ena(weight_en),
		.wea(weight_we[0])
	);
	sram_8x256 weightmem_1(
		.addra(weight_addr[1]),
		.clka(clk),
		.dina(weight_din[1]),
		.douta(weight_dout[1]),
		.ena(weight_en),
		.wea(weight_we[1])
	);
	sram_8x256 weightmem_2(
		.addra(weight_addr[2]),
		.clka(clk),
		.dina(weight_din[2]),
		.douta(weight_dout[2]),
		.ena(weight_en),
		.wea(weight_we[2])
	);
	sram_8x256 weightmem_3(
		.addra(weight_addr[3]),
		.clka(clk),
		.dina(weight_din[3]),
		.douta(weight_dout[3]),
		.ena(weight_en),
		.wea(weight_we[3])
	);
	sram_8x256 weightmem_4(
		.addra(weight_addr[4]),
		.clka(clk),
		.dina(weight_din[4]),
		.douta(weight_dout[4]),
		.ena(weight_en),
		.wea(weight_we[4])
	);
	sram_8x256 weightmem_5(
		.addra(weight_addr[5]),
		.clka(clk),
		.dina(weight_din[5]),
		.douta(weight_dout[5]),
		.ena(weight_en),
		.wea(weight_we[5])
	);
	sram_8x256 weightmem_6(
		.addra(weight_addr[6]),
		.clka(clk),
		.dina(weight_din[6]),
		.douta(weight_dout[6]),
		.ena(weight_en),
		.wea(weight_we[6])
	);
	sram_8x256 weightmem_7(
		.addra(weight_addr[7]),
		.clka(clk),
		.dina(weight_din[7]),
		.douta(weight_dout[7]),
		.ena(weight_en),
		.wea(weight_we[7])
	);
	sram_8x256 weightmem_8(
		.addra(weight_addr[8]),
		.clka(clk),
		.dina(weight_din[8]),
		.douta(weight_dout[8]),
		.ena(weight_en),
		.wea(weight_we[8])
	);
	
	reg [7:0] WEIGHT_ADDR_3X3[3:0];
	reg [13:0] weight_addr_residue[3:0];
	//1x1 32bit to 3x3 8bit address interface
	always @(*) begin
		weight_we[0] = 0;
		weight_we[1] = 0;
		weight_we[2] = 0;
		weight_we[3] = 0;
		weight_we[4] = 0;
		weight_we[5] = 0;
		weight_we[6] = 0;
		weight_we[7] = 0;
		weight_we[8] = 0;
		
		if(weight_write) begin
			WEIGHT_ADDR_3X3[0] = (weight_addr_32 + 0)/9;
			WEIGHT_ADDR_3X3[1] = (weight_addr_32 + 1)/9;
			WEIGHT_ADDR_3X3[2] = (weight_addr_32 + 2)/9;
			WEIGHT_ADDR_3X3[3] = (weight_addr_32 + 3)/9;
			
			weight_addr_residue[0] = (weight_addr_32 + 0)%9;
			weight_addr_residue[1] = (weight_addr_32 + 1)%9;
			weight_addr_residue[2] = (weight_addr_32 + 2)%9;
			weight_addr_residue[3] = (weight_addr_32 + 3)%9;
			
			weight_addr[weight_addr_residue[0]] = WEIGHT_ADDR_3X3[0];
			weight_addr[weight_addr_residue[1]] = WEIGHT_ADDR_3X3[1];
			weight_addr[weight_addr_residue[2]] = WEIGHT_ADDR_3X3[2];
			weight_addr[weight_addr_residue[3]] = WEIGHT_ADDR_3X3[3];
			
			weight_we[weight_addr_residue[0]] = 1;
			weight_we[weight_addr_residue[1]] = 1;
			weight_we[weight_addr_residue[2]] = 1;
			weight_we[weight_addr_residue[3]] = 1;
			
			weight_din[weight_addr_residue[0]] = weight_din_32[7 :0 ];
			weight_din[weight_addr_residue[1]] = weight_din_32[15:8 ];
			weight_din[weight_addr_residue[2]] = weight_din_32[23:16];
			weight_din[weight_addr_residue[3]] = weight_din_32[31:24];
		end	
		else begin
			weight_addr[0] = next_in_ch_count;
			weight_addr[1] = next_in_ch_count;
			weight_addr[2] = next_in_ch_count;
			weight_addr[3] = next_in_ch_count;
			weight_addr[4] = next_in_ch_count;
			weight_addr[5] = next_in_ch_count;
			weight_addr[6] = next_in_ch_count;
			weight_addr[7] = next_in_ch_count;
			weight_addr[8] = next_in_ch_count;
		end
	end
	
	
	assign feature_en = feature_en_i || feature_en_c;

	
	sram_32x2048 featuremem_0(
		.addra(feature_addra[0]),
		.clka(clk),
		.dina(feature_din[0]),
		.douta(feature_cache[0][31:0]),
		.ena(feature_en),
		.wea(feature_we),
		.addrb(feature_addrb[0]),
		.clkb(clk),
		.dinb(32'b0),
		.doutb(feature_cache[0][63:32]),
		.enb(feature_en),
		.web(1'b0)
	);
	sram_32x2048 featuremem_1(
		.addra(feature_addra[1]),
		.clka(clk),
		.dina(feature_din[1]),
		.douta(feature_cache[1][31:0]),
		.ena(feature_en),
		.wea(feature_we),
		.addrb(feature_addrb[1]),
		.clkb(clk),
		.dinb(32'b0),
		.doutb(feature_cache[1][63:32]),
		.enb(feature_en),
		.web(1'b0)
	);
	sram_32x2048 featuremem_2(
		.addra(feature_addra[2]),
		.clka(clk),
		.dina(feature_din[2]),
		.douta(feature_cache[2][31:0]),
		.ena(feature_en),
		.wea(feature_we),
		.addrb(feature_addrb[2]),
		.clkb(clk),
		.dinb(32'b0),
		.doutb(feature_cache[2][63:32]),
		.enb(feature_en),
		.web(1'b0)
	);
	
	always @(*) begin
		if(feature_we) begin
			feature_addra[0] = feature_addr_in[0];
			feature_addra[1] = feature_addr_in[1];
			feature_addra[2] = feature_addr_in[2];
			feature_addrb[0] = 0;
			feature_addrb[1] = 0;
			feature_addrb[2] = 0;
		end
		else begin
			feature_addra[0] = data_index[0]-1;
			feature_addra[1] = data_index[1]-1;
			feature_addra[2] = data_index[2]-1;
			feature_addrb[0] = data_index[0];
			feature_addrb[1] = data_index[1];
			feature_addrb[2] = data_index[2];
		end
	
	end
	
	

	assign S_AXIS_TREADY = s_axis_tready_i || s_axis_tready_b || s_axis_tready_w;
	assign M_AXIS_TDATA = m_axis_tdata;
	assign M_AXIS_TLAST = m_axis_tlast;
	assign M_AXIS_TVALID = m_axis_tvalid;
	assign M_AXIS_TUSER = 1'b0;
	assign M_AXIS_TKEEP = {(C_S00_AXIS_TDATA_WIDTH/8) {1'b1}}; 
	
	assign data_row_size = input_width;
	assign input_channel_size = data_row_size * input_width;
	
	
	
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_0 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[0]), 
		.data_b(data_b[0]),
		.data_c(data_c[0]),
		.mout(psum[0]),
		.done(done_mac_i[0])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_1 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[1]), 
		.data_b(data_b[1]),
		.data_c(data_c[1]),
		.mout(psum[1]),
		.done(done_mac_i[1])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_2 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[2]), 
		.data_b(data_b[2]),
		.data_c(data_c[2]),
		.mout(psum[2]),
		.done(done_mac_i[2])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_3 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[3]), 
		.data_b(data_b[3]),
		.data_c(data_c[3]),
		.mout(psum[3]),
		.done(done_mac_i[3])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_4 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[4]), 
		.data_b(data_b[4]),
		.data_c(data_c[4]),
		.mout(psum[4]),
		.done(done_mac_i[4])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_5 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[5]), 
		.data_b(data_b[5]),
		.data_c(data_c[5]),
		.mout(psum[5]),
		.done(done_mac_i[5])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_6 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[6]), 
		.data_b(data_b[6]),
		.data_c(data_c[6]),
		.mout(psum[6]),
		.done(done_mac_i[6])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_7 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[7]), 
		.data_b(data_b[7]),
		.data_c(data_c[7]),
		.mout(psum[7]),
		.done(done_mac_i[7])
	);
	mac_m #(.A_BITWIDTH(8), .C_BITWIDTH(23), .OUT_BITWIDTH(23)) 
	 u_mac_8 (
		.clk(clk),
		.en(en_mac),
		.rstn(rstn),
		.data_a(data_a[8]), 
		.data_b(data_b[8]),
		.data_c(data_c[8]),
		.mout(psum[8]),
		.done(done_mac_i[8])
	);
	
	wire [22:0] currbias;
	assign currbias = {bias[out_ch_count][7], 9'b0, bias[out_ch_count][6:0], 6'b0};
	
	
	always @(*) begin
		data_b[0] = weight_dout[0];
		data_b[1] = weight_dout[1];
		data_b[2] = weight_dout[2];
		data_b[3] = weight_dout[3];
		data_b[4] = weight_dout[4];
		data_b[5] = weight_dout[5];
		data_b[6] = weight_dout[6];
		data_b[7] = weight_dout[7];
		data_b[8] = weight_dout[8];
		data_c[0] = (in_ch_count == 0)? 0 : psum[0];
		data_c[1] = (in_ch_count == 0)? 0 : psum[1];
		data_c[2] = (in_ch_count == 0)? 0 : psum[2];
		data_c[3] = (in_ch_count == 0)? 0 : psum[3];
		data_c[4] = (in_ch_count == 0)? 0 : psum[4];
		data_c[5] = (in_ch_count == 0)? 0 : psum[5];
		data_c[6] = (in_ch_count == 0)? 0 : psum[6];
		data_c[7] = (in_ch_count == 0)? 0 : psum[7];
		data_c[8] = (in_ch_count == 0)? currbias : psum[8];
	end
	assign done_mac = done_mac_i[0] && done_mac_i[1] && done_mac_i[2] && done_mac_i[3] && done_mac_i[4] && done_mac_i[5] && done_mac_i[6] && done_mac_i[7] && done_mac_i[8];
	
	
	
	
	wire done_sum_1_0, done_sum_1_1, done_sum_1_2, done_sum_1_3, done_sum_2_0, done_sum_2_1;
	wire done_sum_1, done_sum_2, done_sum_3, done_sum_4;
	
	assign done_sum_1 = done_sum_1_0 && done_sum_1_1 && done_sum_1_2 && done_sum_1_3;
	assign done_sum_2 = done_sum_2_0 && done_sum_2_1;
	
	
	//stage 1
	mac_m #(.A_BITWIDTH(23), .OUT_BITWIDTH(24)) sum_1_1 (
		.clk(clk),
		.en(en_sum),
		.rstn(rstn),
		.data_a(psum[0]),
		.data_b(1),
		.data_c(psum[1]),
		.mout(sum_out_1[0]),
		.done(done_sum_1_0)
	);
	
	mac_m #(.A_BITWIDTH(23), .OUT_BITWIDTH(24)) sum_1_2 (
		.clk(clk),
		.en(en_sum),
		.rstn(rstn),
		.data_a(psum[2]),
		.data_b(1),
		.data_c(psum[3]),
		.mout(sum_out_1[1]),
		.done(done_sum_1_1)
	);
	
	mac_m #(.A_BITWIDTH(23), .OUT_BITWIDTH(24)) sum_1_3 (
		.clk(clk),
		.en(en_sum),
		.rstn(rstn),
		.data_a(psum[4]),
		.data_b(1),
		.data_c(psum[5]),
		.mout(sum_out_1[2]),
		.done(done_sum_1_2)
	);
	
	mac_m #(.A_BITWIDTH(23), .OUT_BITWIDTH(24)) sum_1_4 (
		.clk(clk),
		.en(en_sum),
		.rstn(rstn),
		.data_a(psum[6]),
		.data_b(1),
		.data_c(psum[7]),
		.mout(sum_out_1[3]),
		.done(done_sum_1_3)
	);
	
	//stage 2
	mac_m #(.A_BITWIDTH(24), .OUT_BITWIDTH(25)) sum_2_1 (
		.clk(clk),
		.en(done_sum_1),
		.rstn(rstn),
		.data_a(sum_out_1[0]),
		.data_b(1),
		.data_c(sum_out_1[1]),
		.mout(sum_out_2[0]),
		.done(done_sum_2_0)
	);
	
	mac_m #(.A_BITWIDTH(24), .OUT_BITWIDTH(25)) sum_2_2 (
		.clk(clk),
		.en(done_sum_1),
		.rstn(rstn),
		.data_a(sum_out_1[2]),
		.data_b(1),
		.data_c(sum_out_1[3]),
		.mout(sum_out_2[1]),
		.done(done_sum_2_1)
	);
	
	//stage 3
	mac_m #(.A_BITWIDTH(25), .OUT_BITWIDTH(26)) sum_3_1 (
		.clk(clk),
		.en(done_sum_2),
		.rstn(rstn),
		.data_a(sum_out_2[0]),
		.data_b(1),
		.data_c(sum_out_2[1]),
		.mout(sum_out_3),
		.done(done_sum_3)
	);
	
	//stage 4
	mac_m #(.A_BITWIDTH(26), .OUT_BITWIDTH(27)) sum_4_1 (
		.clk(clk),
		.en(done_sum_3),
		.rstn(rstn),
		.data_a(sum_out_3),
		.data_b(1),
		.data_c({psum[8][22], 3'b0, psum[8][21:0]}),
		.mout(sum_out_4),
		.done(done_sum_4)
	);
	
	
	
	//state machine
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			state <= STATE_IDLE;
		end
		else begin
			state <= n_state;
		end
	end
	
	//calculate next state
	always @(*) begin
		case(state)
			STATE_IDLE : begin
					n_state = start_response? STATE_GETI : STATE_IDLE;
				end
			STATE_GETI : begin
					n_state = (COMMAND == 3'b010) ? STATE_GETB : STATE_GETI;
				end
			STATE_GETB : begin
					n_state = (COMMAND == 3'b100) ? STATE_GETW : STATE_GETB;
				end
			STATE_GETW : begin
					n_state = W_writedone? STATE_CALC : STATE_GETW;
				end
			STATE_CALC : begin
					n_state = to_send? STATE_SEND : STATE_CALC;
				end
		    STATE_SEND : begin
					n_state = cal_done? (send_state? STATE_DONE : STATE_SEND) : (send_state? (calc_done? STATE_GETW : STATE_CALC) : STATE_SEND);
				end
			STATE_DONE: begin 
					n_state = (COMMAND == 3'b000)? STATE_IDLE : STATE_DONE;
				end
		endcase
	end
	
	
	//STATE_IDLE : Literally, idleing
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			start_response <= 0;
		end
		else if(state == STATE_IDLE) begin
			if(conv_start)
				start_response <= 1;
		end
		else begin
			start_response <= 0;
		end
	end
	
	reg [1:0] I_state;
	
	//STATE_GETI : store input data into internal sram
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			counter_x <= 0;
			counter_y <= 0;
			s_axis_tready_i <= 0;
			input_ch <= 0;
			input_width <= 0;
			F_writedone <= 0;
			feature_en_i <= 0;
			feature_we <= 0;
			I_state <= 0;
		end
		else if(state == STATE_GETI && !F_writedone) begin
			case(I_state)
				0: begin
					s_axis_tready_i <= 1;
					input_width <= data_length;
					input_ch <= channel_length;
					I_state <= 1;
					counter_x <= 0;
					counter_y <= 0;
					feature_en_i <= 1;
				end
				1: begin
					if(!S_AXIS_TREADY) begin
						counter_x <= counter_x + 1;
						s_axis_tready_i <= 1;
					end
					else if(S_AXIS_TVALID) begin
						feature_we <= 1;
						feature_addr_in[0] <= counter_x;
						feature_addr_in[1] <= counter_x;
						feature_addr_in[2] <= counter_x;
						feature_din[0] <= S_AXIS_TDATA;
						feature_din[1] <= S_AXIS_TDATA;
						feature_din[2] <= S_AXIS_TDATA;
						//counter_x <= counter_x + 1;
						s_axis_tready_i <= 0;
						if(S_AXIS_TLAST) begin
							counter_y <= counter_y + 1;
							if(counter_y == input_ch -1)
								I_state <= 2;
						end
					end
					else begin
						feature_we <= 0;
					end
				end
				2: begin
					s_axis_tready_i <= 0;
					F_writedone <= 1;
					feature_we <= 0;
					feature_en_i <= 0;
					I_state <= 3;
				end
				3: begin
				
				end
			endcase
		end	
		else if(F_writedone && COMMAND == 3'b010) begin
			F_writedone <= 0;
			I_state <= 0;
		end
	end
	
	
	
	
	//STATE_GETB : store bias data into internal sram
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			counter_b <= 0;
			s_axis_tready_b <= 0;
			output_ch <= 0;
			B_writedone <= 0;
			B_state <= 0;
		end
		else if(state == STATE_GETB && !B_writedone) begin
			case(B_state)
				0: begin
					s_axis_tready_b <= 1;
					output_ch <= channel_length;
					B_state <= 1;
					counter_b <= 0;
				end
				1: begin
					if(S_AXIS_TVALID) begin
						bias[counter_b + 0] <= S_AXIS_TDATA[7 :0 ];
						bias[counter_b + 1] <= S_AXIS_TDATA[15:8 ];
						bias[counter_b + 2] <= S_AXIS_TDATA[23:16];
						bias[counter_b + 3] <= S_AXIS_TDATA[31:24];
						
						counter_b <= counter_b + 4;
						
						if(S_AXIS_TLAST) begin
							B_state <= 2;
						end
					end
				end
				2: begin
					s_axis_tready_b <= 0;
					B_writedone <= 1;
					B_state <= 3;
				end
				3: begin
				
				end
			endcase
		end	
		else if(B_writedone && COMMAND == 3'b100) begin
			B_writedone <= 0;
			B_state <= 0;
		end
	end
	
	//STATE_GETW : store weight data into internal sram
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			W_state <= 0;
			counter_w <= 0;
			W_writedone <= 0;
			s_axis_tready_w <= 0;
			weight_en_w <= 0;
			weight_write <= 0;
		end
		else if(state == STATE_GETW && !W_writedone) begin
			case(W_state)
				0: begin
					s_axis_tready_w <= 1;
					W_state <= 1;
					counter_w <= 0;
					weight_en_w <= 1;
				end
				1: begin
					 if(!S_AXIS_TREADY) begin
						 counter_w <= counter_w + 4;
						 s_axis_tready_w <= 1;
					 end
					 else if(S_AXIS_TVALID) begin
						weight_write <= 1;
						weight_addr_32 <= counter_w;
						weight_din_32 <= S_AXIS_TDATA;
						
						counter_w <= counter_w + 4;
						
						//s_axis_tready_w <= 0;
						
						if(S_AXIS_TLAST) begin
							s_axis_tready_w <= 0;	//turn off tready when transmission is over
							W_writedone <= 1;
						end
					end
					else begin
						weight_write <= 0;
					end
				end
				2: begin
					weight_en_w <= 0;
					weight_write <= 0;
					s_axis_tready_w <= 0;
					W_writedone <= 1;
					W_state <= 3;
				end
				3: begin
				
				end
			endcase
		end	
		else if(W_writedone) begin
			weight_write <= 0;
			weight_en_w <= 0;
			W_writedone <= 0;
			W_state <= 0;
		end
	end 
	
	
	
	
	
	
	//STATE_CALC : calculate convolution for one channel
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			in_ch_count <= -1;
			out_ch_count <= 0;	
			row_count <= 0;
			col_count <= 0;
			calc_state <= 0;
			sum_count <= 0;
			to_send <= 0;
			calc_done <= 0;
			feature_en_c <= 0;
			weight_en_c <= 0;
			cal_done <= 0;
		end
		else if(state == STATE_CALC) begin
			if(calc_state == 0) begin
				//W_writedone <= 0;
				in_ch_count <= -1;
				to_send <= 0;
				calc_state <= (n_state == STATE_CALC);
				en_mac <= 0;
				sum_count <= 0;
				sram_delay <= 3;
			end
			
			else if(calc_state == 1) begin
				feature_en_c <= 1;
				weight_en_c <= 1;
				sram_delay <= sram_delay -1;
				if(sram_delay == 0) begin
					in_ch_count <= 0;
					en_mac <= 1;
					calc_state <= 2;
				end
			end
			
			else if(calc_state == 2)begin		//accumulate 3x3
				if(done_mac) begin
					in_ch_count <= next_in_ch_count;
					if(in_ch_count == input_ch-1) begin
						calc_state <= 3;
						en_mac <= 0;
						weight_en_c <= 0;
					end
					else begin
						weight_en_c <= 1;
						en_mac <= 1;
						//result_data_0 <= data_a[0];
						//result_data_1 <= data_b[0];
					end
				end
				else begin
					en_mac <= 0;
				end
			end
			
			else if(calc_state == 3) begin		//sum 3x3
				feature_en_c <= 0;
				en_sum <= 1;
				calc_state <= 4;
			end
			else if(calc_state == 4) begin
				en_sum <= 0;
				calc_state <= 5;
			end
			else if(calc_state == 5 && done_sum_4) begin
				//advance and recalculate
				calc_state <= 0;
				
				result_data[data_index_buff[0][1:0]] <= sum_out_4[26]? 0 : |sum_out_4[25:13] ? 7'b1111111 : sum_out_4[12:6];
				
				if(data_index_buff[0][1:0] == 2'b11) begin
					to_send <= 1;
				end
				
				//variable mod operation is too expensive...
				//instead, use if/else
				if(col_count == input_width-1) begin
					col_count <= 0;
					if(row_count == input_width-1) begin
						calc_done <= 1;
						row_count <= 0;
						if(out_ch_count == output_ch - 1) begin
							out_ch_count <= 0;
							cal_done <= 1;
						end
						else begin
							out_ch_count <= out_ch_count + 1;
						end
					end
					else begin
						row_count <= row_count + 1;
					end
				end
				else begin
					col_count <= col_count + 1;
				end
			end
		end
		else begin
			in_ch_count <= -1;
			if(send_state == 1)
				calc_done <= 0;
			if(cal_done == 1 && COMMAND == 3'b101)begin
				cal_done <= 0;
			end
		end
	end
	
	reg [6:0] f_cache_idx[2:0];
	
	
	always @(*) begin
		data_index_buff[0] = ( (input_channel_size * next_in_ch_count) + data_row_size * (row_count-1) + col_count);
		data_index_buff[1] = ( (input_channel_size * next_in_ch_count) + data_row_size * (row_count) + col_count);
		data_index_buff[2] = ( (input_channel_size * next_in_ch_count) + data_row_size * (row_count+1) + col_count);
		
		//data_index[0] = {data_index_buff[0][12:3], 1'b0};
		//data_index[1] = {data_index_buff[1][12:3], 1'b0};
		//data_index[2] = {data_index_buff[2][12:3], 1'b0};
		
		data_index[0] = (data_index_buff[0]+2)/4;
		data_index[1] = (data_index_buff[1]+2)/4;
		data_index[2] = (data_index_buff[2]+2)/4;
		
		case(data_index_buff[0][1:0])
			0:	f_cache_idx[0] = 32;
			1:	f_cache_idx[0] = 40;
			2:	f_cache_idx[0] = 16;
			3:	f_cache_idx[0] = 24;
		endcase
		
		case(data_index_buff[1][1:0])
			0:	f_cache_idx[1] = 32;
			1:	f_cache_idx[1] = 40;
			2:	f_cache_idx[1] = 16;
			3:	f_cache_idx[1] = 24;
		endcase
		
		case(data_index_buff[2][1:0])
			0:	f_cache_idx[2] = 32;
			1:	f_cache_idx[2] = 40;
			2:	f_cache_idx[2] = 16;
			3:	f_cache_idx[2] = 24;
		endcase
		
		
		data_a[0] = (col_count == 0) || (row_count == 0)?								0 : feature_cache[0][(f_cache_idx[0]-8)+:8 ];
		data_a[1] = (row_count == 0)?													0 : feature_cache[0][(f_cache_idx[0]  )+:8 ];
		data_a[2] = (col_count == input_width-1) || (row_count == 0)? 					0 : feature_cache[0][(f_cache_idx[0]+8)+:8 ];

		data_a[3] = (col_count == 0)? 													0 : feature_cache[1][(f_cache_idx[1]-8)+:8 ];
		data_a[4] = 																		feature_cache[1][(f_cache_idx[1]  )+:8 ];
		data_a[5] = (col_count == input_width-1)? 										0 : feature_cache[1][(f_cache_idx[1]+8)+:8 ];

		data_a[6] = (col_count == 0) || (row_count == input_width - 1)? 				0 : feature_cache[2][(f_cache_idx[2]-8)+:8 ];
		data_a[7] = (row_count == input_width - 1)?										0 : feature_cache[2][(f_cache_idx[2]  )+:8 ];
		data_a[8] = (col_count == input_width - 1) || (row_count == input_width - 1)? 	0 : feature_cache[2][(f_cache_idx[2]+8)+:8 ];
		
	end	
	
	always @(*) begin
		//result_data_0 = input_width;
		result_data_1 = data_b[0];
		result_data_2 = sum_out_3;
		result_data_3 = sum_out_4;
	end
	
	//STATE_SEND
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			m_axis_tvalid <= 0;
			m_axis_tlast <= 0;
			send_state <= 0;
		end
		else if(state == STATE_SEND) begin
			case(send_state)
			0: begin
				m_axis_tvalid <= 1;
				m_axis_tdata[7 :0 ] <= result_data[0];
				m_axis_tdata[15:8 ] <= result_data[1];
				m_axis_tdata[23:16] <= result_data[2];
				m_axis_tdata[31:24] <= result_data[3];
				if(cal_done)
					m_axis_tlast <= 1;
				if(M_AXIS_TREADY)
					send_state <= 1;
			end
			1: begin
				m_axis_tvalid <= 0;
				m_axis_tlast <= 0;
				send_state <= 0;
			end
			endcase
		end
	end
	
	
	
	//STATE_DONE
	always @(posedge clk, negedge rstn) begin
		if(!rstn) begin
			conv_done <= 0;
		end
		else if(state == STATE_DONE) begin
			conv_done <= 1;
		end	
		else if(COMMAND == 3'b000) begin
			conv_done <= 0;  
		end
	end
	

endmodule
