
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  蕎麦粉コア (Sobako Koa) Version 00
  portability and Small size RISC-V core       # #         #        #     # #     RRRR   III   SSSS    CCCC   V     V
                                             #######    #######   # # #   # #     R   R   I   S    S  C    C  V    V
                                               # #         #      # # #  #   #    R   R   I   S       C       V   V
  RISC-V ABI suport : RV32I                   #####      #####    # # #  #   #    RRRR    I    SSSS   C       V  V
                                               #           #      # # #  #   #    R R     I        S  C       V V
  Copyright 2022-2025 yoshiyuki.takeda       #######   #########  # # # #     #   R  R    I   S    S  C    C  VV
                                              #   #        #        #   #     #   R   R  III   SSSS    CCCC   V
                                             # ### #      #####   ##### #     #  
                                            #  # #  #    #   #      #    #####     CCCC    OOOO   RRRR   EEEEE
                                             #######    # # #     # # #    # #    C    C  O    O  R   R  E
                                             #     #       #      # # #    # #    C       O    O  R   R  E   
                                             # ### #      # #     # # #   #  #    C       O    O  RRRR   EEEEE
                                             # # # #     #   #    # # #   #  #    C       O    O  R R    E   
                                             # ### #    #     #   # # #   #  #    C    C  O    O  R  R   E
                                             #     #   #       #    #        #     CCCC    OOOO   R   R  EEEEE

 Lisence
   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
      in the documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
   BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
    IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   (JP)ライセンス  
    ソースコード形式かバイナリ形式か、変更するかしないかを問わず、以下の条件を満たす場合に限り、再頒布および使用が許可されます。  

    1. ソースコードを再頒布する場合、上記の著作権表示、本条件一覧、および下記免責条項を含めること。  
    2. バイナリ形式で再頒布する場合、頒布物に付属のドキュメント等の資料に、上記の著作権表示、本条件一覧、および下記免責条項を含めること。  

     本ソフトウェアは、著作権者およびコントリビューターによって「現状のまま」提供されており、明示黙示を問わず、商業的な使用可能性、  
    および特定の目的に対する適合性に関する暗黙の保証も含め、またそれに限定されない、いかなる保証もありません。  
     著作権者もコントリビューターも、事由のいかんを問わず、 損害発生の原因いかんを問わず、かつ責任の根拠が契約であるか厳格責任であるか（過失その他の）不法行為であるかを問わず、  
    仮にそのような損害が発生する可能性を知らされていたとしても、本ソフトウェアの使用によって発生した（代替品または代用サービスの調達、使用の喪失、データの喪失、利益の喪失、業務の中断も含め、またそれに限定されない）  
    直接損害、間接損害、偶発的な損害、特別損害、懲罰的損害、または結果損害について、一切責任を負わないものとします。  

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

`default_nettype none

/* RV32I CPU core */
module riscv32core_rv32i( input wire reset,clk,NMI_S,INT_S , input wire [31:0] inst_data , PRDATA , 
							 output wire [31:0] PADDR ,output wire [31:0] inst_addr , PWDATA , 
                             output wire PENABLE , PWRITE , PSEL , s_ext , output wire [1:0] Awidth,
							 output wire [14:0] outcode );

	parameter RESET_VECTOR  = 32'h0000_0000;
	parameter VALID_PC_WIDTH = 10;
	parameter NMI_VECTOR    = 32'h0000_1230;
    parameter ENABLE_NMI_VEC = 1'b1;
	parameter DEFAULT_MTVEC = 32'h0000_0040;
	parameter SHIFT_SELECTOR = 0;  // shift 
	parameter SHIFT_LATENCY = 0;
	parameter WRITE_CYCLE_OPT = 0;
	parameter MEMORY_TYPE = 0;
    parameter REGISTER_EXPRESSION = 0;

    /*function declaration   */
	function LUT_sel_or( input sel12 , sel1 , sel2a , sel2b );
		casex( {sel12 , sel1 , sel2a , sel2b} )
			4'b0_0_xx : LUT_sel_or = 1'b0 ;
			4'b0_1_xx : LUT_sel_or = 1'b1 ;

			4'b1_x_00 : LUT_sel_or = 1'b0 ;
			4'b1_x_01 : LUT_sel_or = 1'b1 ;
			4'b1_x_10 : LUT_sel_or = 1'b1 ;
			4'b1_x_11 : LUT_sel_or = 1'b1 ;
			
			  default : LUT_sel_or = 1'b0 ;
		endcase
	endfunction

	function LUT_and_or_xor_z( input [1:0]opa ,input in1 ,input in2 );
		case( { opa[1:0] , in1 , in2 } )
			/* XOR */
			4'b00_00 : LUT_and_or_xor_z = 1'd0 ;
			4'b00_01 : LUT_and_or_xor_z = 1'd1 ;
			4'b00_10 : LUT_and_or_xor_z = 1'd1 ;
			4'b00_11 : LUT_and_or_xor_z = 1'd0 ;
			/* OR */
			4'b10_00 : LUT_and_or_xor_z = 1'd0 ;
			4'b10_01 : LUT_and_or_xor_z = 1'd1 ;
			4'b10_10 : LUT_and_or_xor_z = 1'd1 ;
			4'b10_11 : LUT_and_or_xor_z = 1'd1 ;
			/* AND */
			4'b11_00 : LUT_and_or_xor_z = 1'd0 ;
			4'b11_01 : LUT_and_or_xor_z = 1'd0 ;
			4'b11_10 : LUT_and_or_xor_z = 1'd0 ;
			4'b11_11 : LUT_and_or_xor_z = 1'd1 ;
			/* others */
			 default : LUT_and_or_xor_z = 1'd0 ;
		endcase
	endfunction

	function LUT_csr_v( input [1:0] sf , input  chg , org );
		case( {sf , chg , org} )
			//  no change
			4'b00_00 : LUT_csr_v = 1'b0;
			4'b00_01 : LUT_csr_v = 1'b1;
			4'b00_10 : LUT_csr_v = 1'b0;
			4'b00_11 : LUT_csr_v = 1'b1;
			// over write
			4'b01_00 : LUT_csr_v = 1'b0;
			4'b01_01 : LUT_csr_v = 1'b0;
			4'b01_10 : LUT_csr_v = 1'b1;
			4'b01_11 : LUT_csr_v = 1'b1;
			// bit set
			4'b10_00 : LUT_csr_v = 1'b0;
			4'b10_01 : LUT_csr_v = 1'b1;
			4'b10_10 : LUT_csr_v = 1'b1;
			4'b10_11 : LUT_csr_v = 1'b1;
			// bit clear   
			4'b11_00 : LUT_csr_v = 1'b0;
			4'b11_01 : LUT_csr_v = 1'b1;
			4'b11_10 : LUT_csr_v = 1'b0;
			4'b11_11 : LUT_csr_v = 1'b0;
		endcase
	endfunction

	function LUT_cmp_su( input func , x1_31 , x2_31 , sub_31 ); // func=0 signed, func=1 unsigned
		casex( {func, x1_31, x2_31, sub_31} )
		// compair (signed)
		4'b0_00_0 : LUT_cmp_su = 1'b0; //sub_31;
		4'b0_11_0 : LUT_cmp_su = 1'b0; //sub_31; 
		4'b0_00_1 : LUT_cmp_su = 1'b1; //sub_31;
		4'b0_11_1 : LUT_cmp_su = 1'b1; //sub_31; 
		4'b0_01_x : LUT_cmp_su = 1'b0; //x1_31;
		4'b0_10_x : LUT_cmp_su = 1'b1; //x1_31;
		// compair (unsigned)
		4'b1_00_0 : LUT_cmp_su = 1'b0; //sub_31;
		4'b1_11_0 : LUT_cmp_su = 1'b0; //sub_31;
		4'b1_00_1 : LUT_cmp_su = 1'b1; //sub_31;
		4'b1_11_1 : LUT_cmp_su = 1'b1; //sub_31;
		4'b1_01_x : LUT_cmp_su = 1'b1; //x2_31;
		4'b1_10_x : LUT_cmp_su = 1'b0; //x2_31;
		endcase
	endfunction
    /* end of declaration */

	reg [31:0] inst;
	wire [4:0] rd,rs1,rs2;
	wire [4:0] code_sysb;
	wire [2:0] funct3;
	wire [6:0] op,funct7;

	wire CODE_LUI ,CODE_AUIPC,CODE_JAL  ,CODE_JALR   ,CODE_BRCH ,CODE_LOAD,CODE_STORE;
	wire CODE_ALUI,CODE_ALUR ,CODE_FENCE,CODE_FENCEI,CODE_ECALL,CODE_EBRK ,CODE_CSR,CODE_MRET;
	wire compliment,Load_sub,sft,sft_lr,as,f7i,ALU_sub,ALU_cmp,csr_imac;
	wire  code_sys;
	wire [1:0] BRCH_sub,ACC_Width,CSR_sub;

	wire [31:0] SI_imm, BJ_imm, U_imm;//, B_imm, J_imm;
	wire [4:0] csr_imm;
	wire [11:0] csr_addr;

	reg [VALID_PC_WIDTH-1:0] pc_calc,pc;
	reg [2:0]  stg;
	wire cndtn[3:0];
	wire BRANCH_F;
	wire exec_op;
	wire [VALID_PC_WIDTH-1:0] pc_calc_transfer,sel_p[0:2]; //,pc_add_sel[0:2]
	wire [1:0] sel_pc;//,pc_add_num;

	reg [4:0] q;
	reg sft_wait;
	reg [31:0] shiftr,ALU_add_sub,x1,x2,x1out,x2out,xd1_sel,xd2;
	wire [31:0] xd1,x1val,x2val,logic_op,ALU_out,Others_xd,s2_sel[0:2],xd2_sel[0:2],alu_sel[0:3];//,lms[0:3]
	wire [4:0] x2xd,x1xd;
	wire [5:0] reg1addr,reg2addr;
	wire [31:0] S2_ImmReg;//s2
	reg [31:0] greg[0:47];
	wire we_reg,we_csr,addr_en,stillshift,except_en,cmp_us,cmp_sg,reg1en,reg2en,fsft,Add_Sub_Sel;
	wire [1:0] xd2_sn,s2_num;
	wire [31:0] L_shift,sft_in,in_invLR,R_shift;

	wire e_DataAddrMiss,e_Inst,Jump_e,NMI_int_en,int_en;
	wire [4:0] Ecode;
	wire [31:0] csr_nn,csr_sel,csr_value,csr_mux[0:3];
	reg MIE_bit,MPIE_bit,MEIE_bit,NMIE_bit;
	wire [31:0] other_pc;
	reg [2:0] csr_num;
	reg [1:0] csr_no;
	
	always @(*) inst <= inst_data;
	assign { funct7  , rs2  , rs1 , funct3  , rd , op } = inst;

	assign code_sys = ({inst[31:30],inst[27:23],inst[19:7],op}==27'b0000000000_0000000000_1110011);
	assign code_sysb= {inst[29:28],inst[22:20]};

	assign BRCH_sub[1:0] = funct3[2:1];		//conditional jump
	assign compliment = funct3[0];		//branch compliment
	assign Awidth[1:0] = funct3[1:0];		//memory access  
	assign Load_sub = ~funct3[2];			//sign extensionsion
	assign ALU_sub = funct7[5];				//0:addition or、logical right shift 1:subtraction or arithmetic right shift 
	assign f7i = ( {funct7[6],funct7[4:0]} == 6'd0 );		//  
	assign sft = ( funct3[1:0] == 2'b01 );				//shift
	assign sft_lr = funct3[2];
	assign as = ( (funct3==3'b101) | (funct3==3'b000) );	//add-sub or right shift  
	assign ALU_cmp = ( funct3[2:1] == 2'b01 ); //compair
	assign CSR_sub[1:0] = funct3[1:0];			//CSR access way   
	assign csr_imac = funct3[2];			//0:register 1:immediate

	assign CODE_LUI   = (op==7'b0110111);
	assign CODE_AUIPC = (op==7'b0010111);
	assign CODE_JAL   = (op==7'b1101111);
	assign CODE_JALR  = (op==7'b1100111) & (funct3 == 3'b000);
	assign CODE_CSR   = (op==7'b1110011) & (funct3[1:0]!=2'b00);
	assign CODE_BRCH  = (op==7'b1100011) & (funct3[2:1]!=2'b01); 
	assign CODE_LOAD  = (op==7'b0000011) & (funct3[1:0]!=2'b11);
	assign CODE_STORE = (op==7'b0100011) & (funct3[1:0]!=2'b11) & (~funct3[2]) ;
	assign CODE_ALUI  = (op==7'b0010011) & ( ~sft | ( sft & f7i & ( funct3[2] | (~(funct3[2]|ALU_sub)) ) ) );
	assign CODE_ALUR  = (op==7'b0110011) & (  f7i & ( as | (~(as|ALU_sub)) ) );
	assign CODE_FENCE = (op==7'b0001111) & (funct3==3'b000);
	assign CODE_FENCEI= (op==7'b0001111) & (funct3==3'b001);
	assign CODE_ECALL = code_sys & (code_sysb==5'b00000);
	assign CODE_EBRK  = code_sys & (code_sysb==5'b00001);
	assign CODE_MRET  = code_sys & (code_sysb==5'b11010);

	assign outcode = { CODE_CSR, /*CODE_WFI,*/ CODE_MRET, CODE_EBRK, CODE_ECALL, CODE_FENCEI ,CODE_FENCE, CODE_ALUR, CODE_ALUI,
									CODE_STORE, CODE_LOAD, CODE_BRCH, CODE_JALR, CODE_JAL ,CODE_AUIPC, CODE_LUI} ; //----------------------- no need

	assign csr_addr[11:0] = inst[31:20];
	assign csr_imm = inst[19:15];
	assign U_imm = { inst[31:12] , 12'h000 };
	assign SI_imm = { { 21{inst[31]} } , { inst[30:25] } , {(CODE_STORE|CODE_BRCH)?inst[11:7]:inst[24:20]} };
	//assign BJ_imm = { SI_imm[31:20] , (CODE_BRCH)? SI_imm[19:12]:U_imm[19:12] , SI_imm[0] , SI_imm[10:1] , 1'b0 };
	assign BJ_imm = { {12{inst[31]}} , {(CODE_BRCH)?{{8{inst[31]}},{inst[7]}}:{{inst[19:12]},{inst[20]}}} , {inst[30:25]} , {(CODE_BRCH)?{inst[11:8]}:{inst[24:21]}} ,{1'b0} };

	assign NMI_int_en = NMIE_bit & NMI_S;
	assign int_en = MIE_bit & MEIE_bit & INT_S;
	assign addr_en = (stg>'d1)&(CODE_LOAD|CODE_STORE);
	assign e_DataAddrMiss = addr_en&( (Awidth[0]&PADDR[0]) | (Awidth[1]&(PADDR[0]|PADDR[1])) );
	assign e_Inst = (~(|outcode)) & (stg!='d0) ;
 	assign except_en = e_DataAddrMiss | e_Inst | CODE_EBRK | CODE_ECALL;
	assign Jump_e = NMI_int_en|except_en|int_en;

	assign Ecode = ((~reset)|NMI_int_en)? 5'd31 : (int_en|CODE_ECALL)? 5'd11 : (CODE_EBRK)? 5'd3:
							  (e_Inst)? 5'd2 : (e_DataAddrMiss)? ((CODE_LOAD)? 5'd4 : 5'd6) : 5'd0  ;

	assign S2_ImmReg = ((stg<'d5)&(CODE_ALUI|CODE_JALR|CODE_LOAD|CODE_STORE)) ? SI_imm : x2;
	//assign S2_ImmReg = (stg>'d4)? x2 : (CODE_ALUI|CODE_JALR|CODE_LOAD|CODE_STORE) ? SI_imm : x2;

	assign csr_sel = (csr_imac) ?  { 27'd0 , csr_imm } : x1;
	assign Add_Sub_Sel =  (CODE_ALUR&ALU_sub) | ((CODE_ALUR|CODE_ALUI)&ALU_cmp) | CODE_BRCH ;
	always @(*) ALU_add_sub <= ( ~Add_Sub_Sel ) ? x1+S2_ImmReg : x1-S2_ImmReg  ;
	assign cmp_us = (x1[31]==S2_ImmReg[31])? ALU_add_sub[31] : S2_ImmReg[31];  //x1<s2;  
	assign cmp_sg = (x1[31]==S2_ImmReg[31])? ALU_add_sub[31] : x1[31];         //$signed(x1)<$signed(s2);  

	//memory access circuit
	assign PADDR = ALU_add_sub;
	assign s_ext = Load_sub;
	assign PWDATA = x2;
	assign PWRITE  = (stg>'d1)&(CODE_STORE); //
	assign PSEL = addr_en;
	generate
		if( WRITE_CYCLE_OPT == 1 ) begin
			if( (MEMORY_TYPE == 1)||(MEMORY_TYPE == 2)||(MEMORY_TYPE == 3) )
				assign PENABLE = ((CODE_LOAD)?(stg=='d3):(stg=='d2)) & addr_en & (~Jump_e) ;
			else
				assign PENABLE = ( (Awidth=='d0|CODE_LOAD)?(stg=='d3):(stg=='d2) ) & addr_en & (~Jump_e);
		end
		else
			assign PENABLE = (stg=='d3) & addr_en &(~Jump_e);
	endgenerate

    //  latency or start trigger of sequential shifter
	generate
		if( (SHIFT_LATENCY == 0) && ((SHIFT_SELECTOR == 1)||(SHIFT_SELECTOR == 2)||(SHIFT_SELECTOR == 3)) ) begin
			always @(posedge clk) begin
				sft_wait <= 1'b0;
			end
		end
		else begin
			always @( posedge clk or negedge reset ) begin
				if( ~reset ) sft_wait <= 1'b0;
				else sft_wait <= (stg=='d1) & sft & (CODE_ALUR|CODE_ALUI);
			end
		end
	endgenerate

	 //-----------------  shift --------------------------
	 generate 
		if( SHIFT_SELECTOR == 1 ) begin // multiply shifter
			wire [31:0] s_mul,s_mf;
			wire s_msb = ALU_sub&x1[31];
			genvar ssi;
			for( ssi = 0 ; ssi <= 31 ; ssi = ssi + 1 )
			begin : gen_inv_var_mul
				assign in_invLR[31-ssi] = x1[ssi];
            assign R_shift[31-ssi] = (sft) ? ( (s_mf[ssi])? L_shift[ssi] : s_msb ) : 1'b0;
			end
			assign stillshift = 1'b0;
			assign fsft = sft_wait;
			assign sft_in = (sft_lr) ? in_invLR : x1;
			assign s_mul = 1 << S2_ImmReg[4:0];
			assign s_mf = 32'hffff_ffff * s_mul;
			assign L_shift = sft_in * s_mul;
		end   // end of multiply shifter

		else if( SHIFT_SELECTOR == 2 ) begin  // harf barrel shifter
			wire signed[31:0] AShift;
			wire [31:0] tmp_shift;
			wire s_msb = ALU_sub&x1[31];
			genvar ssk;
			for( ssk = 0 ; ssk <= 31 ; ssk = ssk + 1 ) 
			begin : gen_inv_var_hs
				assign in_invLR[31-ssk] = x1[ssk];
				assign R_shift[ssk] = LUT_sel_or( sft , 1'b0 , tmp_shift[ssk] , AShift[ssk] );
				assign L_shift[31-ssk] = R_shift[ssk];
			end
			assign fsft = sft_wait;
			assign stillshift = 1'b0;
			assign sft_in = (sft_lr) ? x1 : in_invLR;
			assign tmp_shift = sft_in >> S2_ImmReg[4:0];
			assign AShift = $signed({s_msb,31'd0}) >>> S2_ImmReg[4:0];
			//assign AShift = $signed(sft_in) >>> S2_ImmReg[4:0];
			//assign R_shift = (sft) ? ( AShift|tmp_shift ) : 32'd0;
		end  // end of harf barrel shifter

		else if( SHIFT_SELECTOR == 3 ) begin  // full barrel shifter
			wire signed[31:0] AShift;
			wire [31:0] tmp_shift;
			assign fsft = sft_wait;
			assign stillshift = 1'b0;
			assign L_shift = x1 << S2_ImmReg[4:0];
			assign tmp_shift = x1 >> S2_ImmReg[4:0];
			assign AShift = $signed(x1) >>> S2_ImmReg[4:0];
			assign R_shift = (sft) ? ( (ALU_sub)? AShift : tmp_shift ) : 32'd0;
		end // end of full barrel shifter

		else begin  // sequential shifter
			wire s_msb = ALU_sub & x1[31];
			assign stillshift = (q>'d0);
			assign fsft = sft_wait;
			assign R_shift = shiftr;
			assign L_shift = shiftr;
			always @( posedge clk or negedge sft or negedge reset) begin
				if( ~sft|~reset ) begin
					shiftr <= 32'd0;
					q <= 5'd0;
				end
				else begin
					if( fsft ) begin
						shiftr <= x1;
						q <= S2_ImmReg[4:0];
					end
					if( stillshift ) begin
						shiftr <= (sft_lr)? { s_msb , shiftr[31:1] } : { shiftr[30:0] , 1'b0 } ;
						q <= q - 5'd1;
					end
				end
			end
		end // end of sequential shifter
	endgenerate

	//-----------------  end shift --------------------------

	assign alu_sel[0] = ALU_add_sub;
	assign alu_sel[1] = L_shift;
	assign alu_sel[2] = {31'd0,cmp_sg};
	assign alu_sel[3] = {31'd0,cmp_us};

	assign other_pc = { RESET_VECTOR[31 -: (32-VALID_PC_WIDTH-2)]  , pc , 2'd0};
	assign Others_xd = ( (stg=='d5)? {NMI_S|INT_S,26'd0,Ecode} : ((stg<'d5)&CODE_LUI)? 32'd0 : other_pc ) 
                                       + ( (stg>='d5) ? 32'd0 : (CODE_AUIPC|CODE_LUI) ? U_imm : 32'd4 );

	generate
		genvar i;
		for( i = 0 ; i <= 31 ; i = i + 1 )
		begin : gen_LUT4exec
			assign logic_op[i] = LUT_and_or_xor_z( funct3[1:0] , x1[i] , S2_ImmReg[i] );
			assign ALU_out[i]  = LUT_sel_or( funct3[2] , alu_sel[ funct3[1:0] ][i] , R_shift[i] , logic_op[i] );
			assign csr_nn[i]   = LUT_csr_v( CSR_sub , csr_sel[i] , csr_value[i] );
		end
	endgenerate

	assign we_reg = (~Jump_e)
	              & ( ( (stg=='d1)&(CODE_LUI|CODE_AUIPC|CODE_JAL) )
                 | ( (stg=='d2)&( ((~stillshift)&(~fsft)&(CODE_ALUR|CODE_ALUI)) | CODE_JALR ) )
                 | ( PENABLE&CODE_LOAD) ) ;

	assign we_csr =  (~Jump_e)&(stg=='d2)&CODE_CSR;

    parameter MTVAL_CHOICE = 9'o012 ; 
	wire [31:0] mtval_val [0:2];
	wire [2:0]  mtval_cndt;
	assign mtval_cndt = { (stg<'d5)&CODE_CSR , e_DataAddrMiss , e_Inst };
	assign mtval_val[2] = csr_value;
	assign mtval_val[1] = PADDR;
	assign mtval_val[0] = inst;
	always @(*) begin
        casex( {mtval_cndt[MTVAL_CHOICE[8:6]] ,  mtval_cndt[MTVAL_CHOICE[5:3]]} )
			2'b1x: xd1_sel <= mtval_val[ MTVAL_CHOICE[8:6] ];
			2'b01: xd1_sel <= mtval_val[ MTVAL_CHOICE[5:3] ];
			default: xd1_sel <= mtval_val[ MTVAL_CHOICE[2:0] ];
		endcase
	end
	assign xd1 = xd1_sel;
	 
   parameter LP_XD_CHOICE = 9'o012 ;
	localparam LP_XDC0 = LP_XD_CHOICE[2:0];
	localparam LP_XDC1 = LP_XD_CHOICE[5:3];
	localparam LP_XDC2 = LP_XD_CHOICE[8:6];
	wire [2:0] xd2_cond;
	assign xd2_cond = { (stg<'d5)&(CODE_ALUR|CODE_ALUI), ((stg<'d5)&(CODE_LOAD)), ((stg<'d5)&(CODE_CSR)) };
	assign xd2_sel[2] = ALU_out;
	assign xd2_sel[1] = PRDATA;
	assign xd2_sel[0] = csr_nn;
	always @(*) begin
		casex( {xd2_cond[LP_XDC2],xd2_cond[LP_XDC1],xd2_cond[LP_XDC0]} )
			3'b1xx : xd2 <= xd2_sel[LP_XDC2];
			3'b01x : xd2 <= xd2_sel[LP_XDC1];
			3'b001 : xd2 <= xd2_sel[LP_XDC0];
			default: xd2 <= Others_xd;
		endcase
	end

   parameter LP_CN_CHOICE = 18'o012345 ;
    localparam LP_CN0 = LP_CN_CHOICE[17:15] ; // mtvec    32
    localparam LP_CN1 = LP_CN_CHOICE[14:12] ; // mepc     33
    localparam LP_CN2 = LP_CN_CHOICE[11:9] ; // mtval  35
    localparam LP_CN3 = LP_CN_CHOICE[8:6] ; // mcause  37
    localparam LP_CN4 = LP_CN_CHOICE[5:3] ; // misa    34
    localparam LP_CN5 = LP_CN_CHOICE[2:0] ; // others, undefined  36

	assign x1xd = ( we_csr ) ? rd : rs1;
	assign reg1en = (stg=='d5) | (we_csr&(x1xd>'d0));
	assign reg1addr = (stg=='d5)? {3'b100,LP_CN2} : (stg>='d6)? ((NMI_int_en&ENABLE_NMI_VEC)? {3'b101,LP_CN0}:{3'b100,LP_CN0}) : {1'b0,x1xd};
	assign x1val= xd1;

    parameter LP_X2_CHO = 12'o0213;  //o0132 26.976/o0213 27.025/o1023 error
    localparam LP_X20 = LP_X2_CHO[2:0];
    localparam LP_X21 = LP_X2_CHO[5:3];
    localparam LP_X22 = LP_X2_CHO[8:6];
    localparam LP_X23 = LP_X2_CHO[11:9];
    wire [3:0] x_2_cnd;
    wire [5:0] x_2_val[0:3];
    assign x_2_cnd[0] = ((stg<'d5)&CODE_CSR)              , x_2_val[0] = {3'b100,csr_num[2:0]};
    assign x_2_cnd[1] = ((stg=='d7)|((stg<'d5)&CODE_MRET)), x_2_val[1] = {3'b100,LP_CN1};
    assign x_2_cnd[2] = (stg=='d6)                        , x_2_val[2] = 0;
    assign x_2_cnd[3] = (stg=='d5)                        , x_2_val[3] = {3'b100,LP_CN3};
	assign x2xd = ( we_reg ) ? rd : rs2;
	assign reg2en = (stg=='d5) | (stg=='d7) | (we_csr&(csr_num<='d1)) | (we_reg&(x2xd>'d0));
	assign reg2addr =   (x_2_cnd[LP_X20])? x_2_val[LP_X20]:
                        (x_2_cnd[LP_X21])? x_2_val[LP_X21]:
                        (x_2_cnd[LP_X22])? x_2_val[LP_X22]:
                        (x_2_cnd[LP_X23])? x_2_val[LP_X23]:
						{1'b0,x2xd} ;  //
	assign x2val= xd2;

	always @( * ) x2 <= x2out;
	always @( * ) x1 <= x1out ;

    generate
        if( REGISTER_EXPRESSION == 1 ) begin
            always @(posedge clk) begin
                x1out <= greg[reg1addr];
                if(reg1en) greg[reg1addr] <= x1val;
            end
            always @(posedge clk) begin
                x2out <= greg[reg2addr];
                if(reg2en) greg[reg2addr] <= x2val;
            end
        end
        else begin
            always @(posedge clk) begin
                if(~reg1en) x1out <= greg[reg1addr];
                if(reg1en) greg[reg1addr] <= x1val;
            end
            always @(posedge clk) begin
                if(~reg2en) x2out <= greg[reg2addr];
                if(reg2en) greg[reg2addr] <= x2val;
            end
        end
    endgenerate

    // h20(d32) : mtvec , h21(d33) : mepc , h22(d34) : mtval , h23(d35) : mcause , h24(d36) : misa
    integer k;
    initial begin
        for( k=0 ; k<48 ; k=k+1 )
                 if( k == {3'b100,LP_CN0} ) greg[k] = DEFAULT_MTVEC;
            else if( k == {3'b100,LP_CN1} ) greg[k] = 'd20;
            else if( k == {3'b100,LP_CN4} ) greg[k] = { 2'b01 , 4'b0000 , 26'b00_0000_0000_0000_0001_0000_0000 };  // misa	ISA and extensions;;
            else if( k == {3'b101,LP_CN0} ) greg[k] = NMI_VECTOR;
            else                            greg[k] = 'd0;
    end
	
	//program counter circuit
	assign inst_addr = { RESET_VECTOR[31 -: (32-VALID_PC_WIDTH-2)] , (exec_op)? pc_calc : pc , 2'd0};
	
    wire [1:0]BRCH_cdn;
    assign BRCH_cdn[0] = ~(|ALU_add_sub); // x1 == x2;
    assign BRCH_cdn[1] = LUT_cmp_su( BRCH_sub[0], x1[31], S2_ImmReg[31], ALU_add_sub[31] );

	assign cndtn[2'b00] = ~(|ALU_add_sub); // x1 == x2;  
	assign cndtn[2'b01] = cndtn[2'b00];//1'bx;//
	assign cndtn[2'b10] = cmp_sg;
	assign cndtn[2'b11] = cmp_us;

	assign BRANCH_F = (cndtn[BRCH_sub] == compliment)&CODE_BRCH ;
//	assign pc_calc_transfer = pc + ( ( (CODE_JAL|CODE_BRCH))? BJ_imm[2+:VALID_PC_WIDTH] : {{(VALID_PC_WIDTH-1){1'b0}},{1'b1}} );

	wire BRANCH_T = (cndtn[BRCH_sub] != compliment)&CODE_BRCH ;
//	wire BRANCH_T = (BRCH_cdn[BRCH_sub[1]] != compliment)&CODE_BRCH ;
	assign pc_calc_transfer = pc + ( ( (CODE_JAL|BRANCH_T))? BJ_imm[2+:VALID_PC_WIDTH] : {{(VALID_PC_WIDTH-1){1'b0}},{1'b1}} );

	parameter LP_PC_SEL_CHOICE = 6'o10 ;
	localparam LP_PC_SEL0 = LP_PC_SEL_CHOICE[2:0];
	localparam LP_PC_SEL1 = LP_PC_SEL_CHOICE[5:3];
	wire [1:0] pc_sel_cond;
	assign pc_sel_cond = { CODE_MRET|CODE_JALR|(stg>='d6) , BRANCH_F };
	assign sel_p[1] = ALU_add_sub[ VALID_PC_WIDTH+1 : 2 ];
	assign sel_p[0] = Others_xd[ VALID_PC_WIDTH+1 : 2 ];
	
/*	always @( * ) begin
		casex( { pc_sel_cond[LP_PC_SEL1],pc_sel_cond[LP_PC_SEL0] } )
			2'b1x : pc_calc = sel_p[LP_PC_SEL1];
			2'b01 : pc_calc = sel_p[LP_PC_SEL0];
			default: pc_calc = pc_calc_transfer;
		endcase
	end*/
	
	always @( * ) begin
		casex(  pc_sel_cond[1] )
			1'b1 : pc_calc = sel_p[1];
			default : pc_calc = pc_calc_transfer;
		endcase
	end

	assign exec_op = ( (~Jump_e)&( we_csr|we_reg|PENABLE|((stg == 'd1)&(CODE_FENCE|CODE_FENCEI))
										  |((stg == 'd2)&(CODE_BRCH|CODE_MRET)) ) ) | (stg=='d7);

	always @( posedge clk or negedge reset ) begin
		if( ~reset )
			pc <= RESET_VECTOR[ 2 +: VALID_PC_WIDTH ]; // bit2 -> bit17
		else
		begin
			//if (stg == 'd7) pc <= x1[ VALID_PC_WIDTH+1 : 2 ];
			//else if ( exec_op ) pc <= pc_calc;
			if ( exec_op ) pc <= pc_calc;
		end
	end

	//cpu stage circuit
	always @( posedge clk or negedge reset ) begin
		if( ~reset )
			stg <= 'd0;
		else begin
			if( exec_op) stg <= 'd1;
			else stg <= ( ((stg<'d5)&Jump_e )? 3'd4 : stg ) + ( ( (~Jump_e)&(stillshift|fsft) ) ? 3'd0 : 3'd1 );
		end
	end

	/*CSR*/
    parameter LP_CSR_CHOICE = 12'o0123 ;
    localparam LP_CSR0 = LP_CSR_CHOICE[11:9];
    localparam LP_CSR1 = LP_CSR_CHOICE[8:6];
    localparam LP_CSR2 = LP_CSR_CHOICE[5:3];
    localparam LP_CSR3 = LP_CSR_CHOICE[2:0];

	assign csr_mux[LP_CSR0] = { 19'd0,2'b11,3'd0,MPIE_bit,3'd0,MIE_bit,3'd0 }; // mstatus		Machine status register.;
	assign csr_mux[LP_CSR1] = { 20'd0 , MEIE_bit , 11'd0 }; // mie			Machine interrupt-enable register.;
	assign csr_mux[LP_CSR2] = { 20'd0 ,  INT_S   , 11'd0 }; // mip Machine interrupt pending.
	assign csr_mux[LP_CSR3] = x2;
	assign csr_value = csr_mux[csr_no];

	always @(*) begin
		if( csr_addr[11:8]=='d3 ) begin
			case( csr_addr[7:0] )
				'h00 : csr_no = LP_CSR0; //mstatus
				'h04 : csr_no = LP_CSR1; //mie
				'h44 : csr_no = LP_CSR2; //mip
				default : csr_no = LP_CSR3;
			endcase
		end
		else
			csr_no = LP_CSR3;
	end

	always @(*) begin
		if( csr_addr[11:8]=='d3 ) begin
			case( csr_addr[7:0] )
				'h01 : csr_num = LP_CN4; //misa
				'h05 : csr_num = LP_CN0; //mtvec
				'h41 : csr_num = LP_CN1; //mepc
				'h42 : csr_num = LP_CN3; //mcause
				'h43 : csr_num = LP_CN2; //mtval
				default : csr_num = LP_CN5;
			endcase
		end
		else
			csr_num = LP_CN5;
	end

	always @(posedge clk or negedge reset) begin
		if( ~reset ) begin
			{ MIE_bit,MPIE_bit,MEIE_bit }  <= 0;
			NMIE_bit <= 1'b1;
		end
		else begin
			if( stg == 'd7 ) begin
				if(NMI_int_en) NMIE_bit <= 1'b0;
				if(Jump_e) { MPIE_bit, MIE_bit } <= { MIE_bit , 1'b0 };
			end
			if( (stg == 'd2)&CODE_MRET ) begin
				{ NMIE_bit,MPIE_bit } <= { 1'b1,1'b1 } ;
				MIE_bit <= MPIE_bit;
			end
			if( we_csr&(csr_no==LP_CSR0) ) { MPIE_bit , MIE_bit } <= { csr_nn[7], csr_nn[3] }; // mstatus		Machine status register.
			if( we_csr&(csr_no==LP_CSR1) ) MEIE_bit <= csr_nn[11]; // mie		Machine interrupt-enable register.
		end
	end

endmodule

/* main memory */
module EXT_RAM( input wire [31:0] d22, addr1 , addr2 , 
                input wire clk, we2, reset , 
                input wire [3:0] wstrb,
				    output reg [31:0] q1 , q2 );

	parameter INST_ADDR_WIDTH = 10;
	parameter RAM_SIZE = 4;
	parameter MEMORY_TYPE = 0;
	parameter LOAD_HEX_FILE = "test.hex" ;
	localparam MEMORY_CAPA = ((RAM_SIZE*1024/4) - 1);
	
	reg [31:0] Load_buffer[0:MEMORY_CAPA];
	reg [7:0] mem0[0:MEMORY_CAPA];
	reg [7:0] mem1[0:MEMORY_CAPA];
	reg [7:0] mem2[0:MEMORY_CAPA];
	reg [7:0] mem3[0:MEMORY_CAPA];
	reg [31:0] memA[0:MEMORY_CAPA];
	wire [INST_ADDR_WIDTH-1:0] ad1,ad2;

	assign ad1 = addr1[2 +: INST_ADDR_WIDTH];
	assign ad2 = addr2[2 +: INST_ADDR_WIDTH];

	integer byte_no;
	genvar byte_no2;
	generate 
		if( MEMORY_TYPE == 1 ) begin // Quad Block Memory
			always @(posedge clk) begin
				q1 <= { mem3[ad1], mem2[ad1], mem1[ad1], mem0[ad1] };
				if( ~(we2&wstrb[0]) ) q2[ 7: 0] <= mem0[ ad2 ];
				if( (we2&wstrb[0]) ) mem0[ ad2 ] <= d22[ 7: 0];
				if( ~(we2&wstrb[1]) ) q2[15: 8] <= mem1[ ad2 ];
				if( (we2&wstrb[1]) ) mem1[ ad2 ] <= d22[15: 8];
				if( ~(we2&wstrb[2]) ) q2[23:16] <= mem2[ ad2 ];
				if( (we2&wstrb[2]) ) mem2[ ad2 ] <= d22[23:16];
				if( ~(we2&wstrb[3]) ) q2[31:24] <= mem3[ ad2 ];
				if( (we2&wstrb[3]) ) mem3[ ad2 ] <= d22[31:24];
			end

		end else if( MEMORY_TYPE == 3 ) begin //Quad divided meory for Quartus
			always @(posedge clk) begin
				q1 <= { mem3[ad1], mem2[ad1], mem1[ad1], mem0[ad1] };
			end
			always @(posedge clk) begin
				q2 <= { mem3[ad2], mem2[ad2], mem1[ad2], mem0[ad2] };
				if( (we2&wstrb[0]) ) mem0[ ad2 ] <= d22[ 7: 0];
				if( (we2&wstrb[1]) ) mem1[ ad2 ] <= d22[15: 8];
				if( (we2&wstrb[2]) ) mem2[ ad2 ] <= d22[23:16];
				if( (we2&wstrb[3]) ) mem3[ ad2 ] <= d22[31:24];
			end
		
		end else if( MEMORY_TYPE == 2 ) begin //Quad divided meory using Verilog expression
			always @(posedge clk) begin
				q1 <= memA[ ad1 ];
				for( byte_no = 0 ; byte_no < 4 ; byte_no = byte_no + 1 ) begin
					if( ~(we2&wstrb[byte_no]) ) q2[ 8*byte_no +: 8 ] <= memA[ ad2 ][8*byte_no +: 8];
					if( we2&wstrb[byte_no] ) memA[ ad2 ][ 8*byte_no +: 8 ] <= d22[ 8*byte_no +: 8 ];
				end
			end

		end else begin // any verilog can compile
			wire [7:0] din2[0:3];
			wire [1:0] wea;
			assign wea[0] = we2&(wstrb[1] | wstrb[0]);
			assign wea[1] = we2&(wstrb[3] | wstrb[2]);

			for( byte_no2 = 0 ; byte_no2 < 4 ; byte_no2 = byte_no2 + 1 ) begin : mem_choice
				assign din2[byte_no2] =   (wstrb[byte_no2]) ? d22[8*byte_no2 +: 8] : q2[8*byte_no2 +: 8];
			end
			
			always @(posedge clk) begin
				q1 <= memA[ ad1 ];
				if( ~wea[0] ) q2[ 0 +: 16] <= memA[ ad2 ][ 0 +: 16];
				if( ~wea[1] ) q2[16 +: 16] <= memA[ ad2 ][16 +: 16];
				if( wea[0] ) memA[ ad2 ][ 0 +: 16] <= { din2[1] , din2[0] };
				if( wea[1] ) memA[ ad2 ][16 +: 16] <= { din2[3] , din2[2] };
			end
		end
	endgenerate

	initial begin  //memory initialize
		$readmemh( LOAD_HEX_FILE , Load_buffer ); //program read from hex file 
		if( (MEMORY_TYPE == 1) || (MEMORY_TYPE == 3) ) begin
			for( byte_no = 0 ; byte_no <= MEMORY_CAPA ; byte_no = byte_no + 1 )
				{ mem3[byte_no], mem2[byte_no], mem1[byte_no], mem0[byte_no] } = Load_buffer[byte_no];
		end
		else begin
			for( byte_no = 0 ; byte_no <= MEMORY_CAPA ; byte_no = byte_no + 1 )
				memA[byte_no]  = Load_buffer[byte_no];
		end
	end //memory initial end 
endmodule 

/* unified peripheral */
module Super_IO ( input wire clk,reset ,  input wire pe , rdwr , cs , input wire [31:0] addr,
						input wire [31:0] indata , output reg [31:0] outdata , 
						output wire timer_out , timer_int ,
						input wire sw1,
						output wire [2:0] RGB_LED,
						output wire [21:0] LCD,
						output wire tx_serial_out,
						input  wire rx_serial_in
						);
parameter WIDTH = 16;
parameter PRIOD_WIDTH = 15;
// tang nano 1k = 27MHz

    function [1:0] func_LCD_choice( input [2:0] in_l );
        case( in_l )
            3'b011 : func_LCD_choice = 'd0 ;
            3'b101 : func_LCD_choice = 'd1 ;
            3'b110 : func_LCD_choice = 'd2 ;
            default : func_LCD_choice = 'd3;
        endcase
    endfunction

	reg [WIDTH-1:0] tmr_reg;
	reg [WIDTH-1:0] rld_reg;
	reg tmr_en,tmr_IE,tmr_int,tmr_tgl;
    wire [3:0] addr_in = addr[5:2];
	wire tmr_int_rst = pe & cs & rdwr & (addr_in=='d2)&indata[2];
	wire tmr_reload = (tmr_reg == { WIDTH{1'b0} });

    reg LCD_tgl;
	reg [6:0] LCD_bit[0:5];
	reg [2:0] LCD_com;
	reg [PRIOD_WIDTH-1:0] LCD_tgl_period;
    wire [1:0] LCD_choice;
    wire [17:0] LCD_out_pattern[0:2];

	reg [2:0] LED_out;

    wire [7:0] rx_data_path;
    wire tx_ctrl,we_tx,rx_ctrl,rd_rx;

    assign we_tx = pe & cs & rdwr & (addr_in == 'd8);
    assign rd_rx = pe & cs & ~rdwr & (addr_in == 'd10);
    UART_TX_B uart1_tx( .clk(clk) , .rst(reset) , .we(we_tx) , .tx_data(indata[7:0]) , .tx_serial(tx_serial_out) , .tx_empty(tx_ctrl)   );
    UART_RX_B uart1_rx( .clk(clk) , .rst(reset) , .rd(rd_rx) , .buf_rx(rx_data_path) , .rx_serial(rx_serial_in)  , .valid_data(rx_ctrl) );

	/* LCD driver */
	always @(posedge clk or negedge reset ) begin
		if( ~reset ) begin
			LCD_tgl_period <= 'd1; //{(PRIOD_WIDTH){1'b0}};
            LCD_com <= 3'b110;
            LCD_tgl <= 'b0;
		end
		else begin
			LCD_tgl_period <= LCD_tgl_period + { {(PRIOD_WIDTH-1){1'b0}}, {1'b1} };
            if( LCD_tgl_period == 'd0 )
            begin
                LCD_com[2:0] <= { LCD_com[1] , LCD_com[0] , LCD_com[2] };
                LCD_tgl <= ~LCD_tgl;
            end
		end
	end

    assign LCD_out_pattern[0] = {       LCD_bit[5][2:0],           LCD_bit[4][2:0],           LCD_bit[3][2:0],           LCD_bit[2][2:0],           LCD_bit[1][2:0],           LCD_bit[0][2:0] };
    assign LCD_out_pattern[1] = {       LCD_bit[5][5:3],           LCD_bit[4][5:3],           LCD_bit[3][5:3],           LCD_bit[2][5:3],           LCD_bit[1][5:3],           LCD_bit[0][5:3] };
    assign LCD_out_pattern[2] = { 1'b0, LCD_bit[5][6], 1'b0, 1'b0, LCD_bit[4][6], 1'b0, 1'b0, LCD_bit[3][6], 1'b0, 1'b0, LCD_bit[2][6], 1'b0, 1'b0, LCD_bit[1][6], 1'b0, 1'b0, LCD_bit[0][6], 1'b0 };

    assign LCD_choice = func_LCD_choice(LCD_com);
	assign LCD = {  LCD_out_pattern[LCD_choice][17:0]^{18{LCD_tgl}} , LCD_com[2:0] , LCD_tgl };

	assign RGB_LED = LED_out;// LED output

	/* peripheral's register */
	always @(*) begin
		case( addr_in )
			'd0 : outdata <= { {(32-WIDTH){1'bx}} , rld_reg };  // timer reload register
			'd1 : outdata <= { {(32-WIDTH){1'bx}} , tmr_reg };  //　timer count register
			'd2 : outdata <= { {(32-2){1'bx}} , {  tmr_IE , tmr_en  } }; // timer control register (timer interrupt enable bit, timer start bit)
			'd3 : outdata <= { {(32-2){1'bx}} , { tmr_tgl , tmr_int } }; // timer status register

			'd4 : outdata <= { {(32-3){1'bx}} , { LED_out } };   // GPIO output (RGB LED)
			'd5 : outdata <= { {(32-1){1'bx}} , { sw1 } }; // GPIO input  (switch input)
			'd6 : outdata <= { 8'd0, 1'b0, LCD_bit[2], 1'b0, LCD_bit[1], 1'b0, LCD_bit[0] };  // LCD pattern bit1
			'd7 : outdata <= { 8'd0, 1'b0, LCD_bit[5], 1'b0, LCD_bit[4], 1'b0, LCD_bit[3] };  // LCD pattern bit1

            'd9 : outdata <= { 31'dx , tx_ctrl };     // serial tx status register
            'd10: outdata <= { 24'dx , rx_data_path };// serial reciev data
            'd11: outdata <= { 31'dx , rx_ctrl };     // serial rx status register 
			default : outdata <= 32'dx;
		endcase
	end
	
	always @( posedge clk or negedge reset ) begin
		if( ~reset ) begin
			rld_reg <= { WIDTH{1'b1} };
			tmr_en <= 'b0;
			tmr_IE <= 'b0;
			{ LCD_bit[5], LCD_bit[4], LCD_bit[3], LCD_bit[2], LCD_bit[1], LCD_bit[0] } <= {7'h7f,7'h7f,7'h7f,7'h7f,7'h7f,7'h7f};
			LED_out <= 3'b111;
		end
		else begin
			if( pe & cs & rdwr ) begin
				case ( addr_in )
					'd0 : rld_reg <= indata[WIDTH-1:0];
					'd2 : {tmr_IE , tmr_en} <= indata[1:0];
					'd4 : LED_out <= indata[2:0];
					'd6 : { LCD_bit[2], LCD_bit[1], LCD_bit[0] } <= { indata[22:16],indata[14:8],indata[6:0] };
					'd7 : { LCD_bit[5], LCD_bit[4], LCD_bit[3] } <= { indata[22:16],indata[14:8],indata[6:0] };
				endcase
			end
		end
	end

	/* timer circuit */
	assign timer_out = tmr_tgl;
	assign timer_int = tmr_int;

	always @( posedge clk or negedge reset ) begin
		if( ~reset ) begin
			tmr_reg <= { WIDTH{1'b1} };
		end
		else begin
			if( ~tmr_en | tmr_reload  ) 
				tmr_reg <= rld_reg ;
			else
				tmr_reg <= tmr_reg - { { (WIDTH-1){1'b0} } , {1'b01} } ;
		end
	end

	always @( posedge clk or negedge reset ) begin
		if( ~reset ) begin
			tmr_tgl <= 1'b0;
		end
		else begin
			if( tmr_reload ) tmr_tgl <= ~tmr_tgl;
		end
	end

	always @( posedge clk or negedge reset ) begin
		if( ~reset ) begin
			tmr_int <= 1'b0;
		end
		else if( tmr_IE & tmr_reload ) begin 
			tmr_int <= 1'b1;
		end
		else if(tmr_int_rst) begin
			tmr_int <= 1'b0;
		end 
	end
    /* end of timer circuit */

endmodule

/* data transfer control amang cpu,memory,peripherals */
module bus_master( input wire [31:0] dataFp1,dataFp2 ,addrbus, data_from_cpu ,
						output wire [31:0] data2cpu , data2Peri ,
                   input wire [1:0] AWidth , input wire s_ext ,
						 input wire cs, wr , output wire cs1,cs2 ,
                         output wire [3:0] byte_write );
    // addr : 32'h0000_0000 - 32'h0000_0fff as RAM
    // addr : 32'h0001_0000 - 32'h0001_003f as Super_IO
	parameter RESET_VECTOR  = 32'h0000_0000;
	parameter INST_ADDR_WIDTH = 10;
	parameter RAM_SIZE = 4;
	wire chk1 = (addrbus[31:INST_ADDR_WIDTH+2] == RESET_VECTOR[31:INST_ADDR_WIDTH+2]) 
												& ( addrbus[INST_ADDR_WIDTH+1:2] <= ((RAM_SIZE*1024/4)-1) ) ; // main memory
	wire chk2 = (addrbus[31:6]  == 26'h000_0400); // unified peripheral
	assign cs1 = cs & chk1;
	assign cs2 = cs & chk2;

   wire [7:0] qq[0:3];
   wire [31:0] lms[0:3];
	reg  [31:0] data_choice;
   assign { qq[3] , qq[2] , qq[1] , qq[0] } = data_choice;
   assign lms[0] = { {24{s_ext&lms[0][ 7]}} , { qq[addrbus[1:0]]} };
   assign lms[1] = { {16{s_ext&lms[1][15]}} , { qq[{addrbus[1],1'b1}] , qq[{addrbus[1],1'b0}] } };
   assign lms[2] = { qq[3] , qq[2] , qq[1] , qq[0] };
   assign lms[3] = { qq[3] , qq[2] , qq[1] , qq[0] };
   assign data2cpu = lms[AWidth];//data_choice;
	always @(*) begin
		case( { chk2,chk1 } )
			2'b01 : data_choice <= dataFp1; //memory
			2'b10 : data_choice <= dataFp2; // peripheral
			default : data_choice <= 32'dx;
		endcase
	end

	wire [31:0] sms[0:3];
	assign sms[0] = { data_from_cpu[7:0] , data_from_cpu[7:0] , data_from_cpu[7:0] , data_from_cpu[7:0] };
	assign sms[1] = { data_from_cpu[15:0] , data_from_cpu[15:0] };
	assign sms[2] = data_from_cpu;
	assign sms[3] = data_from_cpu;
	assign data2Peri = sms[AWidth];

    function [3:0] wstb_func( input wra , input [1:0] bwa , adl2 );
        casex( {wra,bwa,adl2} )
            5'b1_1000 : wstb_func = 4'b1111;
            5'b1_0100 : wstb_func = 4'b0011;
            5'b1_0110 : wstb_func = 4'b1100;
            5'b1_0000 : wstb_func = 4'b0001;
            5'b1_0001 : wstb_func = 4'b0010;
            5'b1_0010 : wstb_func = 4'b0100;
            5'b1_0011 : wstb_func = 4'b1000;
            default   : wstb_func = 4'b0000;
        endcase
    endfunction

    assign byte_write =  wstb_func( wr , AWidth , addrbus[1:0] );

endmodule

/* top module of minimum risc-v system */
module Soc( input wire clock , reset , sw1 , rx , output wire tx ,
				output wire [2:0] FullColor_LED
                ,output wire [21:0] LCD_out );

	parameter INSTRUCTION_RESET_VECTOR = 32'h0000_0000;
	parameter INTERRUPT_VECTOR = 32'h0000_0040;
	parameter RAM_SIZE = 4; // UNIT : kbyte
	parameter INSTRUCTION_ADDRESS_WIDTH = $clog2(RAM_SIZE*1024) - 2 ;
	parameter WRITE_CYC_OPTIMIZE = 1;
	parameter MEMORY_TYPE = 0;
	parameter LOAD_FILE = "test.hex" ;
//	parameter LOAD_FILE = "testb.hex" ;

    `include "SelPara.v"
/*    parameter MTVAL_C = 9'o102;
    parameter XD_C = 9'o120;  //120
    parameter CN_C = 18'o015342;
    parameter CSR_C = 12'o2130;
    parameter PCSEL_C = 6'o10;*/

	wire	[31:0]	PRDATA1,PRDATA2;
	wire				PSEL1,PSEL2,INT_S;
	wire	[31:0]	inst_addr,inst_data;
	wire	[31:0]	PADDR,PRDATA,PWDATA,PWDATA2;
	wire				PENABLE,PWRITE,PSEL,sign_ext;
	wire	[3:0]		PSTRB;
	wire	[1:0]		data_width;
	wire	[14:0]	opecode;

	wire    tgl_out,tx_wrappr;
	wire    [2:0] LED_wrapper;
    wire    [21:0] LCD_wrapper;

	assign  FullColor_LED[2:0] = LED_wrapper[2:0];
    assign  tx = (tx_wrappr)? 1'bz : 1'b0;
    generate
    genvar k;
        for( k = 0 ; k <= 21 ; k = k +1 ) begin : LCD_OPENDRAINOUT
            assign LCD_out[k] = (LCD_wrapper[k])? 1'bz : 1'b0;
        end
    endgenerate

	riscv32core_rv32i
	#( .RESET_VECTOR(INSTRUCTION_RESET_VECTOR), .VALID_PC_WIDTH(INSTRUCTION_ADDRESS_WIDTH),
		.WRITE_CYCLE_OPT(WRITE_CYC_OPTIMIZE), .MEMORY_TYPE(MEMORY_TYPE) ,.DEFAULT_MTVEC(INTERRUPT_VECTOR)
        ,.REGISTER_EXPRESSION( 0 )
        ,.MTVAL_CHOICE( MTVAL_C ), .LP_XD_CHOICE( XD_C ), .LP_CN_CHOICE( CN_C )
        ,.LP_PC_SEL_CHOICE( PCSEL_C ) ,.LP_CSR_CHOICE( CSR_C )  )
		cpu1( .reset(reset), .clk(clock) , .NMI_S(1'b0) , .INT_S(INT_S) , 
			  .inst_addr(inst_addr) , .inst_data(inst_data) ,  
			  .PADDR(PADDR) , .PRDATA(PRDATA) , .PWDATA(PWDATA) ,
			  .PENABLE(PENABLE) , .PWRITE(PWRITE) , .PSEL(PSEL) , .Awidth(data_width), .s_ext(sign_ext),
			  .outcode(opecode) );

	bus_master  #(	.RESET_VECTOR(INSTRUCTION_RESET_VECTOR) , 
						.INST_ADDR_WIDTH(INSTRUCTION_ADDRESS_WIDTH),
						.RAM_SIZE(RAM_SIZE) ) 
             bm1( .addrbus(PADDR)  , .data_from_cpu(PWDATA) , .data2Peri(PWDATA2),
						.data2cpu(PRDATA), .dataFp1(PRDATA1), .dataFp2(PRDATA2), 
                  .AWidth(data_width) , .s_ext(sign_ext), .wr(PWRITE) , .byte_write(PSTRB),
						.cs(PSEL), .cs1(PSEL1), .cs2(PSEL2) );

	EXT_RAM  #( .INST_ADDR_WIDTH(INSTRUCTION_ADDRESS_WIDTH) , .LOAD_HEX_FILE(LOAD_FILE),
					.RAM_SIZE(RAM_SIZE), .MEMORY_TYPE(MEMORY_TYPE) ) 
         ram1( .clk(clock) , .reset(reset) ,
                .addr1(inst_addr) ,
                .q1(inst_data) ,
				.addr2(PADDR) , 
				.d22(PWDATA2),
				.q2(PRDATA1),
				.we2(PENABLE & PSEL1),
                .wstrb(PSTRB) );

	Super_IO pripheral1(	.clk(clock),
							.reset(reset) ,
							.addr(PADDR) ,
							.indata(PWDATA) ,
							.outdata(PRDATA2) , 
							.pe(PENABLE) , .cs(PSEL2) , .rdwr(PWRITE),
							.timer_out(tgl_out) , .timer_int(INT_S) ,
							.sw1(sw1),
							.RGB_LED(LED_wrapper),
							.LCD(LCD_wrapper),
                            .tx_serial_out(tx_wrappr),
                            .rx_serial_in(rx) );
endmodule




