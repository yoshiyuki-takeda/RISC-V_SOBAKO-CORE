
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  蕎麦粉コア (Sobako Koa) Version 00
  portability and Small size RISC-V core       # #         #        #     # #     RRRR   III   SSSS    CCCC   V     V
                                             #######    #######   # # #   # #     R   R   I   S    S  C    C  V    V
                                               # #         #      # # #  #   #    R   R   I   S       C       V   V
  RISC-V ABI suport : RV32I                   #####      #####    # # #  #   #    RRRR    I    SSSS   C       V  V
                                               #           #      # # #  #   #    R R     I        S  C       V V
  Copyright 2022-2023 yoshiyuki.takeda       #######   #########  # # # #     #   R  R    I   S    S  C    C  VV
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
							 output wire [31:0] inst_addr , PADDR , PWDATA , 
                             output wire PENABLE , PWRITE , PSEL , s_ext , output wire [1:0] Awidth,
							 output wire [14:0] outcode );

	parameter RESET_VECTOR  = 32'h0000_0000;
    parameter VALID_PC_WIDTH = 10;
	parameter NMI_VECTOR    = 32'h0000_1230;
    parameter DEFAULT_MTVEC = 32'h0000_0050;
	parameter SHIFT_SELECTOR = 0;
	parameter SHIFT_LATENCY = 1;

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

    function [31:0] XXd1_sel( input in1,in2,in3, input [31:0] ret1,ret2,ret3 );
        casex( {in1,in2,in3} )
            3'b1xx : XXd1_sel = ret1;
            3'b01x : XXd1_sel = ret2;
            3'b001 : XXd1_sel = ret3;
            default : XXd1_sel = 32'd0;
        endcase
    endfunction

    /* end of declaration */

	wire [31:0] inst;
	wire [4:0] rd,rs1,rs2;
	wire [4:0] code_sysb;
	wire [2:0] funct3;
	wire [6:0] op,funct7;

	wire CODE_LUI ,CODE_AUIPC,CODE_JAL  ,CODE_JALR   ,CODE_BRCH ,CODE_LOAD,CODE_STORE;
	wire CODE_ALUI,CODE_ALUR ,CODE_FENCE,CODE_FENCEI,CODE_ECALL,CODE_EBRK ,CODE_CSR,CODE_MRET;
	wire code_sys,compliment,Load_sub,sft,sft_lr,as,f7i,ALU_sub,ALU_cmp,csr_imac;
	wire [1:0] BRCH_sub,ACC_Width,CSR_sub;

	wire [31:0] SI_imm, B_imm, U_imm, J_imm ,csr_imm;
	wire [11:0] csr_addr;

	reg [VALID_PC_WIDTH-1:0] pc;
	reg [2:0]  stg;
	wire cndtn[3:0];
	wire BRANCH_F;
    wire exec_op;
	wire [VALID_PC_WIDTH-1:0] pc_calc,pc_calc_transfer,pc_add_sel[0:2],sel_p[0:3];
	wire [1:0] sel_pc,pc_add_num;

	reg [31:0] x1,x2;
	reg [4:0] q;
    reg sft_wait;
	reg [31:0] shiftr;
	wire [31:0] xd1,xd2,logic_op,ALU_out,ALU_add_sub,Others_xd,s2_sel[0:2],xd2_sel[0:3],alu_sel[0:3];//,lms[0:3]
	wire [4:0] x2xd,x1xd;
	wire [5:0] reg1addr,reg2addr;
	wire [31:0] x1val,x2val,S2_ImmReg;//s2
    reg [31:0] greg[0:47]; 
	wire we_reg,we_csr,addr_en,stillshift,except_en,cmp_us,cmp_sg,reg1en,reg2en,fsft,Add_Sub_Sel;
	wire [1:0] xd2_sn,s2_num;
    wire [31:0] L_shift,sft_in,in_invLR,R_shift;

    wire e_DataAddrMiss,e_Inst,Jump_e,NMI_int_en,int_en;
	wire [4:0] Ecode;
	wire [31:0] csr_nn;
	wire [31:0] csr_sel;
	reg MIE_bit,MPIE_bit,MEIE_bit,NMIE_bit;
	wire [31:0] csr_value;
	wire [31:0] csr_mux[0:8];
	reg [3:0] csr_num;
	
	assign inst = inst_data;
	assign { funct7  , rs2  , rs1 , funct3  , rd , op } = inst;
	assign code_sys = ({inst[31:30],inst[27:23],inst[19:7],op}==27'b0000000000_0000000000_1110011);
	assign code_sysb= {inst[29:28],inst[22:20]};

	assign BRCH_sub[1:0] = funct3[2:1];		//conditional jump
	assign compliment = funct3[0];		//branch compliment
	assign ACC_Width[1:0] = funct3[1:0];		//memory access  
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

	assign SI_imm = { { 21{inst[31]} } , { inst[30:25] } , {(CODE_STORE)?inst[11:7]:inst[24:20]} };
	assign B_imm = { { 20{inst[31]} } , {inst[7]} , {inst[30:25]} , {inst[11:8]} , {1'b0} };
	assign U_imm = { inst[31:12] , 12'h000 };
	assign J_imm = { { 12{inst[31]} } , {inst[19:12]} , {inst[20]} , { inst[30:21] } , {1'b0} };
	assign csr_addr[11:0] = inst[31:20];
	assign csr_imm = { 27'd0 , inst[19:15] };

	assign NMI_int_en = NMIE_bit & NMI_S;
	assign int_en = MIE_bit & MEIE_bit & INT_S;
    assign addr_en = (stg>'d1)&(CODE_LOAD|CODE_STORE);
	assign e_DataAddrMiss = addr_en&( (Awidth[0]&PADDR[0]) | (Awidth[1]&(PADDR[0]|PADDR[1])) );
	assign e_Inst = (~(|outcode)) & ((stg!='d0)) ;
 	assign except_en = e_DataAddrMiss | e_Inst | CODE_EBRK | CODE_ECALL;
	assign Jump_e = NMI_int_en|except_en|int_en;

   assign Ecode = ((~reset)|NMI_int_en)? 5'd31 : (int_en|CODE_ECALL)? 5'd11 : (CODE_EBRK)? 5'd3:
							  (e_Inst)? 5'd2 : (e_DataAddrMiss)? ((CODE_LOAD)? 5'd4 : 5'd6) : 5'd0  ;

    assign s2_num = ((stg>='d6)|CODE_MRET)? 2'd0 : (CODE_ALUI|CODE_JALR|CODE_LOAD|CODE_STORE)? 2'd2 : 2'd1;
    assign s2_sel[2] = SI_imm;
    assign s2_sel[1] = x2;
    assign s2_sel[0] = 'd0;
    assign S2_ImmReg = s2_sel[s2_num];

	assign csr_sel = (csr_imac) ?  csr_imm  : x1;
	assign Add_Sub_Sel = ~( (CODE_ALUR&ALU_sub) | ((CODE_ALUR|CODE_ALUI)&ALU_cmp) | CODE_BRCH );
	assign ALU_add_sub = ( Add_Sub_Sel ) ? x1+S2_ImmReg : x1-S2_ImmReg;
	assign cmp_us = (x1[31]==S2_ImmReg[31])? ALU_add_sub[31] : S2_ImmReg[31];  //x1<s2;  
	assign cmp_sg = (x1[31]==S2_ImmReg[31])? ALU_add_sub[31] : x1[31];         //$signed(x1)<$signed(s2);  

	//memory access circuit
	assign PADDR = ALU_add_sub;
	assign Awidth = ACC_Width;
    assign s_ext = Load_sub;
	assign PWDATA = x2;
    assign PWRITE  = (stg>'d1)&(CODE_STORE); //
    //assign PWRITE  = (~CODE_LOAD) & addr_en; //
    assign PSEL = addr_en;
	//assign PENABLE = (stg=='d3)&(CODE_STORE|CODE_LOAD)&(~e_DataAddrMiss);
	assign PENABLE = (stg=='d3)& addr_en &(~e_DataAddrMiss);

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

    assign Others_xd = ( (stg=='d5)? {NMI_S|INT_S,26'd0,Ecode} : ((stg<'d5)&CODE_LUI)? 32'd0 : inst_addr ) 
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

    assign we_reg = ( (stg=='d1)&(CODE_AUIPC|CODE_LUI|CODE_JAL) )
                  | ( (stg=='d2)&( ((~stillshift)&(~fsft)&(CODE_ALUR|CODE_ALUI)) | CODE_JALR ) )
                  | ( (stg=='d3)&CODE_LOAD) ;

	assign we_csr =   (stg=='d2)&CODE_CSR;

    assign xd1 = XXd1_sel( e_DataAddrMiss , e_Inst , (stg<'d5)&CODE_CSR , PADDR , inst , csr_value  );

	assign xd2_sel[3] = PRDATA;
	assign xd2_sel[2] = ALU_out;
	assign xd2_sel[1] = csr_nn;
	assign xd2_sel[0] = Others_xd;
	assign xd2_sn = ((stg<'d5)&(CODE_ALUR|CODE_ALUI))? 2'd2 : ((stg<'d5)&(CODE_LOAD)) ? 2'd3 : ((stg<'d5)&(CODE_CSR))? 2'd1 : 2'd0 ;
	assign xd2 = xd2_sel[xd2_sn];

	assign x1xd = ( we_csr ) ? rd : rs1;
	assign reg1en = (stg=='d5) | (we_csr&(x1xd>'d0));
    assign reg1addr = (stg>='d6)? 6'h20 : (stg=='d5)? 6'h22 : ((stg<'d5)&CODE_MRET) ? 6'h21 : {1'b0,x1xd};
    assign x1val= xd1;

	assign x2xd = ( we_reg ) ? rd : rs2;
	assign reg2en = (we_csr&(csr_num<='d1)) | (we_reg&(x2xd>'d0)) | (stg=='d6) | (stg=='d5); 
    assign reg2addr = (stg=='d6)? 6'h21 : (stg=='d5)? 6'h23 : ((stg<'d5)&CODE_CSR)? {2'b10,csr_num[3:0]} : {1'b0,x2xd} ;
    assign x2val= xd2;

    always @(posedge clk) begin
        x1 <= greg[reg1addr];
        if(reg1en)
            greg[reg1addr] <= x1val;
    end
    always @(posedge clk) begin
        x2 <= greg[reg2addr];
        if(reg2en)
            greg[reg2addr] <= x2val;
    end

    // h20(d32) : mtvec , h21(d33) : mepc , h22(d34) : mtval , h23(d35) : mcause
    integer k;
    initial begin
        for( k=0 ; k<48 ; k=k+1 )
            if( k == 32 ) greg[k] = DEFAULT_MTVEC;
            else          greg[k] = 'd0;
    end
	
	//program counter circuit
	//assign inst_addr = {14'd0,pc,2'd0};
	//assign inst_addr = { RESET_VECTOR[31 -: (32-VALID_PC_WIDTH-((VALID_PC_WIDTH < 30)? 2:1))]  , pc , 2'd0};
	assign inst_addr = { RESET_VECTOR[31 -: (32-VALID_PC_WIDTH-2)]  , pc , 2'd0};

	assign cndtn[2'b00] = ~(|ALU_add_sub); // x1 == x2;  
	assign cndtn[2'b01] = 1'bx;
	assign cndtn[2'b10] = cmp_sg;
	assign cndtn[2'b11] = cmp_us;
    assign BRANCH_F = CODE_BRCH&(cndtn[BRCH_sub] == compliment) ;

    assign pc_add_num = (CODE_BRCH)? 2'd2 : (CODE_JAL)? 2'd0 : 2'd1 ;
    assign pc_add_sel[2] = B_imm[2 +: VALID_PC_WIDTH];
    assign pc_add_sel[1] = { {(VALID_PC_WIDTH-1){1'b0}} , {1'b1} }  ;
    assign pc_add_sel[0] = J_imm[2 +: VALID_PC_WIDTH];
	assign pc_calc_transfer = pc + pc_add_sel[pc_add_num];

    assign sel_pc = (stg>='d6) ? ( (NMI_int_en)? 2'd1 : //NMI
                                                 2'd3): //interrupt/except
                          (CODE_MRET|CODE_JALR)? 2'd3 : //MRET , x + i
                                     (BRANCH_F)? 2'd2 : //branch false(Others_xd,pc + 4)
                                                 2'd0 ; //branch true(pc + b) , jal(pc + j) , pc + 4

	assign sel_p[3] = ALU_add_sub[2 +: VALID_PC_WIDTH]; /*mtvec,MRET,JALR*/
	assign sel_p[2] = Others_xd[2 +: VALID_PC_WIDTH]; /* branch false */
	assign sel_p[1] = NMI_VECTOR[2 +: VALID_PC_WIDTH];
	assign sel_p[0] = pc_calc_transfer; /*Branch true , Jal , pc+4*/
	assign pc_calc = sel_p[sel_pc];

	assign exec_op =  we_reg | we_csr | PENABLE | ((stg == 'd1)&(CODE_FENCE|CODE_FENCEI)) | ((stg == 'd2)&(CODE_BRCH|CODE_MRET)) ;

	always @( posedge clk or negedge reset ) begin
		if( ~reset )
			pc <= RESET_VECTOR[ 2 +: VALID_PC_WIDTH ]; // bit2 -> bit17
		else
		begin
			if( (stg == 'd7) | exec_op )
				pc <= pc_calc;
		end
	end

	//cpu stage circuit
	always @( posedge clk or negedge reset ) begin
		if( ~reset )
			stg <= 'd0;
		else begin
			if( exec_op )
				stg <= 'd0;
			else
				stg <= ( ((stg<'d5)&Jump_e&(~exec_op))? 3'd4 : stg ) + ( ( (~Jump_e)&(stillshift|fsft) ) ? 3'd0 : 3'd1 );
		end
	end

	/*CSR*/
	assign csr_value = csr_mux[csr_num];

	assign csr_mux[2] = x2;// mtval	    Machine bad address or instruction.;
	assign csr_mux[3] = x2;// mcause	Machine trap cause.; //{ mc_Int , 23'd0 , Ecode } ;
	assign csr_mux[0] = x2;// mtvec		Machine trap-handler base address.;
	assign csr_mux[1] = x2;// mepc		Machine exception program counter.;//{mepc , 2'b00 }; 

	assign csr_mux[4] = { 19'd0,2'b11,3'd0,MPIE_bit,3'd0,MIE_bit,3'd0 }; // mstatus		Machine status register.;
	assign csr_mux[5] = { 20'd0 , MEIE_bit , 11'd0 }; // mie			Machine interrupt-enable register.;
	assign csr_mux[6] = { 20'd0 , INT_S , 11'd0 }; // mip Machine interrupt pending.  //8'h44 : csr_value = { 20'd0 , MEIP_bit , 11'd0 };
	assign csr_mux[7] = { 2'b01 , 4'b0000 , 26'b00_0000_0000_0000_0001_0000_0000 };  // misa		ISA and extensions;;
	assign csr_mux[8] = 32'd0;

	always @(*) begin
		if( csr_addr[11:8]=='d3 ) begin
			case( csr_addr[7:0] )
				'h00 : csr_num = 'd4; //mstatus
				'h01 : csr_num = 'd7; //misa
				'h04 : csr_num = 'd5; //mie
				'h05 : csr_num = 'd0; //mtvec
				'h41 : csr_num = 'd1; //mepc
				'h42 : csr_num = 'd3; //mcause
				'h43 : csr_num = 'd2; //mtval
				'h44 : csr_num = 'd6; //mip
				default : csr_num = 'd8;
			endcase
		end
		else
			csr_num = 'd8;
	end

	always @(posedge clk or negedge reset) begin
		if( ~reset ) begin
			MIE_bit  <= 'd0;
			MPIE_bit <= 'd0;
			MEIE_bit <= 'd0;
			NMIE_bit <= 'd1;
		end
		else begin
			if( stg == 'd7 ) begin
				if(NMI_int_en) NMIE_bit <='d0;
				if(Jump_e) MPIE_bit <= MIE_bit;
				if(Jump_e) MIE_bit <= 'd0;
			end
			if( (stg == 'd2)&CODE_MRET ) begin
				NMIE_bit <= 'd1;
				MIE_bit <= MPIE_bit;
				MPIE_bit <= 'd1;
			end
			if( we_csr&(csr_num=='d4) ) { MPIE_bit , MIE_bit } <= { csr_nn[7], csr_nn[3] }; // mstatus		Machine status register.
			if( we_csr&(csr_num=='d5) ) MEIE_bit <= csr_nn[11]; // mie		Machine interrupt-enable register.
		end
	end

endmodule

`define MEM_QUAD_SV
//`define MEM_QUAD_V

/* main memory */
module EXT_RAM( input wire [31:0] d22, addr1 , addr2 , 
                input wire clk, we2, reset , 
                input wire [3:0] wstrb,
				    output reg [31:0] q1 , q2 );

    parameter INST_ADDR_WIDTH = 10;
	 parameter RAM_ADJUSTOR = 0;
	wire [INST_ADDR_WIDTH-1:0] ad1,ad2;
	assign ad1 = addr1[2 +: INST_ADDR_WIDTH];
	assign ad2 = addr2[2 +: INST_ADDR_WIDTH];

	integer byte_no;

// Quad divided memory using SystemVerilog expression for intel quartus
`ifdef MEM_QUAD_SV
	reg [3:0][7:0] mem[0:2**INST_ADDR_WIDTH - 1 - RAM_ADJUSTOR];
	always @(posedge clk) begin
		q1 <= mem[ ad1 ];
		q2 <= mem[ ad2 ];
		if( we2&wstrb[0] ) mem[ ad2 ][0] <= d22[ 0 +: 8];
		if( we2&wstrb[1] ) mem[ ad2 ][1] <= d22[ 8 +: 8];
		if( we2&wstrb[2] ) mem[ ad2 ][2] <= d22[16 +: 8];
		if( we2&wstrb[3] ) mem[ ad2 ][3] <= d22[24 +: 8];
	end

// Quad dividec meory using Verilog expression
`elsif MEM_QUAD_V
	reg [31:0] mem[0:2**INST_ADDR_WIDTH - 1 - RAM_ADJUSTOR];
	always @(posedge clk) begin
		q1 <= mem[ ad1 ];
		q2 <= mem[ ad2 ];
		for( byte_no = 0 ; byte_no < 4 ; byte_no = byte_no + 1 )
			if( we2&wstrb[byte_no] )
				mem[ ad2 ][ 8*byte_no +: 8 ] <= d22[ 8*byte_no +: 8 ];
	end

// any verilog can compile
`else
	reg [31:0] mem[0:2**INST_ADDR_WIDTH - 1 -RAM_ADJUSTOR];
	wire [7:0] din2[0:3];
	wire [1:0] wea;
	assign wea[0] = we2&(wstrb[1] | wstrb[0]);
	assign wea[1] = we2&(wstrb[3] | wstrb[2]);

	genvar byte_no2;
	generate
		for( byte_no2 = 0 ; byte_no2 < 4 ; byte_no2 = byte_no2 + 1 ) begin : mem_choice
			assign din2[byte_no2] =   (wstrb[byte_no2]) ? d22[8*byte_no2 +: 8] : q2[8*byte_no2 +: 8];
		end
	endgenerate
			
	always @(posedge clk) begin
		q1 <= mem[ ad1 ];
		q2 <= mem[ ad2 ];
		if( wea[0] ) mem[ ad2 ][ 0 +: 16] <= { din2[1] , din2[0] };
		if( wea[1] ) mem[ ad2 ][16 +: 16] <= { din2[3] , din2[2] };
	end
`endif

	initial begin  //memory initialize
		$readmemh( "./test.hex" , mem ); //program read from hex file
	end //memory initial end 
	
endmodule 

/* unified peripheral */
module Super_IO ( input wire clk,reset ,  input wire pe , rdwr , cs , input wire [31:0] addr,
						input wire [31:0] indata , output reg [31:0] outdata , 
						output wire timer_out , timer_int ,
						input wire sw1,
						output wire [2:0] RGB_LED,
						output wire [6:0] LCD,
						output wire tx_serial_out,
						input  wire rx_serial_in
						);
parameter WIDTH = 16;
parameter PRIOD_WIDTH = 15;
// tang nano 1k = 27MHz

    function [1:0] func_LCD_choice( input [2:0] in_l );
        case( in_l )
            'd3 : func_LCD_choice = 'd0 ;
            'd5 : func_LCD_choice = 'd1 ;
            'd6 : func_LCD_choice = 'd2 ;
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
	reg [2:0] LCD_bit[0:2];
	reg [2:0] LCD_com;
	reg [PRIOD_WIDTH-1:0] LCD_tgl_period;
    wire [1:0] LCD_choice;

	reg [2:0] LED_out;

    wire [7:0] rx_data_path;
    wire tx_ctrl,we_tx,rx_ctrl,rd_rx;

    assign we_tx = pe & cs & rdwr & (addr_in == 'd8);
    assign rd_rx = pe & cs & ~rdwr & (addr_in == 'd10);
    suart_tx uart1_tx( .clk(clk) , .rst(reset) , .we(we_tx) , .tx_data(indata[7:0]) , .tx_serial(tx_serial_out) , .tx_empty(tx_ctrl)   );
    suart_rx uart1_rx( .clk(clk) , .rst(reset) , .rd(rd_rx) , .buf_rx(rx_data_path) , .rx_serial(rx_serial_in)  , .valid_data(rx_ctrl) );

	/* LCD driver */
	always @(posedge clk or negedge reset ) begin
		if( ~reset ) begin
			LCD_tgl_period <= 'd1; //{(PRIOD_WIDTH){1'b0}};
            LCD_com <= 'd6;
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

    assign LCD_choice = func_LCD_choice(LCD_com);
	assign LCD = { LCD_com , LCD_bit[LCD_choice][2]^LCD_tgl , LCD_bit[LCD_choice][1]^LCD_tgl , LCD_bit[LCD_choice][0]^LCD_tgl , LCD_tgl };

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
			'd6 : outdata <= { 23'dx , LCD_bit[2] , LCD_bit[1] , LCD_bit[0] };  // LCD pattern bit1

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
			LCD_bit[0] <= 'b111;
			LCD_bit[1] <= 'b111;
			LCD_bit[2] <= 'b011;
			LED_out <= 3'b111;
		end
		else begin
			if( pe & cs & rdwr ) begin
				case ( addr_in )
					'd0 : rld_reg <= indata[WIDTH-1:0];
					'd2 : {tmr_IE , tmr_en} <= indata[1:0];
					'd4 : LED_out <= indata[2:0];
					'd6 : { LCD_bit[2][1:0] , LCD_bit[1] , LCD_bit[0] } <= indata[7:0];
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

/* Simple UART send data */
module suart_tx( input wire [7:0] tx_data , input wire clk , rst , we , output wire tx_empty,tx_serial );
	parameter UART_DIV = 8'd234; //(27MHz/115200boud)

	reg [9:0] tx_sreg;
	reg [7:0] div_count;
	reg [3:0] send_count;
	
	always @( posedge clk or negedge rst ) begin // uart send timing tick
		if( ~rst ) begin
			div_count <= UART_DIV;
		end
		else begin
			if( (div_count == 8'd0) || (tx_empty&we) ) div_count <= UART_DIV;
			else div_count <= div_count - 8'd1;
		end
	end
	
	always @( posedge clk or negedge rst ) begin // send data
		if( ~rst ) begin
			tx_sreg <= 10'b11_1111_1111;
		end
		else begin
			if( tx_empty&we ) tx_sreg <= { 1'b1 , tx_data , 1'b0 };
			else if( div_count == 8'd0 ) tx_sreg <= { 1'b1 , tx_sreg[9:1] };
		end
	end
	
	assign tx_serial = tx_sreg[0];

	always @(posedge clk or negedge rst ) begin // counting how many bits send
		if( ~rst ) begin
			send_count <= 4'd0;
		end
		else begin
			if( tx_empty&we ) send_count <= 4'd10;
			else if( (div_count == 8'd0) && (send_count > 4'd0) ) send_count <= send_count - 4'd1;
		end
	end
	
	assign tx_empty = (send_count == 5'd0);

endmodule

/* Simple UART reciev data */
module suart_rx( input wire clk , rst , rd , rx_serial, output reg [7:0] buf_rx , output reg valid_data );
	parameter UART_DIV = 9'd234; //(27MHz/115200boud)
	parameter UART_FST = (UART_DIV >> 1);

	reg [7:0] rx_data;
	reg [8:0] div_count;
	reg [3:0] rcv_count;
	reg [2:0] serial_in;
	reg       rcv_edge;

	always @( posedge clk or negedge rst ) begin // serial signal anti metastable and detect start bit
		if( ~rst ) serial_in <= 'd0;//{3{rx_serial}};
		else serial_in <= { serial_in[1:0] , rx_serial };
	end

	always @( posedge clk or negedge rst ) begin // uart read timing tick
		if( ~rst ) begin
			div_count <= UART_DIV;
		end
		else begin
			if( serial_in[2:1] == 2'b10 && rcv_count == 4'd0 ) div_count <= UART_FST;
			else if( div_count == 8'd0 ) div_count <= UART_DIV;
			else div_count <= div_count - 8'd1;
		end
	end

	always @( posedge clk or negedge rst ) begin // counting how many bytes read
		if( ~rst ) begin
			rcv_count <= 4'd0;
		end
		else begin
			if( serial_in[2:1] == 2'b10 && rcv_count == 4'd0 ) rcv_count <= 4'd10;
			else if( div_count == 8'd0 && rcv_count > 4'd0  ) rcv_count <= rcv_count - 4'd1;
		end
	end
	
	always @( posedge clk or negedge rst ) begin // data reading
		if( ~rst ) begin
			rx_data <= 8'd0;
		end
		else begin
			if( div_count == 8'd0 && rcv_count > 4'd1  ) rx_data <= { serial_in[1] , rx_data[7:1] };
		end
	end
	
	always @( posedge clk or negedge rst ) begin // detect lsb bit
		if( ~rst ) begin
			rcv_edge <= 1'b0;
		end
		else begin
			rcv_edge <= (rcv_count == 4'd1);
		end
	end
	
	always @( posedge clk or negedge rst ) begin // notice reading data is finished
		if( ~rst ) begin
		 valid_data <= 1'b0;
		end
		else begin
			if( (rcv_count == 4'd1) && (rcv_edge == 1'b0) ) valid_data <= 1'b1;
			else if( rd ) valid_data <= 1'b0;
		end
	end
	
	always @( posedge clk or negedge rst ) begin // notice reading data is finished
		if( ~rst ) begin
		 buf_rx <= 'd0;
		end
		else begin
			if( (rcv_count == 4'd1) && (rcv_edge == 1'b0) ) buf_rx <= rx_data;
		end
	end
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
	wire chk1 = (addrbus[31:INST_ADDR_WIDTH+2] == RESET_VECTOR[31:INST_ADDR_WIDTH+2]); // main memory
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
                ,output wire [6:0] LCD_out );

    parameter INSTRUCTION_RESET_VECTOR = 32'h0000_0000;
    parameter INSTRUCTION_ADDRESS_WIDTH = 10;
    
	wire	[31:0]	PRDATA1,PRDATA2;
	wire			PSEL1,PSEL2,INT_S;
	wire	[31:0]	inst_addr,inst_data;
	wire	[31:0]	PADDR,PRDATA,PWDATA,PWDATA2;
	wire			PENABLE,PWRITE,PSEL,sign_ext;
    wire    [3:0]   PSTRB;
	wire	[1:0]	data_width;
	wire	[14:0]	opecode;

    wire    tgl_out;
    wire    [2:0] LED_wrapper;

    assign  FullColor_LED[2:0] = LED_wrapper[2:0];

	riscv32core_rv32i #( .RESET_VECTOR(INSTRUCTION_RESET_VECTOR) , .VALID_PC_WIDTH(INSTRUCTION_ADDRESS_WIDTH) ) 
        cpu1( .reset(reset), .clk(clock) , .NMI_S(1'b0) , .INT_S(INT_S) , 
			  .inst_addr(inst_addr) , .inst_data(inst_data) ,  
			  .PADDR(PADDR) , .PRDATA(PRDATA) , .PWDATA(PWDATA) ,
			  .PENABLE(PENABLE) , .PWRITE(PWRITE) , .PSEL(PSEL) , .Awidth(data_width), .s_ext(sign_ext),
			  .outcode(opecode) );

	bus_master       #( .RESET_VECTOR(INSTRUCTION_RESET_VECTOR) , .INST_ADDR_WIDTH(INSTRUCTION_ADDRESS_WIDTH) ) 
                    bm1( .addrbus(PADDR)  , .data_from_cpu(PWDATA) , .data2Peri(PWDATA2),
						 .data2cpu(PRDATA), .dataFp1(PRDATA1), .dataFp2(PRDATA2), 
                         .AWidth(data_width) , .s_ext(sign_ext), .wr(PWRITE) , .byte_write(PSTRB),
						 .cs(PSEL), .cs1(PSEL1), .cs2(PSEL2) );

	EXT_RAM  #( .INST_ADDR_WIDTH(INSTRUCTION_ADDRESS_WIDTH) , .RAM_ADJUSTOR(0) ) 
                 ram1(  .clk(clock) , .reset(reset) ,
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
							.LCD(LCD_out),
                     .tx_serial_out(tx),
                     .rx_serial_in(rx) );
endmodule




