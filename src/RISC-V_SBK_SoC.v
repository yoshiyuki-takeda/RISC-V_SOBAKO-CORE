
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  蕎麦粉コア (Sobako Koa) Version 00 beta
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

/* RV32I CPU core */
module riscv32core_rv32i( input wire reset,clk,NMI_S,INT_S , input wire [31:0] inst_data , in_data , 
							 output wire [31:0] inst_addr , data_addr , out_data , output wire dwe , RDorWR , dcs , output wire [1:0] Awidth,
							 output wire [14:0] outcode );
	parameter RESET_VECTOR = 16'h0000;
	parameter NMI_VECTOR = 32'h0000_1230;
//	parameter INT_VECTOR = 32'h0000_0090; // interrupt entry address

    /*function declaration   */
	function LUT_cmp_func( input x1_in , s2_in , sub_msb ); //compair unsigned integer
		case( {x1_in,s2_in,sub_msb} )
			3'b00_0 : LUT_cmp_func = 1'b0;
			3'b00_1 : LUT_cmp_func = 1'b1;
			3'b11_0 : LUT_cmp_func = 1'b0;
			3'b11_1 : LUT_cmp_func = 1'b1;

			3'b01_0 : LUT_cmp_func = 1'b1;
			3'b01_1 : LUT_cmp_func = 1'b0;
			3'b10_0 : LUT_cmp_func = 1'b1;
			3'b10_1 : LUT_cmp_func = 1'b0;
			default : LUT_cmp_func = 1'b0;
		endcase
	endfunction

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

/*    function LUT_stg1_sft( input p3,p2,p1,p0 );
        case( {p3,p2,p1,p0} )
            4'b0011 : LUT_stg1_sft = 'b1;
            default : LUT_stg1_sft = 'b0;
        endcase
    endfunction

    function LUT_stg7_or_mret( input p3,p2,p1,p0 );
        casex( {p3,p2,p1,p0} )
            4'b111x : LUT_stg7_or_mret = 'b1;
            4'bxxx1 : LUT_stg7_or_mret = 'b1;
            default : LUT_stg7_or_mret = 'b0;
        endcase
    endfunction*/

    function [31:0] XXd1_sel( input in1,in2,in3, input [31:0] ret1,ret2,ret3 );
        casex( {in1,in2,in3} )
            3'b1xx : XXd1_sel = ret1;
            3'b01x : XXd1_sel = ret2;
            3'b001 : XXd1_sel = ret3;
            default XXd1_sel = 32'd0;
        endcase
    endfunction

/*    function [15:0] ppct_sel( input in1,in2, input [15:0] ret1,ret2 );
        casex( {in1,in2} )
            2'b1x : ppct_sel = ret1;
            2'b01 : ppct_sel = ret2;
            default ppct_sel = 16'd1;
        endcase
    endfunction*/

    /* end of declaration */

	wire [31:0] inst;
	wire [4:0] rd,rs1,rs2;
	wire [4:0] code_sysb;
	wire [2:0] funct3;
	wire [6:0] op,funct7;

	wire CODE_LUI ,CODE_AUIPC,CODE_JAL  ,CODE_JALR   ,CODE_BRCH ,CODE_LOAD,CODE_STORE;
	wire CODE_ALUI,CODE_ALUR ,CODE_FENCE,CODE_FENCEI,CODE_ECALL,CODE_EBRK ,CODE_CSR,COD_MRET_E;
	wire code_sys,compliment,Load_sub,sft,sft_lr,as,f7i,ALU_sub,ALU_cmp,csr_imac;
	wire [1:0] BRCH_sub,ACC_Width,CSR_sub;

	wire [31:0] SI_imm, B_imm, U_imm, J_imm ,csr_imm;
	wire [11:0] csr_addr;

	reg [15:0] pc;
	reg [2:0]  stg;
	wire cndtn[3:0];
	wire BRANCH_F;
	wire [15:0] pc_calc;
    wire exec_op;
	wire [15:0] pc_calc_transfer,pc_add_sel[0:2],pc_add;
	wire [15:0] sel_p[0:3];
	wire [1:0] sel_pc,pc_add_num;

	reg [31:0] x1,x2;
	reg [4:0] q;
    reg sft_wait;
	reg [31:0] shiftr;
	wire [31:0] xd1,xd2,logic_op,ALU_out,ALU_add_sub,s2,Others_xd,lms[0:3],s2_sel[0:2],xd2_sel[0:3],alu_sel[0:3];
	wire [4:0] x2xd,x1xd;
	wire [5:0] reg1addr,reg2addr;
	wire [31:0] x1val,x2val;
    reg [31:0] greg[0:47]; 
	wire we_reg,we_csr,addr_en,stillshift,except_en,cmp_us,cmp_sg,reg1en,reg2en,fsft,CODE_MRET,Add_Sub_Sel;
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
	assign COD_MRET_E  = code_sys & (code_sysb==5'b11010);

	assign outcode = { CODE_CSR, /*CODE_WFI,*/ COD_MRET_E, CODE_EBRK, CODE_ECALL, CODE_FENCEI ,CODE_FENCE, CODE_ALUR, CODE_ALUI,
									CODE_STORE, CODE_LOAD, CODE_BRCH, CODE_JALR, CODE_JAL ,CODE_AUIPC, CODE_LUI} ; //----------------------- no need

	assign SI_imm = { { 21{inst[31]} } , { inst[30:25] } , {(CODE_STORE)?inst[11:7]:inst[24:20]} };
	assign B_imm = { { 20{inst[31]} } , {inst[7]} , {inst[30:25]} , {inst[11:8]} , {1'b0} };
	assign U_imm = { inst[31:12] , 12'h000 };
//	assign J_imm = { { 12{inst[31]} } , {inst[19:12]} , {inst[20]} , { inst[30:21] } , {1'b0} };
	assign J_imm = { { 14{inst[31]} }  , {inst[31]} , {inst[16:12]} , {inst[20]} , { inst[30:21] } , {1'b0} };
	assign csr_addr[11:0] = inst[31:20];
	assign csr_imm = { 27'd0 , inst[19:15] };

	assign NMI_int_en = NMIE_bit & NMI_S;
    assign CODE_MRET = ~NMI_int_en&COD_MRET_E;
	assign int_en = MIE_bit & MEIE_bit & INT_S;
    assign addr_en = (stg>'d1)&(CODE_LOAD|CODE_STORE);
	assign e_DataAddrMiss = addr_en&( (Awidth[0]&data_addr[0]) | (Awidth[1]&(data_addr[0]|data_addr[1])) );
	assign e_Inst = ~(|outcode) ;
 	assign except_en = e_DataAddrMiss | e_Inst | CODE_EBRK | CODE_ECALL;
	assign Jump_e = NMI_int_en|except_en|int_en;

   assign Ecode = ((~reset)|NMI_int_en)? 5'd31 : (int_en|CODE_ECALL)? 5'd11 : (CODE_EBRK)? 5'd3:
							  (e_Inst)? 5'd2 : (e_DataAddrMiss)? ((CODE_LOAD)? 5'd4 : 5'd6) : 5'd0  ;

//    assign s2 = (stg>='d6)? 'd0 : (CODE_MRET)? 'd0 : (CODE_ALUI|CODE_JALR|CODE_LOAD|CODE_STORE)? SI_imm : x2;
    assign s2_num = ((stg>='d6)|CODE_MRET)? 2'd0 : (CODE_ALUI|CODE_JALR|CODE_LOAD|CODE_STORE)? 2'd2 : 2'd1;
    assign s2_sel[2] = SI_imm;
    assign s2_sel[1] = x2;
    assign s2_sel[0] = 'd0;
    assign s2 = s2_sel[s2_num];

	assign csr_sel = (csr_imac) ?  csr_imm  : x1;
    assign Add_Sub_Sel = ~( (CODE_ALUR&ALU_sub) | ((CODE_ALUR|CODE_ALUI)&ALU_cmp) | CODE_BRCH );
    assign ALU_add_sub = ( Add_Sub_Sel ) ? x1+s2 : x1-s2;
	assign cmp_us = LUT_cmp_func(x1[31],s2[31],ALU_add_sub[31]);  //x1<s2;  
    assign cmp_sg = ALU_add_sub[31];                             //$signed(x1)<$signed(s2);  

	//memory access circuit
	assign data_addr = ALU_add_sub;
	assign Awidth = ACC_Width;
	assign out_data = x2;
    assign RDorWR  = (stg>'d1)&(CODE_STORE); //
    assign dcs = addr_en;
	assign dwe = (stg=='d3)&CODE_STORE;

	//general register circuit 
    // load data operation
	assign lms[0] = { {24{Load_sub&in_data[7]}} ,{in_data[7:0]}  };
	assign lms[1] = { {16{Load_sub&in_data[15]}},{in_data[15:0]} };
	assign lms[2] = in_data;
	assign lms[3] = in_data;

    /* latency or start trigger of sequential shifter */
	always @( posedge clk or negedge reset ) begin
		if( ~reset ) sft_wait <= 1'b0;
        else sft_wait <= (stg=='d1) & sft & (CODE_ALUR|CODE_ALUI);
    end

    //*// sequential shifter
    wire s_msb = ALU_sub & x1[31];
	assign stillshift = (q>'d0);
    assign fsft = sft_wait;
    assign R_shift = shiftr;
    assign L_shift = shiftr;
	always @( posedge clk or negedge sft ) begin
		if( ~sft ) begin
			shiftr <= 32'd0;
			q <= 5'd0;
		end
		else begin
			if( fsft ) begin
				shiftr <= x1;
				q <= s2[4:0];
			end
			if( stillshift ) begin
				shiftr <= (sft_lr)? { s_msb , shiftr[31:1] } : { shiftr[30:0] , 1'b0 } ;
				q <= q - 5'd1;
			end
		end
	end/* end of sequential shifter */

    /*// multiply shifter
    wire [31:0] s_mul,s_mf;
    wire s_msb = ALU_sub&x1[31];
	generate
		genvar ssi;
		for( ssi = 0 ; ssi <= 31 ; ssi = ssi + 1 )
		begin : gen_inv_var
            assign in_invLR[31-ssi] = x1[ssi];
            assign R_shift[31-ssi] = (sft) ? ( (s_mf[ssi])? L_shift[ssi] : s_msb ) : 1'b0;
		end
	endgenerate
    assign stillshift = 1'b0;
    assign fsft = sft_wait;
    assign sft_in = (sft_lr) ? in_invLR : x1;
    assign s_mul = 1 << s2[4:0];
    assign s_mf = 32'hffff_ffff * s_mul;
    assign L_shift = sft_in * s_mul;/* end of multiply shifter */

   /*// harf barrel shifter
	 wire signed[31:0] AShift;
	 wire [31:0] tmp_shift;
    wire s_msb = ALU_sub&x1[31];
	generate
		genvar ssi;
		for( ssi = 0 ; ssi <= 31 ; ssi = ssi + 1 )
		begin : gen_inv_var
            assign in_invLR[31-ssi] = x1[ssi];
				assign R_shift[ssi] = LUT_sel_or( sft , 1'b0 , tmp_shift[ssi] , AShift[ssi] );
            assign L_shift[31-ssi] = R_shift[ssi];
		end
	endgenerate
    assign fsft = sft_wait;
    assign stillshift = 1'b0;
    assign sft_in = (sft_lr) ? x1 : in_invLR;
	 assign tmp_shift = sft_in >> s2[4:0];
	 assign AShift = $signed({s_msb,31'd0}) >>> s2[4:0];/* end of harf barrel shifter */

   /*// full barrel shifter
	 wire signed[31:0] AShift;
	 wire [31:0] tmp_shift;
    assign fsft = sft_wait;
    assign stillshift = 1'b0;
    assign L_shift = x1 << s2[4:0];
	 assign tmp_shift = x1 >> s2[4:0];
	 assign AShift = $signed(x1) >>> s2[4:0];
    assign R_shift = (sft) ? ( (ALU_sub)? AShift : tmp_shift ) : 32'd0;/* end of full barrel shifter */

	assign alu_sel[0] = ALU_add_sub;
	assign alu_sel[1] = L_shift;
	assign alu_sel[2] = {31'd0,cmp_sg};
	assign alu_sel[3] = {31'd0,cmp_us};

    assign Others_xd = ( (stg=='d7)? {NMI_S|INT_S,26'd0,Ecode} : ((stg<'d6)&CODE_LUI)? 32'd0 : inst_addr ) 
                                       + ( (stg>='d6) ? 32'd0 : (CODE_AUIPC|CODE_LUI) ? U_imm : 32'd4 );

	generate
		genvar i;
		for( i = 0 ; i <= 31 ; i = i + 1 )
		begin : gen_LUT4exec
			assign logic_op[i] = LUT_and_or_xor_z( funct3[1:0] , x1[i] , s2[i] );
			assign ALU_out[i]  = LUT_sel_or( funct3[2] , alu_sel[ funct3[1:0] ][i] , R_shift[i] , logic_op[i] );
            assign csr_nn[i]   = LUT_csr_v( CSR_sub , csr_sel[i] , csr_value[i] );
		end
	endgenerate

    assign we_reg = ( (stg=='d1)&(CODE_AUIPC|CODE_LUI|CODE_JAL) )
                  | ( (stg=='d2)&( ((~stillshift)&(~fsft)&(CODE_ALUR|CODE_ALUI)) | CODE_JALR ) )
                  | ( (stg=='d3)&CODE_LOAD) ;
	assign we_csr =   (stg=='d2)&CODE_CSR;

    assign xd1 = XXd1_sel( e_DataAddrMiss , e_Inst , CODE_CSR  , data_addr , inst , csr_value  );

	assign xd2_sel[3] = lms[ACC_Width];
	assign xd2_sel[2] = ALU_out;
	assign xd2_sel[1] = csr_nn;
	assign xd2_sel[0] = Others_xd;
	assign xd2_sn = (stg>='d6)? 2'd0 : (CODE_LOAD)? 2'd3 : (CODE_CSR)? 2'd1 : (CODE_ALUR|CODE_ALUI)? 2'd2 : 2'd0 ;
	assign xd2 = xd2_sel[xd2_sn];

	assign x1xd = ( we_csr ) ? rd : rs1;
	assign reg1en = (stg=='d5) | (we_csr&(x1xd>'d0));
    assign reg1addr = (CODE_MRET) ? 6'h21 : (stg>='d6)? 6'h20 : (stg=='d5)? 6'h22 : {1'b0,x1xd};
    assign x1val= xd1;

	assign x2xd = ( we_reg ) ? rd : rs2;
	assign reg2en = (we_reg&(x2xd>'d0)) | (we_csr&(csr_num<='d1)) | (stg>='d6) ;
    assign reg2addr = (stg=='d7)? 6'h23 : (stg=='d6)? 6'h21 : (CODE_CSR)? {2'b10,csr_num[3:0]} : {1'b0,x2xd} ;
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

    integer k;
    initial begin
        for( k=0 ; k<48 ; k=k+1 )
            greg[k] = 'd0;
    end
	
	//program counter circuit
	assign inst_addr = {14'd0,pc,2'd0};

	assign cndtn[2'b00] = ~(|ALU_add_sub); // x1 == x2;  
	assign cndtn[2'b01] = 1'bx;
	assign cndtn[2'b10] = cmp_sg;
	assign cndtn[2'b11] = cmp_us;
    assign BRANCH_F = CODE_BRCH&(cndtn[BRCH_sub] == compliment) ;

    assign sel_pc = (stg>='d6) ? ( (NMI_int_en)? 2'd1 : //NMI
                                                 2'd3): //interrupt/except
                          (CODE_MRET|CODE_JALR)? 2'd3 : //MRET , x + i
                                     (BRANCH_F)? 2'd2 : //branch false(Others_xd,pc + 4)
                                                 2'd0 ; //branch true(pc + b) , jal(pc + j) , pc + 4

	//assign pc_calc_transfer = pc + ( (CODE_BRCH)? B_imm[17:2] : (CODE_JAL)? J_imm[17:2] : 16'd1 );
    assign pc_add_num = (CODE_BRCH)? 2'd2 : (CODE_JAL)? 2'd0 : 2'd1 ;
    assign pc_add_sel[2] = B_imm[17:2];
    assign pc_add_sel[0] = J_imm[17:2];
    assign pc_add_sel[1] = 16'd1;
    assign pc_add = pc_add_sel[pc_add_num];
	assign pc_calc_transfer = pc + pc_add;

	assign sel_p[3] = ALU_add_sub[17:2]; /*mtvec,MRET,JALR*/
	assign sel_p[2] = Others_xd[17:2]; /* branch false */
	assign sel_p[1] = NMI_VECTOR[17:2];
	assign sel_p[0] = pc_calc_transfer; /*Branch true , Jal , pc+4*/
	assign pc_calc = sel_p[sel_pc];

	assign exec_op =  we_reg | we_csr | dwe | ((stg == 'd1)&(CODE_FENCE|CODE_FENCEI)) | ((stg == 'd2)&(CODE_BRCH|CODE_MRET)) ;

	always @( posedge clk or negedge reset ) begin
		if( ~reset )
			pc <= RESET_VECTOR;
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
				stg <= ( ((stg<'d4)&Jump_e&(~exec_op))? 3'd4 : stg ) + ( ( (~Jump_e)&(stillshift|fsft) ) ? 3'd0 : 3'd1 );
		end
	end

	/*CSR*/
	//reg [31:0] mscratch;
	//reg  [7:0] Ecode;
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


/* main memory */
module EXT_RAM( input wire [31:0] d2, addr1 , addr2 , input wire clk, we, reset , 
																		output wire [31:0] q1 , q2 );
	reg [31:0] md1,md2;
	wire [31:0] din2;
	wire we2;

//	wire [5:0] ad1,ad2;
//	reg [31:0] mem[0:32];
//	assign ad1 = addr1[7:2];
//	assign ad2 = addr2[7:2];

	wire [9:0] ad1,ad2;
	reg [31:0] mem[0:1023];
	assign ad1 = addr1[11:2];
	assign ad2 = addr2[11:2];

	assign q1 = md1;
	assign q2 = md2;
	assign we2 = we;
	assign din2 = d2;
	
	always @(posedge clk) begin
		md1 <= mem[ ad1 ];
		md2 <= mem[ ad2 ];
		if( we2 )begin
			mem[ ad2 ] <= din2;
		end
	end

	//initialize
 	initial begin
mem[0] = { 12'd72 , 5'd0 , 3'b000 , 5'd2 , 7'b0010011  }; // ADDI　即値　加算
mem[1] = { 12'h305 , 5'd2 , 3'b001 , 5'd0 , 7'b1110011  }; // CSRRW　レジスタ　CSR　スワップ
mem[2] = { 12'd1 , 5'd0 , 3'b000 , 5'd5 , 7'b0010011  }; // ADDI　即値　加算
mem[3] = { 12'd2 , 5'd0 , 3'b000 , 5'd4 , 7'b0010011  }; // ADDI　即値　加算
mem[4] = { 12'd0 , 5'd0 , 3'b000 , 5'd3 , 7'b0010011  }; // ADDI　即値　加算
mem[5] = { 12'd0 , 5'd0 , 3'b000 , 5'd2 , 7'b0010011  }; // ADDI　即値　加算
mem[6] = { 12'h300 , 5'b01000 , 3'b110 , 5'd0 , 7'b1110011  }; // CSRRSI　即値　CSR　ビットセット
mem[7] = { 20'd1 , 5'd1 , 7'b0110111  }; // LUI
mem[8] = { 7'b0000000 , 5'd1 , 5'd1 , 3'b101 , 5'd1 , 7'b0010011  }; // SRLI　即値　論理右シフト
mem[9] = { 12'h304 , 5'd1 , 3'b010 , 5'd0 , 7'b1110011  }; // CSRRS　レジスタ　CSR　ビットセット
mem[10] = { 20'd16 , 5'd1 , 7'b0110111  }; // LUI
mem[11] = { 12'd56 , 5'd0 , 3'b010 , 5'd9 , 7'b0000011  }; // LW　ロード　ワード
mem[12] = { 7'b0000000 , 5'd9 , 5'd1 , 3'b010 , 5'b00000 , 7'b0100011  }; // SW
mem[13] = { 12'd3 , 5'd0 , 3'b000 , 5'd9 , 7'b0010011  }; // ADDI　即値　加算
mem[14] = { 7'b0000000 , 5'd9 , 5'd1 , 3'b010 , 5'b01000 , 7'b0100011  }; // SW
mem[15] = { 7'b0000000 , 5'd0 , 5'd0 , 3'b000 , 5'b00000 , 7'b1100011  }; // BEQ
mem[16] = { 32'd27000  }; // 定数
mem[17] = { 32'd0  }; // 
mem[18] = { 12'd7 , 5'd0 , 3'b000 , 5'd9 , 7'b0010011  }; // ADDI　即値　加算
mem[19] = { 7'b0000000 , 5'd9 , 5'd1 , 3'b010 , 5'b01000 , 7'b0100011  }; // SW
mem[20] = { 7'b0000000 , 5'd1 , 5'd2 , 3'b001 , 5'd2 , 7'b0010011  }; // SLLI　即値　論理左シフト
mem[21] = { 12'd20 , 5'd1 , 3'b100 , 5'd9 , 7'b0000011  }; // LBU　ロード　非負バイト
mem[22] = { 12'd1 , 5'd9 , 3'b111 , 5'd9 , 7'b0010011  }; // ANDI　即値　論理積
mem[23] = { 7'b0000000 , 5'd9 , 5'd2 , 3'b110 , 5'd2 , 7'b0110011  }; // ORレジスタ　論理和
mem[24] = { 12'd3 , 5'd2 , 3'b111 , 5'd2 , 7'b0010011  }; // ANDI　即値　論理積
mem[25] = { 7'b0000000 , 5'd4 , 5'd2 , 3'b001 , 5'b10100 , 7'b1100011  }; // BNE
mem[26] = { 12'd1 , 5'd5 , 3'b000 , 5'd5 , 7'b0010011  }; // ADDI　即値　加算
mem[27] = { 12'd7 , 5'd5 , 3'b111 , 5'd5 , 7'b0010011  }; // ANDI　即値　論理積
mem[28] = { 12'd7 , 5'd0 , 3'b000 , 5'd9 , 7'b0010011  }; // ADDI　即値　加算
mem[29] = { 7'b0000000 , 5'd9 , 5'd1 , 3'b000 , 5'b11000 , 7'b0100011  }; // SB
mem[30] = { 12'd1 , 5'd3 , 3'b000 , 5'd3 , 7'b0010011  }; // ADDI　即値　加算
mem[31] = { 12'd511 , 5'd3 , 3'b111 , 5'd3 , 7'b0010011  }; // ANDI　即値　論理積
mem[32] = { 7'b0000000 , 5'd3 , 5'd0 , 3'b001 , 5'b10000 , 7'b1100011  }; // BNE
mem[33] = { 12'd24 , 5'd1 , 3'b100 , 5'd9 , 7'b0000011  }; // LBU　ロード　非負バイト
mem[34] = { 7'b0000000 , 5'd5 , 5'd9 , 3'b100 , 5'd9 , 7'b0110011  }; // XORレジスタ　排他的論理和
mem[35] = { 7'b0000000 , 5'd9 , 5'd1 , 3'b000 , 5'b11000 , 7'b0100011  }; // SB
mem[36] = { 12'b00_11_00000_010 , 5'd0 , 3'b000 , 5'd0 , 7'b1110011  }; // MRET

	//	$readmemh("./test.hex", mem ); //program read from hex file
	end 
	
endmodule

/* unified peripheral */
module Super_IO ( input wire clk,reset ,  input wire we , input wire [31:0] addr,
						input wire [31:0] indata , output reg [31:0] outdata , 
						output wire timer_out , timer_int ,
						input wire sw1,sw2,
						output wire [2:0] RGB_LED,
						output wire [20:0] LCD
						);
parameter WIDTH = 16;
parameter PRIOD_WIDTH = 15;
// tang nano 1k = 27MHz
	reg [WIDTH-1:0] tmr_reg;
	reg [WIDTH-1:0] rld_reg;
	reg tmr_en,tmr_IE,tmr_int,tmr_tgl;
	wire tmr_int_rst = we&(~addr[4])&addr[3]&(~addr[2])&indata[2];
	wire tmr_reload = tmr_reg == { WIDTH{1'b0} };
	
	reg [17:0] LCD_bit;
	wire [2:0] LCD_com = 3'd0;
	reg [PRIOD_WIDTH-1:0] LCD_tgl_period;
	reg [2:0] LED_out;

	/* LCD driver */
	always @(posedge clk or negedge reset ) begin
		if( ~reset ) begin
			LCD_tgl_period <= {(PRIOD_WIDTH){1'b0}};
		end
		else begin
			LCD_tgl_period <= LCD_tgl_period + { {(PRIOD_WIDTH-1){1'b0}}, {1'b1} };
		end
	end
	assign LCD = { LCD_com , LCD_bit } ^ { 21{LCD_tgl_period[PRIOD_WIDTH-1]} };

	assign RGB_LED = LED_out;// LED output

	/* peripheral's register */
	always @(*) begin
		case( addr[4:2] )
			3'd0 : outdata <= { {(32-WIDTH){1'bx}} , rld_reg };  // timer reload register
			3'd1 : outdata <= { {(32-WIDTH){1'bx}} , tmr_reg };  //　timer count register
			3'd2 : outdata <= { {(32-2){1'bx}} , {  tmr_IE , tmr_en  } }; // timer control register (timer interrupt enable bit, timer start bit)
			3'd3 : outdata <= { {(32-2){1'bx}} , { tmr_tgl , tmr_int } }; // timer status register

			3'd4 : outdata <= { {(32-18){1'bx}} , { LCD_bit } };  // LCD pattern bit
			3'd5 : outdata <= { {(32-2){1'bx}} , { sw2 , sw1 } }; // GPIO input  (switch input)
			3'd6 : outdata <= { {(32-3){1'bx}} , { LED_out } };   // GPIO output (RGB LED)
			default : outdata <= 32'dx;
		endcase
	end
	
	always @( posedge clk or negedge reset ) begin
		if( ~reset ) begin
			rld_reg <= { WIDTH{1'b1} };
			tmr_en <= 1'b0;
			tmr_IE <= 1'b0;
			LCD_bit <= 18'd0;
			LED_out <= 3'b111;
		end
		else begin
			if( we ) begin
				case ( addr[4:2] )
					3'd0 : rld_reg <= indata[WIDTH-1:0];
					3'd2 : {tmr_IE , tmr_en} <= indata[1:0];
					3'd4 : LCD_bit <= indata[17:0];
					3'd6 : LED_out <= indata[2:0];
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
			if( (div_count == 8'd0) || we ) div_count <= UART_DIV;
			else div_count <= div_count - 8'd1;
		end
	end
	
	always @( posedge clk or negedge rst ) begin // send data
		if( ~rst ) begin
			tx_sreg <= 10'b11_1111_1111;
		end
		else begin
			if( we ) tx_sreg <= { 1'b0 , tx_data , 1'b1 };
			else if( div_count == 8'd0 ) tx_sreg <= { tx_sreg[8:0] , 1'b1 };
		end
	end
	
	assign tx_serial = tx_sreg[9];

	always @(posedge clk or negedge rst ) begin // counting how many bits send
		if( ~rst ) begin
			send_count <= 4'd0;
		end
		else begin
			if( we ) send_count <= 4'd10;
			else if( (div_count == 8'd0) && (send_count > 4'd0) ) send_count <= send_count - 4'd1;
		end
	end
	
	assign tx_empty = (send_count == 5'd0);

endmodule

/* Simple UART reciev data */
module suart_rx( input wire clk , rst , rd , rx_serial, output reg [7:0] rx_data , output reg valid_data );
	parameter UART_DIV = 9'd234; //(27MHz/115200boud)
	parameter UART_FST = UART_DIV + (UART_DIV >> 1);

	reg [8:0] div_count;
	reg [3:0] rcv_count;
	reg [2:0] serial_in;
	reg       rcv_edge;

	always @( posedge clk or negedge rst ) begin // serial signal anti metastable and detect start bit
		if( ~rst ) serial_in <= {3{rx_serial}};
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
			if( serial_in[2:1] == 2'b10 && rcv_count == 4'd0 ) rcv_count <= 4'd8;
			else if( div_count == 8'd0 && rcv_count > 4'd0  ) rcv_count <= rcv_count - 4'd1;
		end
	end
	
	always @( posedge clk or negedge rst ) begin // data reading
		if( ~rst ) begin
			rx_data <= 8'd0;
		end
		else begin
			if( div_count == 8'd0 && rcv_count > 4'd0  ) rx_data <= { rx_data[6:0] , serial_in[1] };
		end
	end
	
	always @( posedge clk or negedge rst ) begin // detect lsb bit
		if( ~rst ) begin
			rcv_edge <= 1'b0;
		end
		else begin
			rcv_edge <= (rcv_count == 4'd0);
		end
	end
	
	always @( posedge clk or negedge rst ) begin // notice reading data is finished
		if( ~rst ) begin
		 valid_data <= 1'b0;
		end
		else begin
			if( (rcv_count == 4'd0) && (rcv_edge == 1'b0) ) valid_data <= 1'b1;
			else if( rd ) valid_data <= 1'b0;
		end
	end
	
endmodule


/* data transfer control amang cpu,memory,peripherals */
module bus_master( input wire [31:0] dataFp1,dataFp2 ,addrbus, dataFromCpu ,
						 output wire [31:0] data2cpu , data2pri ,
						 input wire [1:0] bw  ,input wire we, output wire cs1,cs2 );
// addr : 32'h0000_0000 - 32'h0000_0fff as RAM
// addr : 32'h0001_0000 - 32'h0001_001f as Super_IO

	reg  [31:0] data_choice;
	wire [7:0] qq2[3:0];
	wire [7:0] dd2[3:0];

	wire chk1 = (addrbus[31:12] == 20'd0); // main memory
	wire chk2 = (addrbus[31:5]  == 27'h000_0800); // unified peripheral

	assign {qq2[3],qq2[2],qq2[1],qq2[0]} = data_choice;
	assign data2cpu[7:0] = qq2[addrbus[1:0]];
	assign data2cpu[15:8] = addrbus[1] ? qq2[3] : qq2[1];
	assign data2cpu[31:16] = {qq2[3],qq2[2]};
 	
	assign { dd2[3],dd2[2],dd2[1],dd2[0] } = dataFromCpu;
	assign data2pri = wdata( bw , addrbus[1:0] , dd2[0],dd2[1],dd2[2],dd2[3], qq2[0],qq2[1],qq2[2],qq2[3] );

	function [31:0] wdata ( input [1:0] acc_width , addr10 , input [7:0] idd0,idd1,idd2,idd3 , iqq0,iqq1,iqq2,iqq3 );
		case( acc_width )
			2'd0 : begin
					case(addr10)
						2'd0 : wdata = { iqq3,iqq2,iqq1,idd0 } ;
						2'd1 : wdata = { iqq3,iqq2,idd0,iqq0 } ;
						2'd2 : wdata = { iqq3,idd0,iqq1,iqq0 } ;
						2'd3 : wdata = { idd0,iqq2,iqq1,iqq0 } ;
					endcase
				end
			2'd1 : begin 
				case(addr10[1])
						1'd0 : wdata = { iqq3,iqq2,idd1,idd0 } ;
						1'd1 : wdata = { idd1,idd0,iqq1,iqq0 } ;
					endcase
				end
			2'd2 : wdata = { idd3,idd2,idd1,idd0 };
			2'd3 : wdata = { iqq3,iqq2,iqq1,iqq0 };
		endcase
	endfunction

	assign cs1 = we & chk1;
	assign cs2 = we & chk2;
	
	always @(*) begin
		case( { chk2,chk1 } )
			2'b01 : data_choice <= dataFp1;
			2'b10 : data_choice <= dataFp2;
			default : data_choice <= 32'dx;
		endcase
	end
endmodule

/* top module of minimum risc-v system */
module Soc( input wire clock , reset , sw1 ,
				output wire [2:0] FullColor_LED
                /*,output wire [20:0] LCD_out*/ );

	wire	[31:0]	read_data1,read_data2;
	wire				we1,we2,INT_S;
	wire	[31:0]	inst_addr,inst_data;
	wire	[31:0]	data_addr,read_data,write_data,pri_data;
	wire				write_enable;
	wire	[1:0]		data_width;
	wire	[14:0]	opecode;

    wire    tgl_out;
    wire    [2:0] LED_wrapper;

    assign  FullColor_LED[2:0] = LED_wrapper[2:0];
				
	riscv32core_rv32i cpu1( .reset(reset), .clk(clock) , .NMI_S(1'b0) , .INT_S(INT_S) , 
			  .inst_addr(inst_addr) , .inst_data(inst_data) ,  
			   .data_addr(data_addr) , .in_data(read_data) , .out_data(write_data) ,
			  .dwe(write_enable) , .RDorWR() , .dcs() , .Awidth(data_width),
			  .outcode(opecode) );
			  
	bus_master bm1( .addrbus(data_addr) , .data2pri(pri_data) , .dataFromCpu( write_data ) ,
						 .data2cpu(read_data), .dataFp1(read_data1), .dataFp2(read_data2), 
						 .bw(data_width), .we(write_enable), .cs1(we1), .cs2(we2) );
						 
	EXT_RAM ram1(	.clk(clock) , .reset(reset) ,
						.addr1(inst_addr) ,
						.q1(inst_data) ,
						.addr2(data_addr) , 
						.d2(pri_data),
						.q2(read_data1),
						.we(we1)  );

	Super_IO pripheral1(	.clk(clock),
								.reset(reset) ,
								.addr(data_addr) ,
								.indata(pri_data) ,
								.outdata(read_data2) , 
								.we(we2) ,
								.timer_out(tgl_out) , .timer_int(INT_S) ,
								.sw1(sw1),.sw2(1'b0),
								.RGB_LED(LED_wrapper),
								.LCD(/*LCD_out*/)	);
endmodule
