/*
	main mmemory module.  
	 System Verilog version
*/


/* main memory */
module EXT_RAM( input wire [31:0] d22, addr1 , addr2 , 
                input wire clk, we2, reset , 
                input wire [3:0] wstrb,
				    output reg [31:0] q1 , q2 );

	parameter INST_ADDR_WIDTH = 10;
	parameter RAM_SIZE = 4;
	parameter MEMORY_TYPE = 1;
	parameter LOAD_HEX_FILE = "test.hex" ;
	localparam MEMORY_CAPA = ((RAM_SIZE*1024/4) - 1);

	wire [INST_ADDR_WIDTH-1:0] ad1,ad2;

	assign ad1 = addr1[2 +: INST_ADDR_WIDTH];
	assign ad2 = addr2[2 +: INST_ADDR_WIDTH];

	integer byte_no;
	genvar byte_no2;
	generate 
		if( (MEMORY_TYPE == 1)||(MEMORY_TYPE == 3) ) begin // Quad Block Memory (System Verilog)
			reg [3:0][7:0] mem[0:MEMORY_CAPA];
			always @(posedge clk) begin
				q1 <= mem[ ad1 ];
				q2 <= mem[ ad2 ];
				if( we2&wstrb[0] ) mem[ ad2 ][0] <= d22[ 0 +: 8];
				if( we2&wstrb[1] ) mem[ ad2 ][1] <= d22[ 8 +: 8];
				if( we2&wstrb[2] ) mem[ ad2 ][2] <= d22[16 +: 8];
				if( we2&wstrb[3] ) mem[ ad2 ][3] <= d22[24 +: 8];
			end

			initial begin  //memory initialize
				$readmemh( LOAD_HEX_FILE , mem ); //program read from hex file 
			end //memory initial end
	

		end else if( MEMORY_TYPE == 2 ) begin //Quad divided meory using Verilog expression
			reg [31:0] mem[0:MEMORY_CAPA];
			always @(posedge clk) begin
				q1 <= mem[ ad1 ];
				for( byte_no = 0 ; byte_no < 4 ; byte_no = byte_no + 1 ) begin
					if( ~(we2&wstrb[byte_no]) ) q2[ 8*byte_no +: 8 ] <= mem[ ad2 ][8*byte_no +: 8];
					if( we2&wstrb[byte_no] ) mem[ ad2 ][ 8*byte_no +: 8 ] <= d22[ 8*byte_no +: 8 ];
				end
			end

			initial begin  //memory initialize
				$readmemh( LOAD_HEX_FILE , mem ); //program read from hex file 
			end //memory initial end */

	
		end else begin // any verilog can compile
			reg [31:0] mem[0:MEMORY_CAPA];
			wire [7:0] din2[0:3];
			wire [1:0] wea;
			assign wea[0] = we2&(wstrb[1] | wstrb[0]);
			assign wea[1] = we2&(wstrb[3] | wstrb[2]);

			for( byte_no2 = 0 ; byte_no2 < 4 ; byte_no2 = byte_no2 + 1 ) begin : mem_choice
				assign din2[byte_no2] =   (wstrb[byte_no2]) ? d22[8*byte_no2 +: 8] : q2[8*byte_no2 +: 8];
			end
			
			always @(posedge clk) begin
				q1 <= mem[ ad1 ];
				if( ~wea[0] ) q2[ 0 +: 16] <= mem[ ad2 ][ 0 +: 16];
				if( ~wea[1] ) q2[16 +: 16] <= mem[ ad2 ][16 +: 16];
				if( wea[0] ) mem[ ad2 ][ 0 +: 16] <= { din2[1] , din2[0] };
				if( wea[1] ) mem[ ad2 ][16 +: 16] <= { din2[3] , din2[2] };
			end

			initial begin  //memory initialize
				$readmemh( LOAD_HEX_FILE , mem ); //program read from hex file 
			end //memory initial end */
	
		end
	endgenerate

endmodule 

