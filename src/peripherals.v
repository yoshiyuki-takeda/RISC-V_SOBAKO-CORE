
/* peripherals:
	Clock divider
   PWM
	PDM
	Counter
	Timer
	WDT
	RANDOM
	UART
	SPI
	I2C
	GPIO	*/
	
`default_nettype none

/* simple GPIO input */
module SimpleInput
		#( parameter PORT_WIDTH = 16 , DATA_WIDTH = 32 )
		( input wire [PORT_WIDTH-1:0] in_port , output wire [DATA_WIDTH-1:0] rd_data );

		assign rd_data = { {(DATA_WIDTH-PORT_WIDTH){1'dx}} , in_port };
endmodule

/* simple GPIO Output */
module SimpleOutput
		#( parameter PORT_WIDTH = 16 , DATA_WIDTH = 32 )
		( input wire clk , rst , cs, wr,	output reg [PORT_WIDTH-1:0] out_port ,
		  output wire [DATA_WIDTH-1:0] rd_data, input wire [DATA_WIDTH-1:0] wr_data );
	
	assign rd_data = { {(DATA_WIDTH-PORT_WIDTH){1'dx}} , out_port };

	always @(posedge clk or negedge rst) begin
		if( ~rst ) out_port <= 0;
		else if( cs&wr ) out_port <= wr_data[PORT_WIDTH-1:0];
	end
endmodule


/* Simple clock ticker */
module Trig_Generator
		#(parameter DIV_WIDTH = 8)
		( input wire clk,rst, input wire [DIV_WIDTH-1:0] div , output wire clktick );
	reg [DIV_WIDTH-1:0] counter;
	
	always @( posedge clk or negedge rst ) begin
		if( !rst ) begin
			counter <= 0;
		end
		else begin
			 if( clktick ) counter <= 0;
			 else counter <= counter + 1 ;
		end
	end
	
	assign clktick = (counter >= div);
	
endmodule

/* Simple Pulse Width Modulation */
module PWM_A 
		#( parameter PWM_WIDTH = 16 )
		( input wire clk, rst , tick , pwm_en , input wire [PWM_WIDTH-1:0] duty , cycle , output wire pwm );
	reg [PWM_WIDTH-1:0] pwm_counter;
	reg pwm_lock;
	
	assign pwm = pwm_en & ~pwm_lock;
	
	always @( posedge clk or negedge rst ) begin
		if( !rst ) begin
			pwm_counter <= 'd0;
			pwm_lock <= 1'b0;
		end
		else begin
			if( pwm_en ) begin
				if( tick ) begin
					if( pwm_counter >= cycle ) begin
						pwm_counter <= 'd0;
						if( duty != 'd0 ) pwm_lock <= 1'b0;
					end
					else begin
						pwm_counter <= pwm_counter + 'd1;
						if( (pwm_counter == duty) && (pwm_lock == 1'b0) ) pwm_lock <= 1'b1;
					end
				end
			end
			else begin
				pwm_counter <= 'd0;
				pwm_lock <= 1'b0;
			end
		end
	end
	
endmodule

/* Simple Pulse Density Modulation */
module PDM_A
		#( parameter PDM_WIDTH = 8 ) //if you wan't any period PDM cycle, it needs modulo operation
		(input wire clk, rst , tick , pdm_en , input wire [PDM_WIDTH:0] duty , output wire pdm); 
	reg [PDM_WIDTH:0] pdm_counter;

	assign pdm = pdm_counter[PDM_WIDTH];

	always @(posedge clk or negedge rst) begin
		if( !rst ) pdm_counter <= {{1'b0},{(PDM_WIDTH){1'b1}}};
		else begin
			if( pdm_en ) if( tick ) pdm_counter <=  { 1'b0,pdm_counter[PDM_WIDTH-1:0] } + { duty };
			else pdm_counter <= {{1'b0},{(PDM_WIDTH){1'b1}}};
		end
	end

endmodule


/* Simple counter */
module Counter_A
		#( parameter COUNTER_WIDTH = 16 )
		( input wire clk , rst , tick , sig_in , count_en , rise_en, fall_en, rise_IE , fall_IE , ovf_IE , clr_flag , clr_ovf_flag ,
		  output reg [COUNTER_WIDTH-1:0] capture , output wire INT , output reg rise_flag , fall_flag , ovf_flag );
	reg [2:0] ams;
	reg [COUNTER_WIDTH:0] counter;
	wire rise,fall;

	always @( posedge clk or negedge rst ) begin
		if( !rst ) ams <= 3'd0;
		else ams <= { ams[1:0] ,sig_in };
	end

	assign fall = (ams[2:1] == 2'b10) & fall_en ;
	assign rise	= (ams[2:1] == 2'b01) & rise_en ;
	assign INT = |{ {rise_IE , fall_IE , ovf_IE} & {rise_flag , fall_flag , ovf_flag} };
	
	always @( posedge clk or negedge rst ) begin
		if( !rst ) begin
			{ counter , capture , rise_flag , fall_flag , ovf_flag } <= 'd0;
		end
		else begin
			if( count_en ) begin
				counter <=  (tick) ? { 1'b0 , counter } + 'd1 : counter ;
				capture <= ( rise | fall ) ? counter[COUNTER_WIDTH-1:0] : capture;
				rise_flag <= ( rise ) ? 'd1 : (clr_flag) ? 'd0 : rise_flag ; 
				fall_flag <= ( fall ) ? 'd1 : (clr_flag) ? 'd0 : fall_flag ;
				ovf_flag <= ( tick & counter[COUNTER_WIDTH] ) ? 'd1 : (clr_ovf_flag) ? 'd0 : ovf_flag ;
			end
			else begin
				{ counter , rise_flag , fall_flag , ovf_flag } <= 'd0;
			end
		end
	end
	
endmodule


/* simple timer */
module Timer_A
			#(parameter TMR_WIDTH = 16)
			( input wire clk, rst, tick, tmr_en, tmr_IE, tmr_IE_clr, 
			  input wire [TMR_WIDTH-1:0] tmr_val, output wire INT, flag, tmr_clk );
	reg [TMR_WIDTH-1:0] tmr_reg;
	reg tmr_int,tmr_tgl,first;
	
	assign tmr_clk = tmr_tgl;
	assign INT = tmr_IE & tmr_int;
	assign flag = tmr_int;
	
	always @( posedge clk or negedge rst ) begin
		if( !rst )
			{tmr_reg ,tmr_int, tmr_tgl , first } <= 0;
		else begin
			if( tmr_en ) begin
				if( tick ) begin
					if( tmr_reg == 0 ) begin
						tmr_reg <= tmr_val;
						tmr_tgl <= (first) ? ~tmr_tgl : tmr_tgl ;
						first <= 1;
					end
					else 	tmr_reg <= tmr_reg - 1;
				end
				tmr_int <= ( (tmr_reg == 0) & tick & first ) ? 1'b1 : (tmr_IE_clr) ? 1'b0 : tmr_int ;
			end
			else {tmr_reg ,tmr_int, tmr_tgl , first } <= 0;
		end
	end
	
endmodule


/* Small Watch Dog Timer */
module WatchDogTimer_A
			#( parameter TIMER_WIDTH = 16 )
			(input wire clk, rst, tick, timer_start, WDT_rst, input wire [TIMER_WIDTH-1:0] WDT_val, 
			 output wire INT, output wire [TIMER_WIDTH-1:0] current_val );
	reg [TIMER_WIDTH-1:0] WDT_reg;
	reg WDT_int;
	
	assign current_val = WDT_reg;
	assign INT = WDT_int;
	
	always @(posedge clk or negedge rst ) begin
		if( !rst ) { WDT_reg,WDT_int } <= 0;
		else begin
			if( timer_start ) begin
				if( WDT_reg == 0 ) WDT_int <= 1;
				else if( WDT_rst ) WDT_reg <= WDT_val;
				else if( ~WDT_int & tick ) WDT_reg <= WDT_reg - 1;
			end
			else begin
				WDT_reg <= WDT_val;
				WDT_int <= 0;
			end
		end
	end
	
endmodule

/* Small Watch Dog Timer Type B */
module WatchDogTimer_B
			#( parameter TIMER_WIDTH = 16 , TIMER_VAL = 16'd60000 )
			(input wire clk, rst, timer_start, WDT_rst, output wire INT );
	reg [TIMER_WIDTH-1:0] WDT_reg;
	reg WDT_int,Start_dly,rst_dly,start_flag;
	
	assign INT = WDT_int;
	
	always @(posedge clk or negedge rst ) begin
		if( !rst ) { start_flag,WDT_int,Start_dly,rst_dly,WDT_reg } <= { 4'd0,TIMER_VAL };
		else begin
			{ Start_dly , rst_dly } <= { timer_start, WDT_rst };
			if( {timer_start,Start_dly} == 2'b10 ) start_flag <= 1;
			if( start_flag ) begin
				if( WDT_reg == 0 ) WDT_int <= 1;
				else if( {WDT_rst,rst_dly} == 2'b10 ) WDT_reg <= TIMER_VAL;
				else if( ~WDT_int ) WDT_reg <= WDT_reg - 1;
			end
		end
	end
	
endmodule

/* pseudorandom numbers */
module Random_A (input wire clk, rst, we, rd, input wire [15:0] rand_init, output reg [15:0] random );
		always @( posedge clk or negedge rst ) begin
			if( !rst ) random <= 'haa55;
			else random <= (rd)? { random[14:0] , random[15] ^ random[13] ^ random[12] ^ random[10] } : (we)? rand_init : random ;
		end
endmodule

/* simple single port RAM */
module SimpleSinglePortRAM
			#( parameter RAM_DEPTH = 6 , parameter RAM_WIDTH = 8 )
			( input wire clk, cs, wr, rd, output reg [RAM_WIDTH-1:0] rd_data, input wire [RAM_WIDTH-1:0] wr_data, input wire[RAM_DEPTH-1:0] addr);
	reg [RAM_WIDTH-1:0] mem[0:2**RAM_DEPTH-1];
	
	always @( posedge clk ) begin
		rd_data <= mem[addr];
		if(cs&wr) mem[addr] <= wr_data;
	end
endmodule

/* simple dual port RAM */
module SimpleDualPortRAM
			#( parameter RAM_DEPTH = 6 , parameter RAM_WIDTH = 8 )
			( input wire clk1,clk2, cs1,cs2, wr1,wr2, rd1,rd2, input wire[RAM_DEPTH-1:0] addr1,addr2 ,
			  output reg [RAM_WIDTH-1:0] rd_data1,rd_data2, input wire [RAM_WIDTH-1:0] wr_data1,wr_data2 );
	reg [RAM_WIDTH-1:0] mem[0:2**RAM_DEPTH-1];
	
	always @( posedge clk1 ) begin
		rd_data1 <= mem[addr1];
		if(cs1&wr1) mem[addr1] <= wr_data1;
	end
	always @( posedge clk2 ) begin
		rd_data2 <= mem[addr2];
		if(cs2&wr2) mem[addr2] <= wr_data2;
	end
endmodule

/* Simple Synchronus FIFO with RAM */
module SimpleFIFO
			#(parameter FIFO_DEPTH = 6 , parameter FIFO_DATA_WIDTH = 9)
			( input wire clk, rst, wr, rd, clr, output wire [FIFO_DEPTH:0] fifo_gauge , output wire full, empty,
			  input wire [FIFO_DATA_WIDTH-1:0] in_data , output reg [FIFO_DATA_WIDTH-1:0] out_data );
	reg [FIFO_DATA_WIDTH-1:0] mem[0:2**FIFO_DEPTH-1];
	reg [FIFO_DEPTH:0] rd_addr,wr_addr;
	reg empty_shift;
	wire  current_enpty;

	assign fifo_gauge = wr_addr - rd_addr;
	assign current_enpty = (fifo_gauge == 0);
	assign empty = current_enpty | empty_shift;
	assign full = fifo_gauge[FIFO_DEPTH];//(fifo_gauge == (2**FIFO_DEPTH-1));

	always @( posedge clk ) begin
		empty_shift <= current_enpty;
	end
	always @( posedge clk or negedge rst or posedge clr ) begin
		if( ~rst | clr) { rd_addr , wr_addr } <= 0;
		else begin
			rd_addr <= (~empty&rd) ? rd_addr + 1 : rd_addr ;
			wr_addr <= ( ~full&wr) ? wr_addr + 1 : wr_addr ;
			out_data <= mem[rd_addr[FIFO_DEPTH-1:0]];
			if( ~full&wr ) mem[wr_addr[FIFO_DEPTH-1:0]] <= in_data;
		end
	end
	
	integer k1;
	initial begin
		for ( k1=0 ; k1 < 2**FIFO_DEPTH ; k1 = k1 + 1 ) 
			mem[k1] = 0;
	end
endmodule

/* Simple Asynchronus FIFO with RAM */
module SimpleFIFO_Async
			#(parameter FIFO_DEPTH = 6 , parameter FIFO_DATA_WIDTH = 9)
			( input wire rst, clr, i_clk, i_wr, o_clk, o_rd,  output wire [FIFO_DEPTH:0] i_fifo_gauge, o_fifo_gauge, output wire full, empty,
			  input wire [FIFO_DATA_WIDTH-1:0] i_data , output reg [FIFO_DATA_WIDTH-1:0] o_data );
	reg [FIFO_DATA_WIDTH-1:0] mem[0:2**FIFO_DEPTH-1];
	reg [FIFO_DEPTH:0] rd_addr,rd_gray1,rd_gray2 , wr_addr,wr_gray1,wr_gray2;
	wire [FIFO_DEPTH:0] rd_gray,rd_degray , wr_gray,wr_degray;

	assign o_fifo_gauge = wr_degray - rd_addr; // o_clk 
	assign empty = (o_fifo_gauge == 0);
	assign rd_gray = { rd_addr[FIFO_DEPTH] , rd_addr[FIFO_DEPTH-1:0]^rd_addr[FIFO_DEPTH:1] };
	assign wr_degray = { wr_gray2[FIFO_DEPTH] , wr_gray2[FIFO_DEPTH-1:0]^wr_degray[FIFO_DEPTH:1] }; // o_clk domain
	assign i_fifo_gauge = wr_addr - rd_degray;    // i_clk domain
	assign full = i_fifo_gauge[FIFO_DEPTH];
	assign wr_gray = { wr_addr[FIFO_DEPTH] , wr_addr[FIFO_DEPTH-1:0]^wr_addr[FIFO_DEPTH:1] };
	assign rd_degray = { rd_gray2[FIFO_DEPTH] , rd_gray2[FIFO_DEPTH-1:0]^rd_degray[FIFO_DEPTH:1] };  // i_clk domain -->

	always @( posedge o_clk or negedge rst or posedge clr ) begin
		if( ~rst | clr ) { rd_addr,wr_gray2,wr_gray1 } <= 0;
		else begin
			{wr_gray2,wr_gray1} <= {wr_gray1,wr_gray};
			rd_addr <= (~empty&o_rd) ? rd_addr + 1 : rd_addr ;
			o_data <= mem[rd_addr[FIFO_DEPTH-1:0]];
		end
	end
	always @( posedge i_clk or negedge rst or posedge clr ) begin
		if( ~rst | clr ) { wr_addr,rd_gray2,rd_gray1 } <= 0;
		else begin
			{rd_gray2,rd_gray1} <= {rd_gray1,rd_gray};
			wr_addr <= ( ~full&i_wr) ? wr_addr + 1 : wr_addr ;
			if( ~full&i_wr ) mem[wr_addr[FIFO_DEPTH-1:0]] <= i_data;
		end
	end

	integer k1;
	initial begin
		for ( k1=0 ; k1 < 2**FIFO_DEPTH ; k1 = k1 + 1 ) 
			mem[k1] = 0;
	end
endmodule

/* Simple UART send data */
module UART_TX_A
			#( parameter UART_DIV_WIDTH = 8 , FIFO_SIZE = 6 ) //parameter UART_DIV = 8'd234; //(27MHz/115200boud)
			( input wire [7:0] tx_data, input wire [UART_DIV_WIDTH-1:0] UART_DIV,
			  input wire tx_int_en, clk, rst, we, clr, parity/*0:non,1:add*/, size/*0:7bit,1:8bit*/, 
			  stopbit/*0:1bit,1:2bit*/, oddeven/*0:even,1:odd*/, auto_flow/*0:manual,1:auto*/, cts, 
			  output wire tx_full, tx_serial, tx_int );
	reg [8:0] tx_sreg; // send data
	reg [UART_DIV_WIDTH-1:0] div_count;  // uart send timing tick
	reg [3:0] send_count; // counting how many bits send
	reg  parity_bit;
	wire buf_enpty,data_set,tx_active,div_count0;
	wire [7:0] tdata;
	wire parity_point = parity & ((4'd2 + {3'd0,stopbit}) == send_count) ;
	
	assign tx_serial = tx_sreg[0];
	assign tx_active = (send_count > 0);
	assign div_count0 = (div_count==0);
	assign data_set = ~buf_enpty && ~tx_active && ~(cts&auto_flow) && div_count0;
	assign tx_int = ~tx_full & tx_int_en;

	always @( posedge clk or negedge rst ) begin 
		if( ~rst ) begin
			div_count <= 0;
			send_count <= 0;
			tx_sreg <= 9'b1_1111_1111;
			parity_bit <= 1'b0;
		end
		else begin
			if( data_set )begin 
				div_count <= UART_DIV;
				parity_bit <= 1'b0;
				send_count <= { 1'b1 , {2'd0,parity} + {2'd0,size} + {2'd0,stopbit} } ;
				tx_sreg <= { (size) ? tdata[7]:1'b1 , tdata[6:0] , 1'b0 };
			end
			if( (~div_count0)|tx_active ) begin
				if(div_count0)begin
					div_count <= UART_DIV;
					parity_bit <= parity_bit ^ tx_sreg[1];
					send_count <= send_count - 1;
					tx_sreg <= { 1'b1 , tx_sreg[8:2] , (parity_point) ? oddeven^parity_bit : tx_sreg[1] };
				end
				else div_count <= div_count - 1;
			end
		end
	end
	
	SimpleFIFO #( .FIFO_DEPTH(FIFO_SIZE), .FIFO_DATA_WIDTH(8) ) TX_FIFO 
			(  .clk(clk), .rst(rst), .wr(we), .rd(data_set), .clr(clr), .fifo_gauge(), .full(tx_full), 
				.empty(buf_enpty), .in_data(tx_data) , .out_data(tdata) );
endmodule


/* Simple UART reciev data */
module UART_RX_A
			#( parameter UART_DIV_WIDTH = 8 , FIFO_SIZE = 6 )//parameter UART_DIV = 234; //(27MHz/115200boud)
			( input wire clk, rst, rd, clr, rx_serial, auto_flow, rx_int_ne,
			  input wire parity/*0:non,1:add*/, size/*0:7bit,1:8bit*/, stopbit/*0:1bit,1:2bit*/, oddeven/*0:even,1:odd*/, 
			  input wire [UART_DIV_WIDTH-1:0] UART_DIV, output wire [7:0] rx_buf, 
			  output wire rx_data_exist, rts, parity_err/*0:n.p.,1:err*/, rx_int );
	reg [8:0] rx_data;
	reg [UART_DIV_WIDTH-1:0] div_count;
	reg [3:0] rcv_count;
	reg [2:0] serial_in;
	reg       rcv_edge,parity_check;
	
	wire rcv_count1,rcv_count0,div_count0,rx_full,rx_empty,finalize_data,parity_err_bit;
	wire [1:0] data_width = {1'b0,parity} + {1'b0,size};

	assign rcv_count0 = (rcv_count == 4'd0);
	assign rcv_count1 = (rcv_count == ( 4'd1 /*+ { 3'd0 , stopbit }*/ ) ); // detect final data bit
	assign div_count0 = (div_count == 'd0);
	assign  rts = auto_flow & rx_full;
	assign rx_data_exist = ~rx_empty;
	assign rx_int = rx_data_exist & rx_int_ne;
	assign finalize_data = ({rcv_count1,rcv_edge} == 2'b10);
	assign parity_err_bit = parity & ( oddeven ^ parity_check );

	always @( posedge clk or negedge rst ) begin // uart read timing tick
		if( ~rst ) begin
			div_count <= 0;
			{ rcv_count , rx_data , parity_check } <= 0;
		end
		else begin
			if( (serial_in[2:1] == 2'b10) && rcv_count0 ) begin
				div_count <=  UART_DIV>>1;
				rcv_count <= { 1'b1 , 3'd1 + {2'd0,parity} + {2'd0,size} /*+ {2'd0,stopbit}*/ } ; //4'd10;
				parity_check <= 0;
			end
			if( ~rcv_count0 ) begin
				if( div_count0 ) begin
					div_count <= UART_DIV;
					rx_data <= {serial_in[1], (data_width==1)? serial_in[1]:rx_data[8], (data_width==0)? serial_in[1]:rx_data[7], rx_data[6:1]};
					parity_check <= parity_check ^ serial_in[1];
					rcv_count <= rcv_count - 1;
				end
				else div_count <= div_count - 1;
			end
		end
	end

	always @( posedge clk or negedge rst ) begin 
		{ serial_in , rcv_edge } <=  (~rst) ? 0 : { serial_in[1:0] , rx_serial , rcv_count1 };
	end

	SimpleFIFO #( .FIFO_DEPTH(FIFO_SIZE), .FIFO_DATA_WIDTH(9) ) RX_FIFO 
	           ( .clk(clk), .rst(rst), .wr(finalize_data), .rd(rd), .clr(clr), .fifo_gauge(), .full(rx_full), 
				.empty(rx_empty), .in_data( {parity_err_bit , (size)? rx_data[7] : 1'b0 , rx_data[6:0] } ) , .out_data( {parity_err,rx_buf} ) );
endmodule

/* Small UART send data */
module UART_TX_B
			#( parameter UART_DIV = 8'd234 ) //(27MHz/115200boud)
			( input wire [7:0] tx_data, input wire clk, rst, we, output wire tx_empty, tx_serial );
	reg [8:0] tx_sreg; // send data
	reg [7:0] div_count;  // uart send timing tick
	reg [3:0] send_count; // counting how many bits send
	
	assign tx_serial = tx_sreg[0];
	assign tx_empty = (send_count == 'd0);

	always @( posedge clk or negedge rst ) begin 
		if( ~rst ) begin
			div_count <= UART_DIV;
			send_count <= 'd0;
			tx_sreg <= 9'b1_1111_1111;
		end
		else begin
			if(tx_empty&we)begin 
				div_count <= UART_DIV;
				send_count <= 'd10;
				tx_sreg <= {  tx_data , 1'b0 };
			end
			else if(div_count == 8'd0)begin
				div_count <= UART_DIV;
				if(send_count > 'd0) send_count <= send_count - 'd1;
				tx_sreg <= { 1'b1 , tx_sreg[8:1] };
			end
			else begin
				div_count <= div_count - 'd1;
			end
		end
	end
endmodule


/* Small UART reciev data */
module UART_RX_B
			#( parameter UART_DIV = 8'd234 ) //(27MHz/115200boud)
			( input wire clk, rst, rd, rx_serial, output reg [7:0] buf_rx, output reg valid_data );
	reg [7:0] rx_data;
	reg [7:0] div_count;
	reg [3:0] rcv_count;
	reg [2:0] serial_in;
	reg       rcv_edge;
	
	wire rcv_count1,rcv_count0,div_count0;

	assign rcv_count0 = (rcv_count == 'd0);
	assign rcv_count1 = (rcv_count == 'd1);
	assign div_count0 = (div_count == 'd0);

	always @( posedge clk or negedge rst ) begin // uart read timing tick
		if( ~rst ) begin
			div_count <= UART_DIV;
			{rcv_count,rx_data} <= 'd0;
		end
		else begin
			if( serial_in[2:1] == 2'b10 && rcv_count0 ) begin
				div_count <=  UART_DIV>>1;
				rcv_count <= 4'd10;
			end
			else if( div_count0 ) begin
				div_count <= UART_DIV;
				rx_data <= { serial_in[1] , rx_data[7:1] };
				rcv_count <= rcv_count - {3'd0,~rcv_count0}; //if(!rcv_count0) rcv_count <= rcv_count - 'd1;
			end
			else div_count <= div_count - 'd1;
		end
	end

	always @( posedge clk or negedge rst ) begin 
		if( ~rst )
			{ rcv_edge,serial_in,buf_rx,valid_data } <= 0;
		else begin
			rcv_edge <= rcv_count1;
			serial_in <= { serial_in[1:0] , rx_serial };
			if( {rcv_count1,rcv_edge} == 2'b10 ) begin // detect lsb bit
				buf_rx <= rx_data;
				valid_data <= 1'b1;
			end
			else if( rd ) valid_data <= 1'b0;
		end
	end
endmodule


module Simple_SPI_Master_B
			#( parameter DEV_WIDTH = 5 , COUNT_WIDTH = 4 , DATA_NUM = 8 )
			( input wire clk, rst, rd, wr, SPI_MISO, CPOL, CPHA, EOT, SPI_INT_en, 
			  input wire [DATA_NUM-1:0] TransmitData, input wire [2:0] PreHold, PostHold, CSHiHold, 
			  input wire [COUNT_WIDTH-1:0] CountI, input wire [DEV_WIDTH-1:0] divider , 
			  output wire SPI_MOSI, SPI_INT, SPI_CLK, data_set,
			  output reg SPI_CS, RD_DATA_Valid, output wire [DATA_NUM-1:0] ReceiveData );
			  // CPHA 0:Sampling(Lach) first, 1: Shift first // CPOL 0:Low Idle, 1:High Idle

	reg [DATA_NUM-1:0] TR_REG , TR_RESERVE;  /* Transmitter & reciever register */
	reg [COUNT_WIDTH-1:0] TR_COUNT,TR_CNT_RSV;
	reg [DEV_WIDTH-1:0] devide;
	reg Lach_data,interclk,cs_hold,wr_pending,pre_valid;

	assign SPI_CLK = ( (TR_CNT_RSV >= TR_COUNT) & (TR_COUNT != 0) & (interclk ^ CPHA) & ~cs_hold ) ^ CPOL;
	assign SPI_MOSI = TR_REG[TR_CNT_RSV-1] | SPI_CS;
	assign ReceiveData = TR_RESERVE;
	assign SPI_INT = RD_DATA_Valid & SPI_INT_en;
	assign data_set = wr_pending;

	always @( posedge clk or negedge rst ) begin
		if( ~rst ) begin
			{TR_REG,TR_RESERVE,TR_CNT_RSV,TR_COUNT,devide} <= 0;
			{interclk,Lach_data,RD_DATA_Valid,pre_valid,cs_hold,wr_pending,SPI_CS} <= 1;
		end
		else begin
			if( pre_valid & (TR_CNT_RSV == CountI) ) begin
				TR_RESERVE <= TR_REG;
				pre_valid <= 0 ;
			end
			
			if( rd & RD_DATA_Valid ) RD_DATA_Valid <= 0;

			if( wr & ~RD_DATA_Valid ) begin 
				TR_RESERVE <= TransmitData;
				wr_pending <= 1;
			end

			if(TR_COUNT != 0) begin
				if( devide == divider )begin
					interclk <= ~interclk;
					devide <= 0;
					if( interclk ) begin
						TR_COUNT <= TR_COUNT - 1;
						if( cs_hold ) begin
							if( TR_COUNT == CSHiHold ) SPI_CS <= 1;
							if( TR_COUNT == 1 ) cs_hold <= 0;
						end
					end
					if( ~cs_hold & (TR_CNT_RSV >= TR_COUNT) ) begin
						if( interclk ) begin 
							TR_REG <= { TR_REG[DATA_NUM-2:0] , Lach_data };
							if( TR_COUNT == 1 ) {RD_DATA_Valid,pre_valid} <= 2'b11;
						end
						else Lach_data <= SPI_MISO;
					end
				end
				else devide <= devide + 1;
			end /*  */
			else begin
				if( wr_pending ) begin
					if(TR_CNT_RSV == CountI) begin
						{devide,SPI_CS,interclk,wr_pending} <= 0;
						TR_COUNT <= (SPI_CS) ? (PreHold + TR_CNT_RSV) : TR_CNT_RSV;
						TR_REG <= TR_RESERVE;
					end
					else TR_CNT_RSV <= CountI;
				end
				else if( EOT & ~SPI_CS ) begin
					{interclk,cs_hold} <= 1;
					TR_COUNT <=  PostHold;
				end
			end
		end
	end

endmodule

/* Simple Synchronus FIFO with register */
module Simple_Buffer
			#(parameter FIFO_DEPTH = 2 , parameter FIFO_DATA_WIDTH = 9)
			( input wire clk, rst, wr, rd, clr, output wire [FIFO_DEPTH-1:0] fifo_gauge , output wire full, empty,
			  input wire [FIFO_DATA_WIDTH-1:0] in_data , output wire [FIFO_DATA_WIDTH-1:0] out_data );
	reg [FIFO_DATA_WIDTH-1:0] mem[0:2**FIFO_DEPTH-1];
	reg [FIFO_DEPTH-1:0] rd_addr,wr_addr;

	assign fifo_gauge = wr_addr - rd_addr;
	assign empty = (fifo_gauge == 0);
	assign full = (~fifo_gauge == 0);
	assign out_data = mem[rd_addr];

	always @( posedge clk or negedge rst ) begin
		if( ~rst ) { rd_addr , wr_addr } <= 0;
		else if( clr ) { rd_addr , wr_addr } <= 0;
		else begin
			rd_addr <= (~empty&rd) ? rd_addr + 1 : rd_addr ;
			wr_addr <= ( ~full&wr) ? wr_addr + 1 : wr_addr ;
			if( wr ) mem[wr_addr] <= in_data;
		end
	end

	integer k2;
	initial begin
		for ( k2=0 ; k2 < 2**FIFO_DEPTH ; k2 = k2 + 1 ) 
			mem[k2] = 0;
	end

endmodule


module Simple_I2C_Master_A /*I2C suport Standard/Fast/Fast+  */
		#( parameter DIV = 270 , CLK_RISE = 135 , DATA_CHANGE = 64, DATA_LACH = 250, DIV_WIDTH = 9 )
		( input wire clk, rst, wr, rd, EOT, ForceFinish,  I2C_INT_en, I2C_SCL_in, I2C_SDA_in, 
		  output reg I2C_SCL_out, I2C_SDA_out, Valid_Rd_Data, Busy, output wire ACK, I2C_INT,
		  input wire [7:0] Transmit_Data, output wire [7:0] Recieve_Data );
	localparam Data_width = 8;
	reg [DIV_WIDTH-1:0] Clk_div_counter;
	reg [3:0] bit_count;
	reg [Data_width:0] TR_buf;
	reg [1:0] STM; // state machine
	reg Idle,EndFlag,RorW; // H:Read L:Write
	
	assign {Recieve_Data,ACK} = TR_buf;  // bit8-1:data bit0:ack // H:not ack L:ack
	assign I2C_INT = I2C_INT_en & Valid_Rd_Data;
	
	always @( posedge clk or negedge rst ) begin
		if( ~rst ) begin
			{ TR_buf,Idle,I2C_SCL_out, I2C_SDA_out } <= 'b111111111_111;
			{ Clk_div_counter,bit_count,RorW,STM,Valid_Rd_Data,EndFlag,Busy } <= 0;
		end
		else begin
			if(STM==0) begin
				if (rd) Valid_Rd_Data <= 0;
				else if( wr ) begin
					EndFlag <= EOT | ForceFinish;
					Busy <= 1;
					if( ~ForceFinish ) begin
						TR_buf <= { Transmit_Data , ~RorW | EOT };
						Clk_div_counter <= 0;
						Valid_Rd_Data <= 0;
						bit_count <= 9;
						if( Idle ) begin
							RorW <= Transmit_Data[0];
							I2C_SDA_out <= 0;
							Idle <= 0;
							STM <= 1;
						end
						else STM <= 2;
					end
				end
				else if( EndFlag | (~Idle&ACK) ) begin
					{ EndFlag,RorW,Clk_div_counter,I2C_SCL_out } <= 0;
					bit_count <= 4;
					STM <= 3;
				end
				else Busy <= 0;
			end
			
			else if(STM==1)begin
				if( Clk_div_counter == CLK_RISE ) begin
					Clk_div_counter <= 0;
					if(I2C_SCL_out) I2C_SCL_out <= 0;
					else STM <= 2;
				end
				else Clk_div_counter <= Clk_div_counter + 1;
			end // stm == 1
			
			else if(STM==2)begin
				if( bit_count != 0 ) begin
					Clk_div_counter <= ( Clk_div_counter == DIV ) ? 0 : Clk_div_counter + 1;
					if( Clk_div_counter == DATA_CHANGE ) I2C_SDA_out <= TR_buf[Data_width] ;
					else if( Clk_div_counter == CLK_RISE ) I2C_SCL_out <= 1;
					else if( Clk_div_counter == DATA_LACH ) TR_buf <= { TR_buf[Data_width-1:0],I2C_SDA_in };
					else if( Clk_div_counter == DIV ) { bit_count , I2C_SCL_out } <= { bit_count - 1 , 1'b0 };
				end
				else begin
					Valid_Rd_Data <= 1;
					STM <= 0;
				end
			end // stm == 2
			
			else if(STM==3)begin 
				Idle <= 1;
				if( bit_count != 0 ) begin
					if( Clk_div_counter == CLK_RISE ) begin
						if( bit_count == 4 ) I2C_SDA_out <= 0;
						if( bit_count == 3 ) I2C_SCL_out <= 1;
						if( bit_count == 2 ) I2C_SDA_out <= 1;
						bit_count <= bit_count - 1;
						Clk_div_counter <= 0;
					end
					else Clk_div_counter <= Clk_div_counter + 1;
				end
				else STM <= 0;
			end // stm == 3
		end
	end //always

endmodule




