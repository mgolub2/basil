/**
 * ------------------------------------------------------------
 * Copyright (c) SILAB , Physics Institute of Bonn University 
 * ------------------------------------------------------------
 * Modified By Maximilian Golub
 */

module top (
	 output wire REG_DI,
	 output wire TCP_CLOSE_ACK,
	 output wire TCP_TX_WR,             
    output wire [7:0] TCP_TX_DATA,              
	 output wire USR_CLK,   
    output wire [3:0] GPIO_LED,
	 output [7:0] cmd,
	 input USR_TX_AFULL,
	 input wire [3:0] GPIO_DIP,
	 input wire RBCP_ACT,
	 input wire TCP_RX_WR,          
	 input wire [7:0] TCP_RX_DATA,
	 input wire TCP_CLOSE_REQ,
	 input wire USER_RESET, //dipswitch
    input wire USR_CLK50M, //50Mhz in
	 input wire REG_CLK,
	 input wire REG_DO,
	 input PIXEL_SR_OUT
);

    wire CLKFBOUT, CLKOUT0, CLKOUT1, CLKOUT2, CLKOUT3, CLKOUT4, CLKFBIN, LOCKED;
    wire RST, BUS_CLK, BUS_RST, SPI_CLK;

clock clock_maker (
    .CLKIN1_IN(USR_CLK50M), 
    .RST_IN(USER_RESET), 
    .CLKOUT0_OUT(CLKOUT0), 
    .CLKOUT1_OUT(CLKOUT1), 
    .CLKOUT2_OUT(CLKOUT2), 
    .CLKOUT3_OUT(CLKOUT3), 
    .CLKOUT4_OUT(CLKOUT4), 
    .CLKOUT5_OUT(), 
    .LOCKED_OUT(LOCKED)
    );
	 
	 //added to replace wrap_sitcp...
	 assign TCP_CLOSE_ACK = TCP_CLOSE_REQ;
	 (* KEEP = "TRUE" *) wire chipClk;
    assign BUS_RST = USER_RESET | !LOCKED;
    assign CLKFBIN = CLKFBOUT;
    BUFG BUFG_BUS (  .O(BUS_CLK),  .I(REG_CLK) );
    BUFG BUFG_SPI(  .O(SPI_CLK),  .I(CLKOUT2) );
	 BUFG BUFG_CHIP(  .O(chipClk),  .I(CLKOUT3) );
	 assign USR_CLK = BUS_CLK;
	 wire RBCP_WE, RBCP_RE;
    wire [7:0] RBCP_WD, RBCP_RD;
    wire [31:0] RBCP_ADDR;
    wire RBCP_ACK;
	 (* KEEP = "TRUE" *) wire RBCP_ACK_CHIPSCOPE;
	 (* KEEP = "TRUE" *)wire CHIPSCOPE_REG_DI;
	 (* KEEP = "TRUE" *)wire CHIPSCOPE_REG_DO;
	 (* KEEP = "TRUE" *) wire CHIP_TX_WR = TCP_TX_WR;
	 (* KEEP = "TRUE" *) wire CHIP_TX_AFULL = USR_TX_AFULL;
	 (* KEEP = "TRUE" *) wire CHIP_CLOSE_ACK = TCP_CLOSE_ACK;
	 assign RBCP_ACK_CHIPSCOPE = RBCP_ACK;
	 assign CHIPSCOPE_REG_DI = REG_DI;
	 assign CHIPSCOPE_REG_DO = REG_DO;
    wire TCP_TX_FULL;
    
     SIO_SLAVE serial_to_bus (
		 .RSTn(RST), //Not used
		 .FILL_ADDR(32'h0), 
		 .SCK(BUS_CLK), 
		 .SCS(RBCP_ACT), 
		 .SI(REG_DI), 
		 .SO(REG_DO), 
		 .REG_ADDR(RBCP_ADDR[31:0]), 
		 .REG_WD(RBCP_WD[7:0]), 
		 .REG_WE(RBCP_WE), 
		 .REG_RE(RBCP_RE), 
		 .REG_ACK(RBCP_ACK), 
		 .REG_RV(RBCP_ACK), //not used???? 
		 .REG_RD(RBCP_RD[7:0])
    );
	 
    wire BUS_WR, BUS_RD;
    wire [31:0] BUS_ADD;
    wire [7:0] BUS_DATA;
    
    rbcp_to_bus irbcp_to_bus(
    
    .BUS_RST(BUS_RST),
    .BUS_CLK(BUS_CLK),
    
    //.RBCP_ACT(RBCP_ACT),
    .RBCP_ADDR(RBCP_ADDR),
    .RBCP_WD(RBCP_WD),
    .RBCP_WE(RBCP_WE),
    .RBCP_RE(RBCP_RE),
    .RBCP_ACK(RBCP_ACK),
    .RBCP_RD(RBCP_RD),
    
    .BUS_WR(BUS_WR),
    .BUS_RD(BUS_RD),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA)
  );
    
    //MODULE ADREESSES
    localparam GPIO_BASEADDR = 32'h0000_0000;
    localparam GPIO_HIGHADDR = 32'h0000_000f;
    
    localparam FIFO_BASEADDR = 32'h0020;                    // 0x0020
    localparam FIFO_HIGHADDR = FIFO_BASEADDR + 15;          // 0x002f
    
    localparam FAST_SR_AQ_BASEADDR = 32'h0100;                    
    localparam FAST_SR_AQ_HIGHADDR = FAST_SR_AQ_BASEADDR + 15;
    
    localparam TDC_BASEADDR = 32'h0200;                    
    localparam TDC_HIGHADDR = TDC_BASEADDR + 15; 

    localparam SEQ_GEN_BASEADDR = 32'h1000;                      //0x1000
    localparam SEQ_GEN_HIGHADDR = SEQ_GEN_BASEADDR + 16 + 32'h1fff;   //0x300f
    
	 wire [3:0] GPIO_DIP_TEST;
	 assign GPIO_DIP_TEST = GPIO_DIP;
     
    // MODULES //
    gpio 
    #( 
        .BASEADDR(GPIO_BASEADDR), 
        .HIGHADDR(GPIO_HIGHADDR),
        .ABUSWIDTH(32),
        .IO_WIDTH(8), 
        .IO_DIRECTION(8'h0f) 
    ) i_gpio
    (
        .BUS_CLK(BUS_CLK),
        .BUS_RST(BUS_RST),
        .BUS_ADD(BUS_ADD),
        .BUS_DATA(BUS_DATA),
        .BUS_RD(BUS_RD),
        .BUS_WR(BUS_WR),
        .IO({GPIO_DIP_TEST, GPIO_LED})
    );
     
    wire [7:0] SEQ_OUT;
    seq_gen 
    #( 
        .BASEADDR(SEQ_GEN_BASEADDR), 
        .HIGHADDR(SEQ_GEN_HIGHADDR),
        .ABUSWIDTH(32),
        .MEM_BYTES(8*1024), 
        .OUT_BITS(8) 
    ) i_seq_gen
    (
        .BUS_CLK(BUS_CLK),
        .BUS_RST(BUS_RST),
        .BUS_ADD(BUS_ADD),
        .BUS_DATA(BUS_DATA),
        .BUS_RD(BUS_RD),
        .BUS_WR(BUS_WR),
    
        .SEQ_CLK(SPI_CLK),
        .SEQ_OUT(SEQ_OUT)
    );
    wire SR_IN, GLOBAL_SR_EN, GLOBAL_CTR_LD, GLOBAL_DAC_LD, PIXEL_SR_EN, INJECT;
    wire GLOBAL_SR_CLK, PIXEL_SR_CLK;
    assign SR_IN                = SEQ_OUT[0];
    assign GLOBAL_SR_EN         = SEQ_OUT[1];   
    assign GLOBAL_CTR_LD        = SEQ_OUT[2];   
    assign GLOBAL_DAC_LD        = SEQ_OUT[3];     
    assign PIXEL_SR_EN          = SEQ_OUT[4];
    assign INJECT               = SEQ_OUT[5];
	 assign cmd[0] = SR_IN;
	 assign cmd[1] = GLOBAL_SR_EN;
	 assign cmd[2] = GLOBAL_CTR_LD;
	 assign cmd[3] = GLOBAL_DAC_LD;
	 assign cmd[4] = SEQ_OUT[4];
	 assign cmd[5] = SEQ_OUT[5];
	 assign cmd[6] = PIXEL_SR_CLK;
	 assign cmd[7] = GLOBAL_SR_CLK;
	 
	
    OFDDRRSE GLOBAL_SR_GC (
        .CE(GLOBAL_SR_EN), 
        .C0(~SPI_CLK),
        .C1(SPI_CLK),
        .D0(1'b1),
        .D1(1'b0),
        .R(1'b0),
        .S(1'b0),
        .Q(GLOBAL_SR_CLK)
    );

    OFDDRRSE PIXEL_SR_GC (
        .CE(PIXEL_SR_EN), 
        .C0(~SPI_CLK),
        .C1(SPI_CLK),
        .D0(1'b1),
        .D1(1'b0),
        .R(1'b0),
        .S(1'b0),
        .Q(PIXEL_SR_CLK)
    );
	 
	 /*
		ODDR2 #(
			.DDR_ALIGNMENT("NONE"), // Sets output alignment to "NONE", "C0" or "C1"
			.INIT(1'b0), // Sets initial state of the Q output to 1'b0 or 1'b1
			.SRTYPE("SYNC") // Specifies "SYNC" or "ASYNC" set/reset
		) 
		GLOBAL_SR_GC (
		  .CE(GLOBAL_SR_EN), 
        .C0(~SPI_CLK),
        .C1(SPI_CLK),
        .D0(1'b1),
        .D1(1'b0),
        .R(1'b0),
        .S(1'b0),
        .Q(GLOBAL_SR_CLK)
	 );
	 ODDR2 #(
			.DDR_ALIGNMENT("NONE"), // Sets output alignment to "NONE", "C0" or "C1"
			.INIT(1'b0), // Sets initial state of the Q output to 1'b0 or 1'b1
			.SRTYPE("SYNC") // Specifies "SYNC" or "ASYNC" set/reset
		) 
		PIXEL_SR_GC (
		  .CE(PIXEL_SR_EN), 
        .C0(~SPI_CLK),
        .C1(SPI_CLK),
        .D0(1'b1),
        .D1(1'b0),
        .R(1'b0),
        .S(1'b0),
        .Q(PIXEL_SR_CLK)
	 );
	 */
    wire [31:0] FIFO_DATA_SPI_RX;
    wire FIFO_EMPTY_SPI_RX;
    wire FIFO_READ_SPI_RX;
    //wire PIXEL_SR_OUT;
    //assign PIXEL_SR_OUT = SR_IN;
    
    fast_spi_rx 
    #(         
        .BASEADDR(FAST_SR_AQ_BASEADDR), 
        .HIGHADDR(FAST_SR_AQ_HIGHADDR),
		  .ABUSWIDTH(32) //changed from the default of 16
    ) i_pixel_sr_fast_rx
    (
        .BUS_CLK(BUS_CLK),
        .BUS_RST(BUS_RST),
        .BUS_ADD(BUS_ADD),
        .BUS_DATA(BUS_DATA),
        .BUS_RD(BUS_RD),
        .BUS_WR(BUS_WR),
        
        .SCLK(~SPI_CLK),
        .SDI(PIXEL_SR_OUT),
        .SEN(PIXEL_SR_EN),
    
        .FIFO_READ(FIFO_READ_SPI_RX),
        .FIFO_EMPTY(FIFO_EMPTY_SPI_RX),
        .FIFO_DATA(FIFO_DATA_SPI_RX)

    ); 
    
    wire TDC_FIFO_READ;
    wire TDC_FIFO_EMPTY;
    wire [31:0] TDC_FIFO_DATA;
    assign TDC_FIFO_EMPTY = 1'b1;
    //TODO: TDC

    wire ARB_READY_OUT, ARB_WRITE_OUT;
    wire [31:0] ARB_DATA_OUT;
     
    rrp_arbiter 
    #( 
        .WIDTH(2)
    ) i_rrp_arbiter
    (
        .RST(BUS_RST),
        .CLK(BUS_CLK),
    
        .WRITE_REQ({~FIFO_EMPTY_SPI_RX, ~TDC_FIFO_EMPTY}),
        .HOLD_REQ({2'b0}),
        .DATA_IN({FIFO_DATA_SPI_RX, TDC_FIFO_DATA}),
        .READ_GRANT({FIFO_READ_SPI_RX, TDC_FIFO_READ}),

        .READY_OUT(ARB_READY_OUT),
        .WRITE_OUT(ARB_WRITE_OUT),
        .DATA_OUT(ARB_DATA_OUT)
    );
    
    wire FIFO_EMPTY, FIFO_FULL;
    fifo_32_to_8 #(.DEPTH(4*1024)) i_data_fifo (
        .RST(BUS_RST),
        .CLK(BUS_CLK),
        
        .WRITE(ARB_WRITE_OUT),
        .READ(TCP_TX_WR),
        .DATA_IN(ARB_DATA_OUT),
        .FULL(FIFO_FULL),
        .EMPTY(FIFO_EMPTY),
        .DATA_OUT(TCP_TX_DATA)
    );
    assign ARB_READY_OUT = !FIFO_FULL;
    assign TCP_TX_WR = !USR_TX_AFULL && !FIFO_EMPTY;//TCP_TX_FULL changed to USR_TX_AFULL
    
endmodule
