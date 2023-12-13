/*
File: DE1_SoC_Computer.v
Modified by: Erik Van Schijndel
Course: ECE 643
Instructor: Don Gruenbacher
*/
module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
input						AUD_ADCLRCK;
input						AUD_BCLK;
output					AUD_DACDAT;
input						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
output			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0 /* synthesis chip_pin = "AH28, AG28, AF28, AG27, AE28, AE27, AE26" */;
output		[ 6: 0]	HEX1 /* synthesis chip_pin = "AD27, AF30, AF29, AG30, AH30, AH29, AJ29" */;
output		[ 6: 0]	HEX2 /* synthesis chip_pin = "AC30,AC29,AD30,AC28, AD29, AE29, AB23" */;
output		[ 6: 0]	HEX3 /* synthesis chip_pin = "Ab22, AB25, AB28, AC25, AD25, AC27, AD26" */;
output		[ 6: 0]	HEX4 /* synthesis chip_pin = "W25, V23, W24, W22, Y24, Y23, AA24" */;
output		[ 6: 0]	HEX5 /* synthesis chip_pin = "AA25, AA26, AB26, AB27,Y27, AA28, V25" */;


// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

//wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
//assign HEX4 = 7'b1111111;
//assign HEX5 = 7'b1111111;

//HexDigit Digit0(HEX0, hex3_hex0[3:0]);
//HexDigit Digit1(HEX1, hex3_hex0[7:4]);
//HexDigit Digit2(HEX2, hex3_hex0[11:8]);
//HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),

	// Audio Subsystem
	/*.audio_pll_ref_clk_clk					(CLOCK3_50),
	.audio_pll_ref_reset_reset				(1'b0),
	.audio_clk_clk								(AUD_XCK),
	.audio_ADCDAT								(AUD_ADCDAT),
	.audio_ADCLRCK								(AUD_ADCLRCK),
	.audio_BCLK									(AUD_BCLK),
	.audio_DACDAT								(AUD_DACDAT),
	.audio_DACLRCK								(AUD_DACLRCK),
*/
/*	.audio_pll_0_ref_clk_clk         (CLOCK_50),         //  audio_pll_0_ref_clk.clk
   .audio_interface_ADCDAT          (AUD_ADCDAT),          //      audio_interface.ADCDAT
        .audio_interface_ADCLRCK         (AUD_ADCLRCK),         //                     .ADCLRCK
        .audio_interface_BCLK            (AUD_BCLK),            //                     .BCLK
        .audio_interface_DACDAT          (AUD_DACDAT),          //                     .DACDAT
        .audio_interface_DACLRCK         (AUD_DACLRCK),         //                     .DACLRCK
*/    
  
	// Slider Switches
	.slider_switches_export					(SW),

	// Pushbuttons (~KEY[3:0]),
	.pushbuttons_export						(~KEY[3:0]),

	.letter_decoded_pio_external_connection_export			(decoded_buffer),
	
	// Expansion JP1
	//.expansion_jp1_export					({GPIO_0[35:19], GPIO_0[17], GPIO_0[15:3], GPIO_0[1]}),

	// Expansion JP2
	//.expansion_jp2_export					({GPIO_1[35:19], GPIO_1[17], GPIO_1[15:3], GPIO_1[1]}),

	// LEDs
	//.leds_export								(LEDR),
	
	// Seven Segs
	//.hex3_hex0_export							(hex3_hex0),
	//.hex5_hex4_export							(hex5_hex4),
	
	// PS2 Ports
	//.ps2_port_CLK								(PS2_CLK),
	//.ps2_port_DAT								(PS2_DAT),
	//.ps2_port_dual_CLK						(PS2_CLK2),
	//.ps2_port_dual_DAT						(PS2_DAT2),

	// IrDA
	//.irda_RXD									(IRDA_RXD),
	//.irda_TXD									(IRDA_TXD),

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// Video In Subsystem
//	.video_in_TD_CLK27 						(TD_CLK27),
//	.video_in_TD_DATA							(TD_DATA),
//	.video_in_TD_HS							(TD_HS),
//	.video_in_TD_VS							(TD_VS),
//	.video_in_clk27_reset					(),
//	.video_in_TD_RESET						(TD_RESET_N),
//	.video_in_overflow_flag					(),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT),
	.ctrl_export                     (audio_ctrl),
    .space_export                    (audio_space),
    .dac_left_dat_export             (dacl_fifo_indat),
    .adc_left_dat_export  			(adcl_fifo_outdat)
);

wire [7:0] audio_ctrl;
wire [15:0] audio_space;

assign adcl_fifo_rdreq = audio_ctrl[3];
assign dacl_fifo_wrreq = audio_ctrl[2];
assign adcl_fifo_sclr = audio_ctrl[1];
assign dacl_fifo_sclr = audio_ctrl[0];

assign audio_space = {10'b0, adcl_fifo_full, adcl_fifo_usedw[6], adcl_fifo_empty, 
							 dacl_fifo_full, dacl_fifo_usedw[6], dacl_fifo_empty};

	// ADC Left FIFO interface signals
	wire	[31:0]  adcl_fifo_indat, adcl_fifo_outdat;
	wire	  adcl_fifo_rdreq;
	wire	  adcl_fifo_sclr;
	wire	  adcl_fifo_wrreq;
	wire	  adcl_fifo_empty;
	wire	  adcl_fifo_full;
	wire	[6:0]  adcl_fifo_usedw;
	reg		adcl_fifo_rdreq_prev, adcl_fifo_wrreq_prev;
	wire	adcl_fifo_rdreq_edge, adcl_fifo_wrreq_edge;

	// DAC Left FIFO interface signals
	wire	[31:0]  dacl_fifo_indat, dacl_fifo_outdat;
	wire	  dacl_fifo_rdreq;
	wire	  dacl_fifo_sclr;
	wire	  dacl_fifo_wrreq;
	wire	  dacl_fifo_empty;
	wire	  dacl_fifo_full;
	wire	[6:0]  dacl_fifo_usedw;
	reg		dacl_fifo_rdreq_prev, dacl_fifo_wrreq_prev;
	wire	dacl_fifo_rdreq_edge, dacl_fifo_wrreq_edge;
	
	wire  ar;
	
assign ar = KEY[2];

// Geneate read and write request control signals for FIFO's.   These signals can only be active for 1 clock period.  Hence they indicate a positive edge  

assign dacl_fifo_rdreq = AUD_DACLRCK;   // Read request for left DAC FIFO
assign adcl_fifo_wrreq = AUD_ADCLRCK;	// Write request for left ADC FIFO


	always@(negedge ar or posedge AUD_XCK)
		if(~ar)
			begin
			adcl_fifo_rdreq_prev = 1'b0;
			adcl_fifo_wrreq_prev = 1'b0;
			dacl_fifo_rdreq_prev = 1'b0;
			dacl_fifo_wrreq_prev = 1'b0;
			end
		else
			begin
			adcl_fifo_rdreq_prev = adcl_fifo_rdreq;
			adcl_fifo_wrreq_prev = adcl_fifo_wrreq;
			dacl_fifo_rdreq_prev = dacl_fifo_rdreq;
			dacl_fifo_wrreq_prev = dacl_fifo_wrreq;
			end
			
	assign adcl_fifo_rdreq_edge = adcl_fifo_rdreq & ~adcl_fifo_rdreq_prev;
	assign adcl_fifo_wrreq_edge = adcl_fifo_wrreq & ~adcl_fifo_wrreq_prev;
	assign dacl_fifo_rdreq_edge = dacl_fifo_rdreq & ~dacl_fifo_rdreq_prev;
	assign dacl_fifo_wrreq_edge = dacl_fifo_wrreq & ~dacl_fifo_wrreq_prev;
	
	assign GPIO_0[6] = dacl_fifo_rdreq_edge;
	assign GPIO_0[7] = adcl_fifo_wrreq_edge;
	
	//User input
	assign button = KEY[0];
	
	//input clock:50mhz
	reg [25:0] clk_counter = 0; 

	always@(posedge CLOCK_50)begin
		clk_counter <= clk_counter + 1; //Counter used to divide clock
	end	
	
	assign clock_millisec = clk_counter[22]; //Divides clock to milliseconds
	
	reg [10:0] counter = 0;			//Time counter
	reg [4:0] pattern = 0;			//pattern=0000 -> 4 short  , pattern=1111 -> 4 long
	reg [4:0] pattern_buff = 0; //holds prev letter pattern
	reg [2:0] input_num = 0;		// how many letters in the pattern, eg:E:1, H4
	reg [2:0] input_num_prev = 0; //holds prev button press count
	reg [1:0] button_state = 0;
	reg [4:0] decoded_buffer = 0; // 10->A 35->Z
	reg  newletterFlag= 0;
	reg [4:0] decoded_display[0:5];	//holds prev input, is used to shift letters over 
	
	localparam WAIT= 0; //assign for state machine
	localparam BUTTON_PRESS = 1;
	localparam BUTTON_CHECK_FURTHER = 2;

	reg short = 0; //debugging, first red is short, second red is long
	reg long = 0;
	assign LEDR[0] = short; 
	assign LEDR[1] = long;
	
	
	always@(posedge clock_millisec) begin
		case(button_state)
			WAIT:begin
				if(button == 0) begin 		//button pressed
					button_state <= BUTTON_PRESS;
				end
				pattern <= 0; //Reset all reg
				input_num <= 0;
				counter <= 0;
				newletterFlag<= 0;				
			end
			BUTTON_PRESS:begin
				if(button == 0)begin       //still pressed, track length for decoding
					counter <= counter + 1;	
				end
				else begin				    //now released
					if(counter > 4) begin 				// its a long press
						pattern[input_num] <= 1;
						input_num<=input_num + 1;
						button_state <= BUTTON_CHECK_FURTHER;
						short <= 0;
						long <= 1;
					end
					else begin 								// its a short press
						pattern[input_num] <= 0;
						input_num <= input_num + 1;
						button_state <= BUTTON_CHECK_FURTHER;
						short <= 1;
						long <= 0;						
					end
					counter <= 0;
				end
			end
			BUTTON_CHECK_FURTHER:begin
				if(button == 1)begin					
					
					if(counter > 4)begin					//released for long time:new letter
						button_state <= WAIT; 				//Begin shifting buffer values into decoded_display
						newletterFlag<= 1;					//for sev_seg_erik
						decoded_display[0] <= decoded_buffer; 
						decoded_display[1] <= decoded_display[0];
						decoded_display[2] <= decoded_display[1];
						decoded_display[3] <= decoded_display[2];
						decoded_display[4] <= decoded_display[3];
						decoded_display[5] <= decoded_display[4];
						pattern_buff <= pattern;		
						input_num_prev <= input_num;						
					end
					else
					  counter <= counter + 1;				
						
				end
				else begin
  						//same letter continuation
						button_state <= BUTTON_PRESS;
						counter <= 0;
				end
			end
			
		endcase
	end
	
	always@(*)begin
		if(newletterFlag)begin
			if(input_num_prev == 1)begin /// letters with single morse signals
				if(pattern_buff[0] == 1)
					decoded_buffer = 29;//T=20
				else
					decoded_buffer = 14;//E=5
			end
			else if(input_num_prev == 2)begin // letters with two morse signals
				if(pattern_buff[1:0] == 2'b10)begin
					decoded_buffer = 10;//A
					
				end
				else if(pattern_buff[1:0] == 2'b00)begin
					decoded_buffer = 18;//I
				end
				else if(pattern_buff[1:0] == 2'b11)begin
					decoded_buffer = 22;//M
				end
				else if(pattern_buff[1:0] == 2'b01)begin
					decoded_buffer = 23;//N
				end			
			end
			else if(input_num_prev == 3)begin // letters with three morse signals
				if(pattern_buff[2:0] == 3'b000)begin
					decoded_buffer = 28;//S
				end
				else if(pattern_buff[2:0] == 3'b100)begin
					decoded_buffer = 30;//U
				end
				else if(pattern_buff[2:0] == 3'b001)begin
					decoded_buffer = 13;//D
				end	
				else if(pattern_buff[2:0] == 3'b011)begin
					decoded_buffer = 16; //G
				end	
				else if(pattern_buff[2:0] == 3'b101)begin
					decoded_buffer = 20;//K
				end	
				else if(pattern_buff[2:0] == 3'b111)begin
					decoded_buffer = 24;//O
				end
				else if(pattern_buff[2:0] == 3'b010)begin
					decoded_buffer = 27;//R
				end
				else if(pattern_buff[2:0] == 3'b110)begin
					decoded_buffer = 32;//W
				end			
			end
			else if(input_num_prev == 4)begin // letters with four morse signals
				if(pattern_buff[3:0] == 4'b0000)begin
					decoded_buffer = 17;//H
				end
				else if(pattern_buff[3:0] == 4'b0001)begin
					decoded_buffer = 11;//B
				end
				else if(pattern_buff[3:0] == 4'b0101)begin
					decoded_buffer = 12;//C
				end				
				else if(pattern_buff[3:0] == 4'b0100)begin
					decoded_buffer = 15;//F
				end
				else if(pattern_buff[3:0] == 4'b1110)begin
					decoded_buffer = 19;//J
				end
				else if(pattern_buff[3:0] == 4'b0010)begin
					decoded_buffer = 21;//L
				end
				else if(pattern_buff[3:0] == 4'b0110)begin
					decoded_buffer = 25;//P 
				end
				else if(pattern_buff[3:0] == 4'b1011)begin
					decoded_buffer = 26;//Q
				end
				else if(pattern_buff[3:0] == 4'b1000)begin
					decoded_buffer = 31;//V
				end
				else if(pattern_buff[3:0] == 4'b1001)begin
					decoded_buffer = 33;//X
				end	
				else if(pattern_buff[3:0] == 4'b1101)begin
					decoded_buffer = 34;//Y
				end				
				else if(pattern_buff[3:0] == 4'b0011)begin
					decoded_buffer = 35;//Z
				end					
			end	
				else if(input_num_prev == 5)begin // numbers with five signals
				if(pattern_buff[4:0] == 5'b11111)begin
					decoded_buffer = 0;//0
				end
				else if(pattern_buff[4:0] == 5'b11110)begin
					decoded_buffer = 1; //1
				end
				else if(pattern_buff[4:0] == 5'b11100)begin
					decoded_buffer = 2; //2
				end
				else if(pattern_buff[4:0] == 5'b11000)begin
					decoded_buffer = 3; //3
				end
				else if(pattern_buff[4:0] == 5'b10000)begin
					decoded_buffer = 4; //4
				end
				else if(pattern_buff[4:0] == 5'b00000)begin
					decoded_buffer = 5; //5
				end
				else if(pattern_buff[4:0] == 5'b00001)begin
					decoded_buffer = 6; //6
				end
				else if(pattern_buff[4:0] == 5'b00011)begin
					decoded_buffer = 7; //7
				end
				else if(pattern_buff[4:0] == 5'b00111)begin
					decoded_buffer = 8; //8
				end
				else if(pattern_buff[4:0] == 5'b01111)begin
					decoded_buffer = 9; //9
				end
			end
		end	
	end 
	
	// Test numbers thru LEDs
	sev_seg_erik h0(decoded_display[0],HEX0);
	sev_seg_erik h1(decoded_display[1],HEX1);
	sev_seg_erik h2(decoded_display[2],HEX2);
	sev_seg_erik h3(decoded_display[3],HEX3);
	sev_seg_erik h4(decoded_display[4],HEX4);
	sev_seg_erik h5(decoded_display[5],HEX5);
	
audio audio_sys(
.aud_xclk(AUD_XCK), // clock 12. MHz?
.bclk(AUD_BCLK)		, // bit stream clock
.adclrck(AUD_ADCLRCK)	, // left right clock ADC
.adcdat(AUD_ADCDAT)	, // data stream ADC
.daclrck(AUD_DACLRCK)	, // left right clock DAC
.dacdat(AUD_DACDAT)	, // data stream DAC
//sclk		, // serial clock I2C
//sdat		, // serail data I2C
.ar(KEY[2])		,
.clk(CLOCK_50)		,  // 50 MHz
.adc_left_data(adcl_fifo_indat),
.dac_left_data({1'b0,dacl_fifo_indat}),
//.dac_left_data(dacl_fifo_outdat),
.gpio(GPIO_0[5:0]));     // 40 pin header

/*
audio_fifo test_fifo (  // Used to just test R/W to FIFO through PIOs
	.clock(AUD_XCK),
	.data(dacl_fifo_indat),
	.rdreq(adcl_fifo_rdreq_edge),
	.sclr(adcl_fifo_sclr),
	.wrreq(dacl_fifo_wrreq_edge),
	.empty(adcl_fifo_empty),
	.full(adcl_fifo_full),
	.q(adcl_fifo_outdat),
	.usedw(adcl_fifo_usedw));
*/	
	
audio_fifo adcl_fifo (
	.clock(AUD_XCK),
	.data(adcl_fifo_indat),
	.rdreq(adcl_fifo_rdreq_edge),
	.sclr(adcl_fifo_sclr),
	.wrreq(adcl_fifo_wrreq_edge),
	.empty(adcl_fifo_empty),
	.full(adcl_fifo_full),
	.q(adcl_fifo_outdat),
	.usedw(adcl_fifo_usedw));


audio_fifo dacl_fifo (
	.clock(AUD_XCK),
	.data(dacl_fifo_indat),
		//.rdreq(1'b1),
	.rdreq(dacl_fifo_rdreq_edge),
	.sclr(dacl_fifo_sclr),
	.wrreq(dacl_fifo_wrreq_edge),
	.empty(dacl_fifo_empty),
	.full(dacl_fifo_full),
	.q(dacl_fifo_outdat),
	.usedw(dacl_fifo_usedw));
	


endmodule
