--
-- Synopsys
-- Vhdl wrapper for top level design, written on Sat Feb 17 17:02:57 2024
--
library ieee;
use ieee.std_logic_1164.all;
library ecp5u;
use ecp5u.components.all;
use ieee.numeric_std.all;
library work;
use work.components.all;

entity wrapper_for_sdram_fpga_hex_oled is
   port (
      clk_25mhz : in std_logic;
      ftdi_rxd : out std_logic;
      ftdi_txd : in std_logic;
      ftdi_ndtr : in std_logic;
      ftdi_ndsr : in std_logic;
      ftdi_nrts : in std_logic;
      ftdi_txden : in std_logic;
      wifi_rxd : out std_logic;
      wifi_txd : in std_logic;
      wifi_en : in std_logic;
      wifi_gpio0 : in std_logic;
      wifi_gpio2 : in std_logic;
      wifi_gpio15 : in std_logic;
      wifi_gpio16 : in std_logic;
      led : out std_logic_vector(7 downto 0);
      btn : in std_logic_vector(6 downto 0);
      sw : in std_logic_vector(1 to 4);
      oled_csn : out std_logic;
      oled_clk : out std_logic;
      oled_mosi : out std_logic;
      oled_dc : out std_logic;
      oled_resn : out std_logic;
      gp : in std_logic_vector(27 downto 0);
      gn : in std_logic_vector(27 downto 0);
      usb_fpga_dp : in std_logic;
      usb_fpga_dn : in std_logic;
      shutdown : out std_logic;
      sdram_csn : out std_logic;
      sdram_clk : out std_logic;
      sdram_cke : out std_logic;
      sdram_rasn : out std_logic;
      sdram_casn : out std_logic;
      sdram_wen : out std_logic;
      sdram_a : out unsigned(12 downto 0);
      sdram_ba : out unsigned(1 downto 0);
      sdram_dqm : out std_logic_vector(1 downto 0);
      sdram_d : in std_logic_vector(15 downto 0);
      sd_dat3_csn : in std_logic;
      sd_cmd_di : in std_logic;
      sd_dat0_do : in std_logic;
      sd_dat1_irq : in std_logic;
      sd_dat2 : in std_logic;
      sd_clk : in std_logic;
      sd_cdn : in std_logic;
      sd_wp : in std_logic
   );
end wrapper_for_sdram_fpga_hex_oled;

architecture behavioral of wrapper_for_sdram_fpga_hex_oled is

component sdram_fpga_hex_oled
 port (
   clk_25mhz : in std_logic;
   ftdi_rxd : out std_logic;
   ftdi_txd : in std_logic;
   ftdi_ndtr : inout std_logic;
   ftdi_ndsr : inout std_logic;
   ftdi_nrts : inout std_logic;
   ftdi_txden : inout std_logic;
   wifi_rxd : out std_logic;
   wifi_txd : in std_logic;
   wifi_en : inout std_logic;
   wifi_gpio0 : inout std_logic;
   wifi_gpio2 : inout std_logic;
   wifi_gpio15 : inout std_logic;
   wifi_gpio16 : inout std_logic;
   led : out std_logic_vector (7 downto 0);
   btn : in std_logic_vector (6 downto 0);
   sw : in std_logic_vector (1 to 4);
   oled_csn : out std_logic;
   oled_clk : out std_logic;
   oled_mosi : out std_logic;
   oled_dc : out std_logic;
   oled_resn : out std_logic;
   gp : inout std_logic_vector (27 downto 0);
   gn : inout std_logic_vector (27 downto 0);
   usb_fpga_dp : inout std_logic;
   usb_fpga_dn : inout std_logic;
   shutdown : out std_logic;
   sdram_csn : out std_logic;
   sdram_clk : out std_logic;
   sdram_cke : out std_logic;
   sdram_rasn : out std_logic;
   sdram_casn : out std_logic;
   sdram_wen : out std_logic;
   sdram_a : out std_logic_vector (12 downto 0);
   sdram_ba : out std_logic_vector (1 downto 0);
   sdram_dqm : out std_logic_vector (1 downto 0);
   sdram_d : inout std_logic_vector (15 downto 0);
   sd_dat3_csn : inout std_logic;
   sd_cmd_di : inout std_logic;
   sd_dat0_do : inout std_logic;
   sd_dat1_irq : inout std_logic;
   sd_dat2 : inout std_logic;
   sd_clk : inout std_logic;
   sd_cdn : inout std_logic;
   sd_wp : inout std_logic
 );
end component;

signal tmp_clk_25mhz : std_logic;
signal tmp_ftdi_rxd : std_logic;
signal tmp_ftdi_txd : std_logic;
signal tmp_ftdi_ndtr : std_logic;
signal tmp_ftdi_ndsr : std_logic;
signal tmp_ftdi_nrts : std_logic;
signal tmp_ftdi_txden : std_logic;
signal tmp_wifi_rxd : std_logic;
signal tmp_wifi_txd : std_logic;
signal tmp_wifi_en : std_logic;
signal tmp_wifi_gpio0 : std_logic;
signal tmp_wifi_gpio2 : std_logic;
signal tmp_wifi_gpio15 : std_logic;
signal tmp_wifi_gpio16 : std_logic;
signal tmp_led : std_logic_vector (7 downto 0);
signal tmp_btn : std_logic_vector (6 downto 0);
signal tmp_sw : std_logic_vector (1 to 4);
signal tmp_oled_csn : std_logic;
signal tmp_oled_clk : std_logic;
signal tmp_oled_mosi : std_logic;
signal tmp_oled_dc : std_logic;
signal tmp_oled_resn : std_logic;
signal tmp_gp : std_logic_vector (27 downto 0);
signal tmp_gn : std_logic_vector (27 downto 0);
signal tmp_usb_fpga_dp : std_logic;
signal tmp_usb_fpga_dn : std_logic;
signal tmp_shutdown : std_logic;
signal tmp_sdram_csn : std_logic;
signal tmp_sdram_clk : std_logic;
signal tmp_sdram_cke : std_logic;
signal tmp_sdram_rasn : std_logic;
signal tmp_sdram_casn : std_logic;
signal tmp_sdram_wen : std_logic;
signal tmp_sdram_a : std_logic_vector (12 downto 0);
signal tmp_sdram_ba : std_logic_vector (1 downto 0);
signal tmp_sdram_dqm : std_logic_vector (1 downto 0);
signal tmp_sdram_d : std_logic_vector (15 downto 0);
signal tmp_sd_dat3_csn : std_logic;
signal tmp_sd_cmd_di : std_logic;
signal tmp_sd_dat0_do : std_logic;
signal tmp_sd_dat1_irq : std_logic;
signal tmp_sd_dat2 : std_logic;
signal tmp_sd_clk : std_logic;
signal tmp_sd_cdn : std_logic;
signal tmp_sd_wp : std_logic;

begin

tmp_clk_25mhz <= clk_25mhz;

ftdi_rxd <= tmp_ftdi_rxd;

tmp_ftdi_txd <= ftdi_txd;

tmp_ftdi_ndtr <= ftdi_ndtr;

tmp_ftdi_ndsr <= ftdi_ndsr;

tmp_ftdi_nrts <= ftdi_nrts;

tmp_ftdi_txden <= ftdi_txden;

wifi_rxd <= tmp_wifi_rxd;

tmp_wifi_txd <= wifi_txd;

tmp_wifi_en <= wifi_en;

tmp_wifi_gpio0 <= wifi_gpio0;

tmp_wifi_gpio2 <= wifi_gpio2;

tmp_wifi_gpio15 <= wifi_gpio15;

tmp_wifi_gpio16 <= wifi_gpio16;

led <= tmp_led;

tmp_btn <= btn;

tmp_sw <= sw;

oled_csn <= tmp_oled_csn;

oled_clk <= tmp_oled_clk;

oled_mosi <= tmp_oled_mosi;

oled_dc <= tmp_oled_dc;

oled_resn <= tmp_oled_resn;

tmp_gp <= gp;

tmp_gn <= gn;

tmp_usb_fpga_dp <= usb_fpga_dp;

tmp_usb_fpga_dn <= usb_fpga_dn;

shutdown <= tmp_shutdown;

sdram_csn <= tmp_sdram_csn;

sdram_clk <= tmp_sdram_clk;

sdram_cke <= tmp_sdram_cke;

sdram_rasn <= tmp_sdram_rasn;

sdram_casn <= tmp_sdram_casn;

sdram_wen <= tmp_sdram_wen;

sdram_a <= ieee.numeric_std.unsigned(tmp_sdram_a);

sdram_ba <= ieee.numeric_std.unsigned(tmp_sdram_ba);

sdram_dqm <= tmp_sdram_dqm;

tmp_sdram_d <= sdram_d;

tmp_sd_dat3_csn <= sd_dat3_csn;

tmp_sd_cmd_di <= sd_cmd_di;

tmp_sd_dat0_do <= sd_dat0_do;

tmp_sd_dat1_irq <= sd_dat1_irq;

tmp_sd_dat2 <= sd_dat2;

tmp_sd_clk <= sd_clk;

tmp_sd_cdn <= sd_cdn;

tmp_sd_wp <= sd_wp;



u1:   sdram_fpga_hex_oled port map (
		clk_25mhz => tmp_clk_25mhz,
		ftdi_rxd => tmp_ftdi_rxd,
		ftdi_txd => tmp_ftdi_txd,
		ftdi_ndtr => tmp_ftdi_ndtr,
		ftdi_ndsr => tmp_ftdi_ndsr,
		ftdi_nrts => tmp_ftdi_nrts,
		ftdi_txden => tmp_ftdi_txden,
		wifi_rxd => tmp_wifi_rxd,
		wifi_txd => tmp_wifi_txd,
		wifi_en => tmp_wifi_en,
		wifi_gpio0 => tmp_wifi_gpio0,
		wifi_gpio2 => tmp_wifi_gpio2,
		wifi_gpio15 => tmp_wifi_gpio15,
		wifi_gpio16 => tmp_wifi_gpio16,
		led => tmp_led,
		btn => tmp_btn,
		sw => tmp_sw,
		oled_csn => tmp_oled_csn,
		oled_clk => tmp_oled_clk,
		oled_mosi => tmp_oled_mosi,
		oled_dc => tmp_oled_dc,
		oled_resn => tmp_oled_resn,
		gp => tmp_gp,
		gn => tmp_gn,
		usb_fpga_dp => tmp_usb_fpga_dp,
		usb_fpga_dn => tmp_usb_fpga_dn,
		shutdown => tmp_shutdown,
		sdram_csn => tmp_sdram_csn,
		sdram_clk => tmp_sdram_clk,
		sdram_cke => tmp_sdram_cke,
		sdram_rasn => tmp_sdram_rasn,
		sdram_casn => tmp_sdram_casn,
		sdram_wen => tmp_sdram_wen,
		sdram_a => tmp_sdram_a,
		sdram_ba => tmp_sdram_ba,
		sdram_dqm => tmp_sdram_dqm,
		sdram_d => tmp_sdram_d,
		sd_dat3_csn => tmp_sd_dat3_csn,
		sd_cmd_di => tmp_sd_cmd_di,
		sd_dat0_do => tmp_sd_dat0_do,
		sd_dat1_irq => tmp_sd_dat1_irq,
		sd_dat2 => tmp_sd_dat2,
		sd_clk => tmp_sd_clk,
		sd_cdn => tmp_sd_cdn,
		sd_wp => tmp_sd_wp
       );
end behavioral;
