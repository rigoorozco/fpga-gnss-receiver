library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_l1_ca_ctrl_phase2 is
  generic (
    ADDRW : integer := 8;
    DATAW : integer := 32;
    G_NUM_CHANNELS : integer := 5
  );
  port (
    clk    : in  std_logic;
    rst_n  : in  std_logic;

    ctrl_wreq  : in  std_logic;
    ctrl_waddr : in  unsigned(ADDRW - 1 downto 0);
    ctrl_wdata : in  std_logic_vector(DATAW - 1 downto 0);
    ctrl_wack  : out std_logic;

    ctrl_rreq  : in  std_logic;
    ctrl_raddr : in  unsigned(ADDRW - 1 downto 0);
    ctrl_rdata : out std_logic_vector(DATAW - 1 downto 0);
    ctrl_rack  : out std_logic;

    acq_done_i         : in  std_logic;
    acq_success_i      : in  std_logic;
    detected_prn_i     : in  unsigned(5 downto 0);
    detected_code_i    : in  unsigned(10 downto 0);
    detected_dopp_i    : in  signed(15 downto 0);
    chan_alloc_i       : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_lock_i        : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    obs_count_i        : in  unsigned(7 downto 0);
    eph_valid_prn_i    : in  std_logic_vector(31 downto 0);
    pvt_valid_i        : in  std_logic;
    pvt_sats_used_i    : in  unsigned(7 downto 0);
    pvt_lat_e7_i       : in  signed(31 downto 0);
    pvt_lon_e7_i       : in  signed(31 downto 0);
    pvt_height_mm_i    : in  signed(31 downto 0);
    pvt_cbias_i        : in  signed(31 downto 0);
    uart_busy_i        : in  std_logic;

    core_en_o          : out std_logic;
    soft_reset_req_o   : out std_logic;
    acq_rescan_pulse_o : out std_logic;
    tracking_en_o      : out std_logic;
    uart_en_o          : out std_logic;
    nav_en_o           : out std_logic;
    obs_en_o           : out std_logic;
    pvt_en_o           : out std_logic;
    chan_enable_mask_o : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    preferred_count_o  : out unsigned(7 downto 0);
    prn_start_o        : out unsigned(5 downto 0);
    prn_stop_o         : out unsigned(5 downto 0);
    doppler_min_o      : out signed(15 downto 0);
    doppler_max_o      : out signed(15 downto 0);
    doppler_step_o     : out signed(15 downto 0);
    detect_thresh_o    : out unsigned(31 downto 0);
    min_cn0_dbhz_o     : out unsigned(7 downto 0);
    carrier_lock_th_o  : out signed(15 downto 0);
    max_lock_fail_o    : out unsigned(7 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_ctrl_phase2 is
  signal ctrl_wack_r       : std_logic := '0';
  signal ctrl_rack_r       : std_logic := '0';
  signal ctrl_rdata_r      : std_logic_vector(DATAW - 1 downto 0) := (others => '0');

  signal core_en_r         : std_logic := '0';
  signal tracking_en_r     : std_logic := '0';
  signal uart_en_r         : std_logic := '0';
  signal nav_en_r          : std_logic := '0';
  signal obs_en_r          : std_logic := '0';
  signal pvt_en_r          : std_logic := '0';
  signal soft_reset_req_r  : std_logic := '0';
  signal acq_rescan_pulse_r: std_logic := '0';

  signal chan_enable_mask_r: std_logic_vector(G_NUM_CHANNELS - 1 downto 0) := (others => '1');
  signal preferred_count_r : unsigned(7 downto 0) := to_unsigned(G_NUM_CHANNELS, 8);

  signal prn_start_r       : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal prn_stop_r        : unsigned(5 downto 0) := to_unsigned(16, 6);
  signal doppler_min_r     : signed(15 downto 0) := to_signed(-5000, 16);
  signal doppler_max_r     : signed(15 downto 0) := to_signed(5000, 16);
  signal doppler_step_r    : signed(15 downto 0) := to_signed(500, 16);
  signal detect_thresh_r   : unsigned(31 downto 0) := to_unsigned(10000, 32);
  signal min_cn0_dbhz_r    : unsigned(7 downto 0) := to_unsigned(22, 8);
  signal carrier_lock_th_r : signed(15 downto 0) := to_signed(19661, 16); -- ~0.60 in Q15
  signal max_lock_fail_r   : unsigned(7 downto 0) := to_unsigned(50, 8);
begin
  ctrl_wack <= ctrl_wack_r;
  ctrl_rack <= ctrl_rack_r;
  ctrl_rdata <= ctrl_rdata_r;

  core_en_o          <= core_en_r;
  soft_reset_req_o   <= soft_reset_req_r;
  acq_rescan_pulse_o <= acq_rescan_pulse_r;
  tracking_en_o      <= tracking_en_r;
  uart_en_o          <= uart_en_r;
  nav_en_o           <= nav_en_r;
  obs_en_o           <= obs_en_r;
  pvt_en_o           <= pvt_en_r;
  chan_enable_mask_o <= chan_enable_mask_r;
  preferred_count_o  <= preferred_count_r;
  prn_start_o        <= prn_start_r;
  prn_stop_o         <= prn_stop_r;
  doppler_min_o      <= doppler_min_r;
  doppler_max_o      <= doppler_max_r;
  doppler_step_o     <= doppler_step_r;
  detect_thresh_o    <= detect_thresh_r;
  min_cn0_dbhz_o     <= min_cn0_dbhz_r;
  carrier_lock_th_o  <= carrier_lock_th_r;
  max_lock_fail_o    <= max_lock_fail_r;

  process (clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        ctrl_wack_r        <= '0';
        ctrl_rack_r        <= '0';
        acq_rescan_pulse_r <= '0';
        soft_reset_req_r   <= '0';
      else
        ctrl_wack_r        <= '0';
        ctrl_rack_r        <= '0';
        acq_rescan_pulse_r <= '0';
        soft_reset_req_r   <= '0';

        if ctrl_wreq = '1' then
          ctrl_wack_r <= '1';
          case to_integer(ctrl_waddr) is
            when 16#00# =>
              core_en_r         <= ctrl_wdata(0);
              soft_reset_req_r  <= ctrl_wdata(1);
              acq_rescan_pulse_r<= ctrl_wdata(2);
              tracking_en_r     <= ctrl_wdata(3);
              uart_en_r         <= ctrl_wdata(4);
              nav_en_r          <= ctrl_wdata(5);
              obs_en_r          <= ctrl_wdata(6);
              pvt_en_r          <= ctrl_wdata(7);
            when 16#04# =>
              chan_enable_mask_r <= ctrl_wdata(G_NUM_CHANNELS - 1 downto 0);
              preferred_count_r  <= unsigned(ctrl_wdata(15 downto 8));
            when 16#08# =>
              prn_start_r <= unsigned(ctrl_wdata(5 downto 0));
              prn_stop_r  <= unsigned(ctrl_wdata(13 downto 8));
            when 16#0C# =>
              detect_thresh_r <= unsigned(ctrl_wdata);
            when 16#10# =>
              doppler_min_r <= signed(ctrl_wdata(15 downto 0));
            when 16#14# =>
              doppler_max_r <= signed(ctrl_wdata(15 downto 0));
            when 16#18# =>
              doppler_step_r <= signed(ctrl_wdata(15 downto 0));
            when 16#1C# =>
              min_cn0_dbhz_r <= unsigned(ctrl_wdata(7 downto 0));
            when 16#20# =>
              carrier_lock_th_r <= signed(ctrl_wdata(15 downto 0));
            when 16#24# =>
              max_lock_fail_r <= unsigned(ctrl_wdata(7 downto 0));
            when others =>
              null;
          end case;
        end if;

        if ctrl_rreq = '1' then
          ctrl_rack_r <= '1';
        end if;
      end if;
    end if;
  end process;

  process (all)
    variable rd : std_logic_vector(DATAW - 1 downto 0);
  begin
    rd := (others => '0');
    case to_integer(ctrl_raddr) is
      when 16#00# =>
        rd(0) := core_en_r;
        rd(3) := tracking_en_r;
        rd(4) := uart_en_r;
        rd(5) := nav_en_r;
        rd(6) := obs_en_r;
        rd(7) := pvt_en_r;
      when 16#04# =>
        rd(G_NUM_CHANNELS - 1 downto 0) := chan_enable_mask_r;
        rd(15 downto 8) := std_logic_vector(preferred_count_r);
      when 16#08# =>
        rd(5 downto 0)  := std_logic_vector(prn_start_r);
        rd(13 downto 8) := std_logic_vector(prn_stop_r);
      when 16#0C# =>
        rd := std_logic_vector(detect_thresh_r);
      when 16#10# =>
        rd(15 downto 0) := std_logic_vector(doppler_min_r);
      when 16#14# =>
        rd(15 downto 0) := std_logic_vector(doppler_max_r);
      when 16#18# =>
        rd(15 downto 0) := std_logic_vector(doppler_step_r);
      when 16#1C# =>
        rd(7 downto 0) := std_logic_vector(min_cn0_dbhz_r);
      when 16#20# =>
        rd(15 downto 0) := std_logic_vector(carrier_lock_th_r);
      when 16#24# =>
        rd(7 downto 0) := std_logic_vector(max_lock_fail_r);
      when 16#40# =>
        rd(0) := acq_done_i;
        rd(1) := acq_success_i;
        rd(2) := pvt_valid_i;
        rd(3) := uart_busy_i;
      when 16#44# =>
        rd(5 downto 0) := std_logic_vector(detected_prn_i);
        rd(26 downto 16) := std_logic_vector(detected_code_i);
      when 16#48# =>
        rd(15 downto 0) := std_logic_vector(detected_dopp_i);
      when 16#4C# =>
        rd(G_NUM_CHANNELS - 1 downto 0) := chan_alloc_i;
        rd(23 downto 16) := std_logic_vector(obs_count_i);
      when 16#50# =>
        rd(G_NUM_CHANNELS - 1 downto 0) := chan_lock_i;
        rd(23 downto 16) := std_logic_vector(pvt_sats_used_i);
      when 16#54# =>
        rd := eph_valid_prn_i;
      when 16#58# =>
        rd := std_logic_vector(pvt_lat_e7_i);
      when 16#5C# =>
        rd := std_logic_vector(pvt_lon_e7_i);
      when 16#60# =>
        rd := std_logic_vector(pvt_height_mm_i);
      when 16#64# =>
        rd := std_logic_vector(pvt_cbias_i);
      when others =>
        null;
    end case;
    ctrl_rdata_r <= rd;
  end process;
end architecture;
