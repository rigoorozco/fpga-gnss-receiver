library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_phase1_top is
  generic (
    ADDRW    : integer := 8;
    DATAW    : integer := 32;
    CLK_HZ   : integer := 50000000;
    UART_BAUD: integer := 115200;
    G_ACQ_IMPL_FFT : boolean := false
  );
  port (
    clk         : in  std_logic;
    rst_n       : in  std_logic;
    sample_valid: in  std_logic;
    sample_i    : in  signed(15 downto 0);
    sample_q    : in  signed(15 downto 0);
    sample_ready: out std_logic;

    ctrl_wreq   : in  std_logic;
    ctrl_waddr  : in  unsigned(ADDRW - 1 downto 0);
    ctrl_wdata  : in  std_logic_vector(DATAW - 1 downto 0);
    ctrl_wack   : out std_logic;
    ctrl_rreq   : in  std_logic;
    ctrl_raddr  : in  unsigned(ADDRW - 1 downto 0);
    ctrl_rdata  : out std_logic_vector(DATAW - 1 downto 0);
    ctrl_rack   : out std_logic;

    uart_txd    : out std_logic
  );
end entity;

architecture rtl of gps_l1_ca_phase1_top is
  signal in_tvalid      : std_logic;
  signal in_tready      : std_logic := '1';
  signal in_i           : signed(15 downto 0);
  signal in_q           : signed(15 downto 0);
  signal sample_counter : unsigned(31 downto 0);

  signal core_en        : std_logic;
  signal soft_reset_req : std_logic;
  signal acq_start_pulse: std_logic;
  signal tracking_en    : std_logic;
  signal uart_en        : std_logic;
  signal prn_start      : unsigned(5 downto 0);
  signal prn_stop       : unsigned(5 downto 0);
  signal doppler_min    : signed(15 downto 0);
  signal doppler_max    : signed(15 downto 0);
  signal doppler_step   : signed(15 downto 0);
  signal detect_thresh  : unsigned(31 downto 0);
  signal pll_gain       : unsigned(15 downto 0);
  signal dll_gain       : unsigned(15 downto 0);
  signal lock_thresh    : unsigned(15 downto 0);
  signal init_prn       : unsigned(5 downto 0);
  signal init_dopp      : signed(15 downto 0);

  signal acq_done       : std_logic;
  signal acq_success    : std_logic;
  signal acq_valid      : std_logic;
  signal acq_prn        : unsigned(5 downto 0);
  signal acq_dopp       : signed(15 downto 0);
  signal acq_code       : unsigned(10 downto 0);
  signal acq_metric     : unsigned(31 downto 0);

  signal track_state    : track_state_t;
  signal code_lock      : std_logic;
  signal carrier_lock   : std_logic;
  signal trk_report_v   : std_logic;
  signal trk_prn        : unsigned(5 downto 0);
  signal trk_dopp       : signed(15 downto 0);
  signal trk_code       : unsigned(10 downto 0);
  signal prompt_i       : signed(23 downto 0);
  signal prompt_q       : signed(23 downto 0);
  signal min_cn0_dbhz   : unsigned(7 downto 0) := to_unsigned(22, 8);
  signal carrier_lock_th: signed(15 downto 0) := to_signed(16384, 16);
  signal max_lock_fail  : unsigned(7 downto 0) := to_unsigned(50, 8);
  signal dopp_step_pullin : unsigned(15 downto 0);
  signal dopp_step_lock   : unsigned(15 downto 0);
  signal acq_coh_ms       : unsigned(7 downto 0) := to_unsigned(1, 8);
  signal acq_noncoh_dwells: unsigned(7 downto 0) := to_unsigned(2, 8);
  signal acq_dopp_bins    : unsigned(7 downto 0) := to_unsigned(9, 8);
  signal acq_code_bins    : unsigned(7 downto 0) := to_unsigned(8, 8);
  signal acq_code_step    : unsigned(10 downto 0) := to_unsigned(64, 11);

  signal nav_valid      : std_logic;
  signal nav_bit        : std_logic;

  signal rep_tx_valid   : std_logic;
  signal rep_tx_data    : std_logic_vector(7 downto 0);
  signal rep_tx_last    : std_logic;
  signal uart_tx_ready  : std_logic;
  signal uart_busy      : std_logic;
begin
  ingress_u : entity work.axis_sample_ingress
    port map (
      clk            => clk,
      rst_n          => rst_n and not soft_reset_req,
      in_valid       => sample_valid,
      in_i           => sample_i,
      in_q           => sample_q,
      in_ready       => sample_ready,
      out_tvalid     => in_tvalid,
      out_tready     => in_tready,
      out_i          => in_i,
      out_q          => in_q,
      sample_counter => sample_counter
    );

  ctrl_u : entity work.gps_l1_ca_ctrl
    generic map (
      ADDRW => ADDRW,
      DATAW => DATAW
    )
    port map (
      clk             => clk,
      rst_n           => rst_n,
      ctrl_wreq       => ctrl_wreq,
      ctrl_waddr      => ctrl_waddr,
      ctrl_wdata      => ctrl_wdata,
      ctrl_wack       => ctrl_wack,
      ctrl_rreq       => ctrl_rreq,
      ctrl_raddr      => ctrl_raddr,
      ctrl_rdata      => ctrl_rdata,
      ctrl_rack       => ctrl_rack,
      acq_done_i      => acq_done,
      acq_success_i   => acq_success,
      detected_prn_i  => acq_prn,
      detected_code_i => acq_code,
      detected_dopp_i => acq_dopp,
      track_state_i   => state_to_slv(track_state),
      code_lock_i     => code_lock,
      carrier_lock_i  => carrier_lock,
      nav_bit_valid_i => nav_valid,
      uart_busy_i     => uart_busy,
      core_en_o       => core_en,
      soft_reset_req_o=> soft_reset_req,
      acq_start_pulse_o => acq_start_pulse,
      tracking_en_o   => tracking_en,
      uart_en_o       => uart_en,
      prn_start_o     => prn_start,
      prn_stop_o      => prn_stop,
      doppler_min_o   => doppler_min,
      doppler_max_o   => doppler_max,
      doppler_step_o  => doppler_step,
      detect_thresh_o => detect_thresh,
      pll_gain_o      => pll_gain,
      dll_gain_o      => dll_gain,
      lock_thresh_o   => lock_thresh,
      init_prn_o      => init_prn,
      init_dopp_o     => init_dopp,
      dopp_step_pullin_o => dopp_step_pullin,
      dopp_step_lock_o => dopp_step_lock
    );

  acq_u : entity work.gps_l1_ca_acq
    generic map (
      G_ACQ_IMPL_FFT => G_ACQ_IMPL_FFT
    )
    port map (
      clk           => clk,
      rst_n         => rst_n and not soft_reset_req,
      core_en       => core_en,
      start_pulse   => acq_start_pulse,
      prn_start     => prn_start,
      prn_stop      => prn_stop,
      doppler_min   => doppler_min,
      doppler_max   => doppler_max,
      doppler_step  => doppler_step,
      detect_thresh => detect_thresh,
      coh_ms_i      => acq_coh_ms,
      noncoh_dwells_i => acq_noncoh_dwells,
      doppler_bin_count_i => acq_dopp_bins,
      code_bin_count_i => acq_code_bins,
      code_bin_step_i => acq_code_step,
      s_valid       => in_tvalid,
      s_i           => in_i,
      s_q           => in_q,
      acq_done      => acq_done,
      acq_success   => acq_success,
      result_valid  => acq_valid,
      result_prn    => acq_prn,
      result_dopp   => acq_dopp,
      result_code   => acq_code,
      result_metric => acq_metric
    );

  track_u : entity work.gps_l1_ca_track_chan
    port map (
      clk            => clk,
      rst_n          => rst_n and not soft_reset_req,
      core_en        => core_en,
      tracking_en    => tracking_en,
      init_prn       => init_prn,
      init_dopp      => init_dopp,
      acq_valid      => acq_valid,
      acq_prn        => acq_prn,
      acq_dopp       => acq_dopp,
      acq_code       => acq_code,
      s_valid        => in_tvalid,
      s_i            => in_i,
      s_q            => in_q,
      min_cn0_dbhz_i => min_cn0_dbhz,
      carrier_lock_th_i => carrier_lock_th,
      max_lock_fail_i => max_lock_fail,
      dopp_step_pullin_i => dopp_step_pullin,
      dopp_step_lock_i => dopp_step_lock,
      pll_bw_hz_i     => pll_gain,
      dll_bw_hz_i     => dll_gain,
      pll_bw_narrow_hz_i => shift_right(pll_gain, 3),
      dll_bw_narrow_hz_i => shift_right(dll_gain, 2),
      fll_bw_hz_i     => to_unsigned(2560, 16),
      track_state_o  => track_state,
      code_lock_o    => code_lock,
      carrier_lock_o => carrier_lock,
      report_valid_o => trk_report_v,
      prn_o          => trk_prn,
      dopp_o         => trk_dopp,
      code_o         => trk_code,
      cn0_dbhz_o     => open,
      prompt_i_o     => prompt_i,
      prompt_q_o     => prompt_q
    );

  nav_u : entity work.gps_l1_ca_nav
    port map (
      clk          => clk,
      rst_n        => rst_n and not soft_reset_req,
      code_lock    => code_lock,
      prompt_valid => trk_report_v,
      prompt_i     => prompt_i,
      nav_valid    => nav_valid,
      nav_bit      => nav_bit
    );

  report_u : entity work.gps_l1_ca_report
    port map (
      clk             => clk,
      rst_n           => rst_n and not soft_reset_req,
      report_enable   => uart_en,
      sample_counter  => sample_counter,
      track_state     => track_state,
      code_lock       => code_lock,
      carrier_lock    => carrier_lock,
      report_valid_in => trk_report_v,
      prn             => trk_prn,
      doppler_hz      => trk_dopp,
      code_phase      => trk_code,
      prompt_i        => prompt_i,
      prompt_q        => prompt_q,
      nav_valid       => nav_valid,
      nav_bit         => nav_bit,
      tx_ready        => uart_tx_ready,
      tx_valid        => rep_tx_valid,
      tx_data         => rep_tx_data,
      tx_last         => rep_tx_last
    );

  uart_u : entity work.uart_tx
    generic map (
      G_CLK_HZ  => CLK_HZ,
      G_BAUD_HZ => UART_BAUD
    )
    port map (
      clk      => clk,
      rst_n    => rst_n and not soft_reset_req,
      tx_valid => rep_tx_valid,
      tx_ready => uart_tx_ready,
      tx_data  => rep_tx_data,
      txd      => uart_txd,
      busy     => uart_busy
    );
end architecture;
