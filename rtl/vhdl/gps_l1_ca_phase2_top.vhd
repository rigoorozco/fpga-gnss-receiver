library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_phase2_top is
  generic (
    ADDRW          : integer := 8;
    DATAW          : integer := 32;
    G_NUM_CHANNELS : integer := 5;
    CLK_HZ         : integer := 50000000;
    UART_BAUD      : integer := 115200;
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

architecture rtl of gps_l1_ca_phase2_top is
  signal in_tvalid        : std_logic;
  signal in_tready        : std_logic := '1';
  signal in_i             : signed(15 downto 0);
  signal in_q             : signed(15 downto 0);
  signal sample_counter   : unsigned(31 downto 0);

  signal core_en          : std_logic;
  signal soft_reset_req   : std_logic;
  signal acq_rescan_pulse : std_logic;
  signal tracking_en      : std_logic;
  signal uart_en          : std_logic;
  signal nav_en           : std_logic;
  signal obs_en           : std_logic;
  signal pvt_en           : std_logic;
  signal chan_enable_mask : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal preferred_count  : unsigned(7 downto 0);
  signal prn_start_cfg    : unsigned(5 downto 0);
  signal prn_stop_cfg     : unsigned(5 downto 0);
  signal doppler_min      : signed(15 downto 0);
  signal doppler_max      : signed(15 downto 0);
  signal doppler_step     : signed(15 downto 0);
  signal detect_thresh    : unsigned(31 downto 0);
  signal min_cn0_dbhz     : unsigned(7 downto 0);
  signal carrier_lock_th  : signed(15 downto 0);
  signal max_lock_fail    : unsigned(7 downto 0);
  signal dopp_step_pullin : unsigned(15 downto 0);
  signal dopp_step_lock   : unsigned(15 downto 0);
  signal pll_bw_hz        : unsigned(15 downto 0);
  signal dll_bw_hz        : unsigned(15 downto 0);
  signal pll_bw_narrow_hz : unsigned(15 downto 0);
  signal dll_bw_narrow_hz : unsigned(15 downto 0);
  signal fll_bw_hz        : unsigned(15 downto 0);
  signal acq_coh_ms       : unsigned(7 downto 0);
  signal acq_noncoh_dwells: unsigned(7 downto 0);
  signal acq_dopp_bins    : unsigned(7 downto 0);
  signal acq_code_bins    : unsigned(7 downto 0);
  signal acq_code_step    : unsigned(10 downto 0);

  signal acq_start_pulse  : std_logic;
  signal acq_prn_start    : unsigned(5 downto 0);
  signal acq_prn_stop     : unsigned(5 downto 0);
  signal assign_valid     : std_logic;
  signal assign_ch_idx    : unsigned(7 downto 0);
  signal assign_prn       : unsigned(5 downto 0);
  signal assign_dopp      : signed(15 downto 0);
  signal assign_code      : unsigned(10 downto 0);

  signal acq_done         : std_logic;
  signal acq_success      : std_logic;
  signal acq_valid        : std_logic;
  signal acq_prn          : unsigned(5 downto 0);
  signal acq_dopp         : signed(15 downto 0);
  signal acq_code         : unsigned(10 downto 0);
  signal acq_metric       : unsigned(31 downto 0);

  signal chan_alloc       : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_state       : track_state_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_code_lock   : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_carrier_lock: std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_report_valid: std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_prn         : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_dopp        : s16_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_code        : u11_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_cn0_dbhz    : u8_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_prompt_i    : s24_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_prompt_q    : s24_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_nav_valid   : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_nav_bit     : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);

  signal eph_valid_prn    : std_logic_vector(31 downto 0);
  signal nav_word_count   : unsigned(31 downto 0);
  signal tow_seconds      : unsigned(31 downto 0);
  signal sat_x_ecef_prn   : s32_arr_t(0 to 31);
  signal sat_y_ecef_prn   : s32_arr_t(0 to 31);
  signal sat_z_ecef_prn   : s32_arr_t(0 to 31);
  signal sat_clk_corr_prn : s32_arr_t(0 to 31);

  signal epoch_tick       : std_logic := '0';
  signal ms_sample_cnt    : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;

  signal obs_valid        : std_logic;
  signal obs_epoch        : unsigned(31 downto 0);
  signal obs_count        : unsigned(7 downto 0);
  signal obs_valid_mask   : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal obs_prn          : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_dopp         : s16_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_range        : u32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_rate_mps     : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_carrier_cyc  : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_cn0_meta     : u8_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_lock_quality : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal obs_sat_x        : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_sat_y        : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_sat_z        : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_clk_corr     : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_first_prn    : unsigned(5 downto 0);
  signal obs_first_range  : unsigned(31 downto 0);

  signal pvt_valid        : std_logic;
  signal pvt_sats_used    : unsigned(7 downto 0);
  signal pvt_lat_e7       : signed(31 downto 0);
  signal pvt_lon_e7       : signed(31 downto 0);
  signal pvt_height_mm    : signed(31 downto 0);
  signal pvt_cbias        : signed(31 downto 0);
  signal pvt_resid_rms_m  : unsigned(31 downto 0);
  signal pvt_gdop_x100    : unsigned(15 downto 0);
  signal pvt_pdop_x100    : unsigned(15 downto 0);
  signal pvt_hdop_x100    : unsigned(15 downto 0);
  signal pvt_vdop_x100    : unsigned(15 downto 0);

  signal chan_evt_valid   : std_logic := '0';
  signal chan_evt_idx     : unsigned(7 downto 0) := (others => '0');
  signal chan_evt_state   : track_state_t := TRACK_IDLE;
  signal chan_evt_code_l  : std_logic := '0';
  signal chan_evt_car_l   : std_logic := '0';
  signal chan_evt_prn     : unsigned(5 downto 0) := (others => '0');
  signal chan_evt_dopp    : signed(15 downto 0) := (others => '0');
  signal chan_evt_code    : unsigned(10 downto 0) := (others => '0');
  signal chan_evt_cn0_dbhz: unsigned(7 downto 0) := (others => '0');
  signal chan_evt_nav_v   : std_logic := '0';
  signal chan_evt_nav_b   : std_logic := '0';
  signal chan_evt_rr_start: integer range 0 to G_NUM_CHANNELS - 1 := 0;

  signal rep_tx_valid     : std_logic;
  signal rep_tx_data      : std_logic_vector(7 downto 0);
  signal rep_tx_last      : std_logic;
  signal uart_tx_ready    : std_logic;
  signal uart_busy        : std_logic;
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

  ctrl_u : entity work.gps_l1_ca_ctrl_phase2
    generic map (
      ADDRW => ADDRW,
      DATAW => DATAW,
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk               => clk,
      rst_n             => rst_n,
      ctrl_wreq         => ctrl_wreq,
      ctrl_waddr        => ctrl_waddr,
      ctrl_wdata        => ctrl_wdata,
      ctrl_wack         => ctrl_wack,
      ctrl_rreq         => ctrl_rreq,
      ctrl_raddr        => ctrl_raddr,
      ctrl_rdata        => ctrl_rdata,
      ctrl_rack         => ctrl_rack,
      acq_done_i        => acq_done,
      acq_success_i     => acq_success,
      detected_prn_i    => acq_prn,
      detected_code_i   => acq_code,
      detected_dopp_i   => acq_dopp,
      chan_alloc_i      => chan_alloc,
      chan_lock_i       => chan_code_lock,
      obs_count_i       => obs_count,
      eph_valid_prn_i   => eph_valid_prn,
      pvt_valid_i       => pvt_valid,
      pvt_sats_used_i   => pvt_sats_used,
      pvt_lat_e7_i      => pvt_lat_e7,
      pvt_lon_e7_i      => pvt_lon_e7,
      pvt_height_mm_i   => pvt_height_mm,
      pvt_cbias_i       => pvt_cbias,
      pvt_resid_rms_m_i => pvt_resid_rms_m,
      pvt_gdop_x100_i   => pvt_gdop_x100,
      pvt_pdop_x100_i   => pvt_pdop_x100,
      pvt_hdop_x100_i   => pvt_hdop_x100,
      pvt_vdop_x100_i   => pvt_vdop_x100,
      uart_busy_i       => uart_busy,
      core_en_o         => core_en,
      soft_reset_req_o  => soft_reset_req,
      acq_rescan_pulse_o=> acq_rescan_pulse,
      tracking_en_o     => tracking_en,
      uart_en_o         => uart_en,
      nav_en_o          => nav_en,
      obs_en_o          => obs_en,
      pvt_en_o          => pvt_en,
      chan_enable_mask_o=> chan_enable_mask,
      preferred_count_o => preferred_count,
      prn_start_o       => prn_start_cfg,
      prn_stop_o        => prn_stop_cfg,
      doppler_min_o     => doppler_min,
      doppler_max_o     => doppler_max,
      doppler_step_o    => doppler_step,
      detect_thresh_o   => detect_thresh,
      min_cn0_dbhz_o    => min_cn0_dbhz,
      carrier_lock_th_o => carrier_lock_th,
      max_lock_fail_o   => max_lock_fail,
      dopp_step_pullin_o => dopp_step_pullin,
      dopp_step_lock_o => dopp_step_lock,
      pll_bw_hz_o       => pll_bw_hz,
      dll_bw_hz_o       => dll_bw_hz,
      pll_bw_narrow_hz_o => pll_bw_narrow_hz,
      dll_bw_narrow_hz_o => dll_bw_narrow_hz,
      fll_bw_hz_o       => fll_bw_hz,
      acq_coh_ms_o       => acq_coh_ms,
      acq_noncoh_dwells_o=> acq_noncoh_dwells,
      acq_dopp_bins_o    => acq_dopp_bins,
      acq_code_bins_o    => acq_code_bins,
      acq_code_step_o    => acq_code_step
    );

  acq_sched_u : entity work.gps_l1_ca_acq_sched
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk               => clk,
      rst_n             => rst_n and not soft_reset_req,
      core_en           => core_en,
      rescan_pulse      => acq_rescan_pulse,
      prn_start_cfg     => prn_start_cfg,
      prn_stop_cfg      => prn_stop_cfg,
      chan_enable_mask  => chan_enable_mask,
      chan_alloc_i      => chan_alloc,
      chan_lock_i       => chan_code_lock,
      chan_carrier_lock_i => chan_carrier_lock,
      acq_done_i        => acq_done,
      acq_success_i     => acq_success,
      acq_result_prn_i  => acq_prn,
      acq_result_dopp_i => acq_dopp,
      acq_result_code_i => acq_code,
      acq_result_metric_i => acq_metric,
      acq_start_pulse_o => acq_start_pulse,
      acq_prn_start_o   => acq_prn_start,
      acq_prn_stop_o    => acq_prn_stop,
      assign_valid_o    => assign_valid,
      assign_ch_idx_o   => assign_ch_idx,
      assign_prn_o      => assign_prn,
      assign_dopp_o     => assign_dopp,
      assign_code_o     => assign_code
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
      prn_start     => acq_prn_start,
      prn_stop      => acq_prn_stop,
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

  chan_bank_u : entity work.gps_l1_ca_chan_bank
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk                => clk,
      rst_n              => rst_n and not soft_reset_req,
      core_en            => core_en,
      tracking_en        => tracking_en,
      chan_enable_mask   => chan_enable_mask,
      s_valid            => in_tvalid,
      s_i                => in_i,
      s_q                => in_q,
      min_cn0_dbhz_i     => min_cn0_dbhz,
      carrier_lock_th_i  => carrier_lock_th,
      max_lock_fail_i    => max_lock_fail,
      dopp_step_pullin_i => dopp_step_pullin,
      dopp_step_lock_i   => dopp_step_lock,
      pll_bw_hz_i        => pll_bw_hz,
      dll_bw_hz_i        => dll_bw_hz,
      pll_bw_narrow_hz_i => pll_bw_narrow_hz,
      dll_bw_narrow_hz_i => dll_bw_narrow_hz,
      fll_bw_hz_i        => fll_bw_hz,
      assign_valid_i     => assign_valid,
      assign_ch_idx_i    => assign_ch_idx,
      assign_prn_i       => assign_prn,
      assign_dopp_i      => assign_dopp,
      assign_code_i      => assign_code,
      chan_alloc_o       => chan_alloc,
      chan_state_o       => chan_state,
      chan_code_lock_o   => chan_code_lock,
      chan_carrier_lock_o=> chan_carrier_lock,
      chan_report_valid_o=> chan_report_valid,
      chan_prn_o         => chan_prn,
      chan_dopp_o        => chan_dopp,
      chan_code_o        => chan_code,
      chan_cn0_dbhz_o    => chan_cn0_dbhz,
      chan_prompt_i_o    => chan_prompt_i,
      chan_prompt_q_o    => chan_prompt_q,
      chan_nav_valid_o   => chan_nav_valid,
      chan_nav_bit_o     => chan_nav_bit
    );

  nav_store_u : entity work.gps_l1_ca_nav_store
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk              => clk,
      rst_n            => rst_n and not soft_reset_req,
      nav_en_i         => nav_en,
      chan_nav_valid_i => chan_nav_valid,
      chan_nav_bit_i   => chan_nav_bit,
      chan_prn_i       => chan_prn,
      eph_valid_prn_o  => eph_valid_prn,
      nav_word_count_o => nav_word_count,
      tow_seconds_o    => tow_seconds,
      sat_x_ecef_o     => sat_x_ecef_prn,
      sat_y_ecef_o     => sat_y_ecef_prn,
      sat_z_ecef_o     => sat_z_ecef_prn,
      sat_clk_corr_m_o => sat_clk_corr_prn
    );

  observables_u : entity work.gps_l1_ca_observables
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk               => clk,
      rst_n             => rst_n and not soft_reset_req,
      obs_en_i          => obs_en,
      epoch_tick_i      => epoch_tick,
      sample_counter_i  => sample_counter,
      tow_seconds_i     => tow_seconds,
      chan_alloc_i      => chan_alloc,
      chan_code_lock_i  => chan_code_lock,
      chan_carrier_lock_i => chan_carrier_lock,
      chan_prn_i        => chan_prn,
      chan_dopp_i       => chan_dopp,
      chan_code_i       => chan_code,
      chan_cn0_dbhz_i   => chan_cn0_dbhz,
      eph_valid_prn_i   => eph_valid_prn,
      sat_x_ecef_i      => sat_x_ecef_prn,
      sat_y_ecef_i      => sat_y_ecef_prn,
      sat_z_ecef_i      => sat_z_ecef_prn,
      sat_clk_corr_m_i  => sat_clk_corr_prn,
      rx_est_valid_i    => pvt_valid,
      rx_est_lat_e7_i   => pvt_lat_e7,
      rx_est_lon_e7_i   => pvt_lon_e7,
      rx_est_height_mm_i=> pvt_height_mm,
      obs_valid_o       => obs_valid,
      obs_epoch_o       => obs_epoch,
      obs_count_o       => obs_count,
      obs_valid_mask_o  => obs_valid_mask,
      obs_prn_o         => obs_prn,
      obs_dopp_o        => obs_dopp,
      obs_range_o       => obs_range,
      obs_rate_mps_o    => obs_rate_mps,
      obs_carrier_cyc_o => obs_carrier_cyc,
      obs_cn0_dbhz_o    => obs_cn0_meta,
      obs_lock_quality_o=> obs_lock_quality,
      obs_sat_x_o       => obs_sat_x,
      obs_sat_y_o       => obs_sat_y,
      obs_sat_z_o       => obs_sat_z,
      obs_clk_corr_o    => obs_clk_corr,
      obs_first_prn_o   => obs_first_prn,
      obs_first_range_o => obs_first_range
    );

  pvt_u : entity work.gps_l1_ca_pvt
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk              => clk,
      rst_n            => rst_n and not soft_reset_req,
      pvt_en_i         => pvt_en,
      obs_valid_i      => obs_valid,
      obs_count_i      => obs_count,
      obs_valid_mask_i => obs_valid_mask,
      obs_prn_i        => obs_prn,
      obs_dopp_i       => obs_dopp,
      obs_range_i      => obs_range,
      obs_sat_x_i      => obs_sat_x,
      obs_sat_y_i      => obs_sat_y,
      obs_sat_z_i      => obs_sat_z,
      obs_clk_corr_i   => obs_clk_corr,
      pvt_valid_o      => pvt_valid,
      pvt_sats_used_o  => pvt_sats_used,
      pvt_lat_e7_o     => pvt_lat_e7,
      pvt_lon_e7_o     => pvt_lon_e7,
      pvt_height_mm_o  => pvt_height_mm,
      pvt_cbias_o      => pvt_cbias,
      pvt_resid_rms_m_o=> pvt_resid_rms_m,
      pvt_gdop_x100_o  => pvt_gdop_x100,
      pvt_pdop_x100_o  => pvt_pdop_x100,
      pvt_hdop_x100_o  => pvt_hdop_x100,
      pvt_vdop_x100_o  => pvt_vdop_x100
    );

  report_u : entity work.gps_l1_ca_report_phase2
    generic map (
      G_NUM_CHANNELS => G_NUM_CHANNELS
    )
    port map (
      clk                 => clk,
      rst_n               => rst_n and not soft_reset_req,
      report_enable_i     => uart_en,
      sample_counter_i    => sample_counter,
      chan_event_valid_i  => chan_evt_valid,
      chan_idx_i          => chan_evt_idx,
      chan_state_i        => chan_evt_state,
      chan_code_lock_i    => chan_evt_code_l,
      chan_carrier_lock_i => chan_evt_car_l,
      chan_prn_i          => chan_evt_prn,
      chan_dopp_i         => chan_evt_dopp,
      chan_code_i         => chan_evt_code,
      chan_cn0_dbhz_i     => chan_evt_cn0_dbhz,
      chan_nav_valid_i    => chan_evt_nav_v,
      chan_nav_bit_i      => chan_evt_nav_b,
      all_chan_alloc_i    => chan_alloc,
      all_chan_state_i    => chan_state,
      all_chan_code_lock_i => chan_code_lock,
      all_chan_carrier_lock_i => chan_carrier_lock,
      all_chan_prn_i      => chan_prn,
      all_chan_dopp_i     => chan_dopp,
      all_chan_code_i     => chan_code,
      all_chan_cn0_dbhz_i => chan_cn0_dbhz,
      all_chan_nav_valid_i => chan_nav_valid,
      all_chan_nav_bit_i  => chan_nav_bit,
      obs_event_valid_i   => obs_valid,
      obs_epoch_i         => obs_epoch,
      obs_count_i         => obs_count,
      obs_first_prn_i     => obs_first_prn,
      obs_first_range_i   => obs_first_range,
      pvt_event_valid_i   => pvt_valid,
      pvt_sats_used_i     => pvt_sats_used,
      pvt_lat_e7_i        => pvt_lat_e7,
      pvt_lon_e7_i        => pvt_lon_e7,
      pvt_height_mm_i     => pvt_height_mm,
      pvt_cbias_i         => pvt_cbias,
      tx_ready_i          => uart_tx_ready,
      tx_valid_o          => rep_tx_valid,
      tx_data_o           => rep_tx_data,
      tx_last_o           => rep_tx_last
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

  epoch_proc : process (clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' or soft_reset_req = '1' then
        ms_sample_cnt <= 0;
        epoch_tick    <= '0';
      else
        epoch_tick <= '0';
        if in_tvalid = '1' then
          if ms_sample_cnt = C_SAMPLES_PER_MS - 1 then
            ms_sample_cnt <= 0;
            epoch_tick    <= '1';
          else
            ms_sample_cnt <= ms_sample_cnt + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  event_pick_proc : process (clk)
    variable found_v  : boolean;
    variable idx_v    : integer;
    variable pick_idx : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        chan_evt_valid <= '0';
        chan_evt_idx   <= (others => '0');
        chan_evt_state <= TRACK_IDLE;
        chan_evt_code_l<= '0';
        chan_evt_car_l <= '0';
        chan_evt_prn   <= (others => '0');
        chan_evt_dopp  <= (others => '0');
        chan_evt_code  <= (others => '0');
        chan_evt_cn0_dbhz <= (others => '0');
        chan_evt_nav_v <= '0';
        chan_evt_nav_b <= '0';
        chan_evt_rr_start <= 0;
      else
        found_v := false;
        pick_idx := chan_evt_rr_start;
        chan_evt_valid <= '0';
        for off in 0 to G_NUM_CHANNELS - 1 loop
          idx_v := chan_evt_rr_start + off;
          if idx_v >= G_NUM_CHANNELS then
            idx_v := idx_v - G_NUM_CHANNELS;
          end if;

          if chan_report_valid(idx_v) = '1' and not found_v then
            found_v := true;
            pick_idx := idx_v;
            chan_evt_valid <= '1';
            chan_evt_idx   <= to_unsigned(idx_v, chan_evt_idx'length);
            chan_evt_state <= chan_state(idx_v);
            chan_evt_code_l<= chan_code_lock(idx_v);
            chan_evt_car_l <= chan_carrier_lock(idx_v);
            chan_evt_prn   <= chan_prn(idx_v);
            chan_evt_dopp  <= chan_dopp(idx_v);
            chan_evt_code  <= chan_code(idx_v);
            chan_evt_cn0_dbhz <= chan_cn0_dbhz(idx_v);
            chan_evt_nav_v <= chan_nav_valid(idx_v);
            chan_evt_nav_b <= chan_nav_bit(idx_v);
          end if;
        end loop;

        if found_v then
          if pick_idx = G_NUM_CHANNELS - 1 then
            chan_evt_rr_start <= 0;
          else
            chan_evt_rr_start <= pick_idx + 1;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
