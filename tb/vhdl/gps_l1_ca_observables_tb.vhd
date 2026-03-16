library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_observables_tb is
end entity;

architecture tb of gps_l1_ca_observables_tb is
  constant C_CLK_PERIOD : time := 10 ns;
  constant C_NUM_CH     : integer := 2;

  signal clk                : std_logic := '0';
  signal rst_n              : std_logic := '0';
  signal obs_en_i           : std_logic := '0';
  signal epoch_tick_i       : std_logic := '0';
  signal sample_counter_i   : unsigned(31 downto 0) := (others => '0');
  signal tow_seconds_i      : unsigned(31 downto 0) := (others => '0');
  signal chan_alloc_i       : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_code_lock_i   : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_prn_i         : u6_arr_t(0 to C_NUM_CH - 1);
  signal chan_dopp_i        : s16_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_i        : u11_arr_t(0 to C_NUM_CH - 1);
  signal eph_valid_prn_i    : std_logic_vector(31 downto 0) := (others => '0');
  signal sat_x_ecef_i       : s32_arr_t(0 to 31);
  signal sat_y_ecef_i       : s32_arr_t(0 to 31);
  signal sat_z_ecef_i       : s32_arr_t(0 to 31);
  signal sat_clk_corr_m_i   : s32_arr_t(0 to 31);
  signal rx_est_valid_i     : std_logic := '0';
  signal rx_est_lat_e7_i    : signed(31 downto 0) := (others => '0');
  signal rx_est_lon_e7_i    : signed(31 downto 0) := (others => '0');
  signal rx_est_height_mm_i : signed(31 downto 0) := (others => '0');

  signal obs_valid_o        : std_logic;
  signal obs_epoch_o        : unsigned(31 downto 0);
  signal obs_count_o        : unsigned(7 downto 0);
  signal obs_valid_mask_o   : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal obs_prn_o          : u6_arr_t(0 to C_NUM_CH - 1);
  signal obs_dopp_o         : s16_arr_t(0 to C_NUM_CH - 1);
  signal obs_range_o        : u32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_x_o        : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_y_o        : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_z_o        : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_clk_corr_o     : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_first_prn_o    : unsigned(5 downto 0);
  signal obs_first_range_o  : unsigned(31 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_observables
    generic map (
      G_NUM_CHANNELS          => C_NUM_CH,
      G_REQUIRE_EPH_FOR_VALID => true,
      G_IONO_CORR_MM          => 0,
      G_TROPO_CORR_MM         => 0
    )
    port map (
      clk                => clk,
      rst_n              => rst_n,
      obs_en_i           => obs_en_i,
      epoch_tick_i       => epoch_tick_i,
      sample_counter_i   => sample_counter_i,
      tow_seconds_i      => tow_seconds_i,
      chan_alloc_i       => chan_alloc_i,
      chan_code_lock_i   => chan_code_lock_i,
      chan_prn_i         => chan_prn_i,
      chan_dopp_i        => chan_dopp_i,
      chan_code_i        => chan_code_i,
      eph_valid_prn_i    => eph_valid_prn_i,
      sat_x_ecef_i       => sat_x_ecef_i,
      sat_y_ecef_i       => sat_y_ecef_i,
      sat_z_ecef_i       => sat_z_ecef_i,
      sat_clk_corr_m_i   => sat_clk_corr_m_i,
      rx_est_valid_i     => rx_est_valid_i,
      rx_est_lat_e7_i    => rx_est_lat_e7_i,
      rx_est_lon_e7_i    => rx_est_lon_e7_i,
      rx_est_height_mm_i => rx_est_height_mm_i,
      obs_valid_o        => obs_valid_o,
      obs_epoch_o        => obs_epoch_o,
      obs_count_o        => obs_count_o,
      obs_valid_mask_o   => obs_valid_mask_o,
      obs_prn_o          => obs_prn_o,
      obs_dopp_o         => obs_dopp_o,
      obs_range_o        => obs_range_o,
      obs_sat_x_o        => obs_sat_x_o,
      obs_sat_y_o        => obs_sat_y_o,
      obs_sat_z_o        => obs_sat_z_o,
      obs_clk_corr_o     => obs_clk_corr_o,
      obs_first_prn_o    => obs_first_prn_o,
      obs_first_range_o  => obs_first_range_o
    );

  stim_proc : process
    variable dopp_i : integer;
    variable rng_i  : integer;
  begin
    for p in 0 to 31 loop
      sat_x_ecef_i(p) <= (others => '0');
      sat_y_ecef_i(p) <= (others => '0');
      sat_z_ecef_i(p) <= (others => '0');
      sat_clk_corr_m_i(p) <= (others => '0');
    end loop;

    chan_prn_i(0) <= to_unsigned(1, 6);
    chan_prn_i(1) <= to_unsigned(2, 6);
    chan_dopp_i(0) <= to_signed(1000, 16);
    chan_dopp_i(1) <= (others => '0');
    chan_code_i(0) <= (others => '0');
    chan_code_i(1) <= (others => '0');

    sat_x_ecef_i(0) <= to_signed(20000000, 32);
    sat_y_ecef_i(0) <= (others => '0');
    sat_z_ecef_i(0) <= (others => '0');

    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';

    obs_en_i <= '1';
    chan_alloc_i <= "01";
    chan_code_lock_i <= "01";
    eph_valid_prn_i(0) <= '1';

    -- Epoch 1: valid observation expected on channel 0.
    sample_counter_i <= to_unsigned(2000, 32);
    epoch_tick_i <= '1';
    wait until rising_edge(clk);
    epoch_tick_i <= '0';
    wait until rising_edge(clk);

    assert obs_valid_o = '1' report "Expected observables valid pulse for valid locked/eph channel." severity failure;
    assert to_integer(obs_count_o) = 1 report "Expected exactly one valid observable." severity failure;
    assert obs_valid_mask_o(0) = '1' and obs_valid_mask_o(1) = '0'
      report "Expected valid mask only on channel 0." severity failure;
    assert to_integer(obs_first_prn_o) = 1 report "Expected first valid PRN to be 1." severity failure;

    rng_i := to_integer(obs_first_range_o);
    assert rng_i > 10000000 and rng_i < 50000000
      report "Expected pseudorange in plausible GNSS range." severity failure;

    dopp_i := to_integer(obs_dopp_o(0));
    assert dopp_i > 850 and dopp_i < 1150
      report "Expected Doppler observable to stay near input Doppler on first epoch." severity failure;

    -- Epoch 2: remove ephemeris validity and expect no valid observations.
    eph_valid_prn_i(0) <= '0';
    sample_counter_i <= to_unsigned(4000, 32);
    epoch_tick_i <= '1';
    wait until rising_edge(clk);
    epoch_tick_i <= '0';
    wait until rising_edge(clk);

    assert obs_valid_o = '0' report "Expected no observables-valid pulse without ephemeris." severity failure;
    assert to_integer(obs_count_o) = 0 report "Expected zero valid observations without ephemeris." severity failure;
    assert obs_valid_mask_o = "00" report "Expected all observable mask bits clear without ephemeris." severity failure;

    log_msg("gps_l1_ca_observables_tb completed");
    wait;
  end process;
end architecture;
