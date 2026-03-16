library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_pvt_tb is
end entity;

architecture tb of gps_l1_ca_pvt_tb is
  constant C_CLK_PERIOD : time := 10 ns;
  constant C_NUM_CH     : integer := 4;

  signal clk              : std_logic := '0';
  signal rst_n            : std_logic := '0';
  signal pvt_en_i         : std_logic := '0';
  signal obs_valid_i      : std_logic := '0';
  signal obs_count_i      : unsigned(7 downto 0) := (others => '0');
  signal obs_valid_mask_i : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal obs_prn_i        : u6_arr_t(0 to C_NUM_CH - 1);
  signal obs_dopp_i       : s16_arr_t(0 to C_NUM_CH - 1);
  signal obs_range_i      : u32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_x_i      : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_y_i      : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_sat_z_i      : s32_arr_t(0 to C_NUM_CH - 1);
  signal obs_clk_corr_i   : s32_arr_t(0 to C_NUM_CH - 1);

  signal pvt_valid_o      : std_logic;
  signal pvt_sats_used_o  : unsigned(7 downto 0);
  signal pvt_lat_e7_o     : signed(31 downto 0);
  signal pvt_lon_e7_o     : signed(31 downto 0);
  signal pvt_height_mm_o  : signed(31 downto 0);
  signal pvt_cbias_o      : signed(31 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_pvt
    generic map (
      G_NUM_CHANNELS => C_NUM_CH,
      G_LOG_INTERNAL => false
    )
    port map (
      clk              => clk,
      rst_n            => rst_n,
      pvt_en_i         => pvt_en_i,
      obs_valid_i      => obs_valid_i,
      obs_count_i      => obs_count_i,
      obs_valid_mask_i => obs_valid_mask_i,
      obs_prn_i        => obs_prn_i,
      obs_dopp_i       => obs_dopp_i,
      obs_range_i      => obs_range_i,
      obs_sat_x_i      => obs_sat_x_i,
      obs_sat_y_i      => obs_sat_y_i,
      obs_sat_z_i      => obs_sat_z_i,
      obs_clk_corr_i   => obs_clk_corr_i,
      pvt_valid_o      => pvt_valid_o,
      pvt_sats_used_o  => pvt_sats_used_o,
      pvt_lat_e7_o     => pvt_lat_e7_o,
      pvt_lon_e7_o     => pvt_lon_e7_o,
      pvt_height_mm_o  => pvt_height_mm_o,
      pvt_cbias_o      => pvt_cbias_o
    );

  stim_proc : process
  begin
    -- Self-consistent synthetic geometry with small residuals near the default estimate.
    obs_prn_i(0) <= to_unsigned(1, 6);
    obs_prn_i(1) <= to_unsigned(2, 6);
    obs_prn_i(2) <= to_unsigned(3, 6);
    obs_prn_i(3) <= to_unsigned(4, 6);

    obs_dopp_i(0) <= (others => '0');
    obs_dopp_i(1) <= (others => '0');
    obs_dopp_i(2) <= (others => '0');
    obs_dopp_i(3) <= (others => '0');

    obs_sat_x_i(0) <= to_signed(10000000, 32);
    obs_sat_y_i(0) <= to_signed(0, 32);
    obs_sat_z_i(0) <= to_signed(0, 32);

    obs_sat_x_i(1) <= to_signed(0, 32);
    obs_sat_y_i(1) <= to_signed(10000000, 32);
    obs_sat_z_i(1) <= to_signed(0, 32);

    obs_sat_x_i(2) <= to_signed(0, 32);
    obs_sat_y_i(2) <= to_signed(0, 32);
    obs_sat_z_i(2) <= to_signed(10000000, 32);

    obs_sat_x_i(3) <= to_signed(-10000000, 32);
    obs_sat_y_i(3) <= to_signed(0, 32);
    obs_sat_z_i(3) <= to_signed(0, 32);

    obs_range_i(0) <= to_unsigned(9999001, 32);
    obs_range_i(1) <= to_unsigned(9998001, 32);
    obs_range_i(2) <= to_unsigned(9997000, 32);
    obs_range_i(3) <= to_unsigned(10001001, 32);

    obs_clk_corr_i(0) <= (others => '0');
    obs_clk_corr_i(1) <= (others => '0');
    obs_clk_corr_i(2) <= (others => '0');
    obs_clk_corr_i(3) <= (others => '0');

    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';

    pvt_en_i <= '1';
    obs_valid_i <= '1';
    obs_count_i <= to_unsigned(4, 8);
    obs_valid_mask_i <= "1111";

    for i in 0 to 7 loop
      wait until rising_edge(clk);
    end loop;

    assert pvt_valid_o = '1' report "Expected PVT valid for 4-consistent-observable geometry." severity failure;
    assert to_integer(pvt_sats_used_o) >= 4 report "Expected at least 4 satellites used in solution." severity failure;
    assert abs(to_integer(pvt_cbias_o)) < 1000000 report "Expected bounded receiver clock bias estimate." severity failure;
    assert to_integer(pvt_height_mm_o) <= 20000000 report "Expected bounded height output." severity failure;

    log_msg("gps_l1_ca_pvt_tb completed");
    wait;
  end process;
end architecture;
