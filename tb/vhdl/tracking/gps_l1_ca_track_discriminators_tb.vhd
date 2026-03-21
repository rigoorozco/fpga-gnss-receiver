library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_discriminators_tb is
end entity;

architecture tb of gps_l1_ca_track_discriminators_tb is
  signal state_s            : track_state_t := TRACK_PULLIN;
  signal prompt_i_acc_s     : signed(31 downto 0) := (others => '0');
  signal prompt_q_acc_s     : signed(31 downto 0) := (others => '0');
  signal early_i_acc_s      : signed(31 downto 0) := (others => '0');
  signal early_q_acc_s      : signed(31 downto 0) := (others => '0');
  signal late_i_acc_s       : signed(31 downto 0) := (others => '0');
  signal late_q_acc_s       : signed(31 downto 0) := (others => '0');
  signal prev_prompt_i_s    : signed(31 downto 0) := (others => '0');
  signal prev_prompt_q_s    : signed(31 downto 0) := (others => '0');
  signal prev_prompt_valid_s: std_logic := '0';

  signal prompt_mag_s       : integer;
  signal early_mag_s        : integer;
  signal late_mag_s         : integer;
  signal dll_err_q15_s      : integer;
  signal carrier_err_pll_q15_s : integer;
  signal carrier_err_fll_q15_s : integer;
  signal carrier_err_sel_q15_s : integer;
  signal prompt_i_s         : integer;
  signal prompt_q_s         : integer;
  signal early_i_s          : integer;
  signal early_q_s          : integer;
  signal late_i_s           : integer;
  signal late_q_s           : integer;

  function exp_dll_err_q15(early_mag : integer; late_mag : integer) return integer is
    variable den_v : integer;
    variable q8_v  : integer;
  begin
    den_v := early_mag + late_mag + 1;
    q8_v := ((early_mag - late_mag) * 256) / den_v;
    return clamp_i(q8_v * 128, -32767, 32767);
  end function;

  function exp_pll_err_q15(pi_v : integer; pq_v : integer) return integer is
    variable den_v : integer;
    variable q8_v  : integer;
  begin
    den_v := abs_i(pi_v) + 1;
    q8_v := (pq_v * 256) / den_v;
    return clamp_i(q8_v * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
  end function;

  function exp_fll_err_q15(prev_i_v : integer; prev_q_v : integer; curr_i_v : integer; curr_q_v : integer) return integer is
    variable cross_v : integer;
    variable dot_v   : integer;
    variable den_v   : integer;
    variable q8_v    : integer;
  begin
    cross_v := prev_i_v * curr_q_v - prev_q_v * curr_i_v;
    dot_v := prev_i_v * curr_i_v + prev_q_v * curr_q_v;
    den_v := abs_i(dot_v) + 1;
    q8_v := (cross_v * 256) / den_v;
    return clamp_i(q8_v * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
  end function;
begin
  dut : entity work.gps_l1_ca_track_discriminators
    port map (
      state_i               => state_s,
      prompt_i_acc_i        => prompt_i_acc_s,
      prompt_q_acc_i        => prompt_q_acc_s,
      early_i_acc_i         => early_i_acc_s,
      early_q_acc_i         => early_q_acc_s,
      late_i_acc_i          => late_i_acc_s,
      late_q_acc_i          => late_q_acc_s,
      prev_prompt_i_i       => prev_prompt_i_s,
      prev_prompt_q_i       => prev_prompt_q_s,
      prev_prompt_valid_i   => prev_prompt_valid_s,
      prompt_mag_o          => prompt_mag_s,
      early_mag_o           => early_mag_s,
      late_mag_o            => late_mag_s,
      dll_err_q15_o         => dll_err_q15_s,
      carrier_err_pll_q15_o => carrier_err_pll_q15_s,
      carrier_err_fll_q15_o => carrier_err_fll_q15_s,
      carrier_err_sel_q15_o => carrier_err_sel_q15_s,
      prompt_i_s_o          => prompt_i_s,
      prompt_q_s_o          => prompt_q_s,
      early_i_s_o           => early_i_s,
      early_q_s_o           => early_q_s,
      late_i_s_o            => late_i_s,
      late_q_s_o            => late_q_s
    );

  stim : process
    variable exp_prompt_mag_v : integer;
    variable exp_early_mag_v  : integer;
    variable exp_late_mag_v   : integer;
    variable exp_pll_v        : integer;
    variable exp_fll_v        : integer;
  begin
    -- Case 1: locked mode uses PLL discriminator.
    state_s <= TRACK_LOCKED;
    prev_prompt_valid_s <= '0';
    prompt_i_acc_s <= to_signed(409600, 32);
    prompt_q_acc_s <= to_signed(0, 32);
    early_i_acc_s  <= to_signed(327680, 32);
    early_q_acc_s  <= to_signed(0, 32);
    late_i_acc_s   <= to_signed(245760, 32);
    late_q_acc_s   <= to_signed(0, 32);
    wait for 1 ns;

    exp_prompt_mag_v := 409600;
    exp_early_mag_v := 327680;
    exp_late_mag_v := 245760;
    exp_pll_v := exp_pll_err_q15(100, 0);

    assert prompt_mag_s = exp_prompt_mag_v severity failure;
    assert early_mag_s = exp_early_mag_v severity failure;
    assert late_mag_s = exp_late_mag_v severity failure;
    assert dll_err_q15_s = exp_dll_err_q15(exp_early_mag_v, exp_late_mag_v) severity failure;
    assert carrier_err_pll_q15_s = exp_pll_v severity failure;
    assert carrier_err_fll_q15_s = exp_pll_v severity failure;
    assert carrier_err_sel_q15_s = exp_pll_v severity failure;

    -- Case 2: pull-in mode uses cross/dot FLL discriminator when previous prompt is valid.
    state_s <= TRACK_PULLIN;
    prev_prompt_valid_s <= '1';
    prev_prompt_i_s <= to_signed(368640, 32); -- 90 * 4096
    prev_prompt_q_s <= to_signed(81920, 32);  -- 20 * 4096
    prompt_i_acc_s <= to_signed(409600, 32);  -- 100 * 4096
    prompt_q_acc_s <= to_signed(122880, 32);  -- 30 * 4096
    early_i_acc_s  <= to_signed(286720, 32);
    early_q_acc_s  <= to_signed(81920, 32);
    late_i_acc_s   <= to_signed(245760, 32);
    late_q_acc_s   <= to_signed(40960, 32);
    wait for 1 ns;

    exp_fll_v := exp_fll_err_q15(90, 20, 100, 30);
    assert carrier_err_fll_q15_s = exp_fll_v severity failure;
    assert carrier_err_pll_q15_s = exp_fll_v severity failure;
    assert carrier_err_sel_q15_s = exp_fll_v severity failure;
    assert prompt_i_s = 100 severity failure;
    assert prompt_q_s = 30 severity failure;

    finish;
  end process;
end architecture;
