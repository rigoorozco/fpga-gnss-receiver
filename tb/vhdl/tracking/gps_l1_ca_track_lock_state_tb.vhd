library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_lock_state_tb is
end entity;

architecture tb of gps_l1_ca_track_lock_state_tb is
  signal state_s           : track_state_t := TRACK_PULLIN;
  signal prompt_mag_s      : integer := 0;
  signal cn0_dbhz_s        : integer := 0;
  signal min_cn0_dbhz_s    : unsigned(7 downto 0) := (others => '0');
  signal dll_err_q15_s     : integer := 0;
  signal carrier_metric_s  : integer := 0;
  signal carrier_err_q15_s : integer := 0;
  signal carrier_lock_th_s : signed(15 downto 0) := (others => '0');
  signal max_lock_fail_s   : unsigned(7 downto 0) := (others => '0');
  signal lock_score_s      : integer := 0;

  signal state_o_s         : track_state_t;
  signal code_lock_o_s     : std_logic;
  signal carrier_lock_o_s  : std_logic;
  signal lock_score_o_s    : integer;
begin
  dut : entity work.gps_l1_ca_track_lock_state
    port map (
      state_i           => state_s,
      prompt_mag_i      => prompt_mag_s,
      cn0_dbhz_i        => cn0_dbhz_s,
      min_cn0_dbhz_i    => min_cn0_dbhz_s,
      dll_err_q15_i     => dll_err_q15_s,
      carrier_metric_i  => carrier_metric_s,
      carrier_err_q15_i => carrier_err_q15_s,
      carrier_lock_th_i => carrier_lock_th_s,
      max_lock_fail_i   => max_lock_fail_s,
      lock_score_i      => lock_score_s,
      state_o           => state_o_s,
      code_lock_o       => code_lock_o_s,
      carrier_lock_o    => carrier_lock_o_s,
      lock_score_o      => lock_score_o_s
    );

  stim : process
  begin
    -- Enter locked from pull-in.
    state_s <= TRACK_PULLIN;
    prompt_mag_s <= 10000;
    cn0_dbhz_s <= 45;
    min_cn0_dbhz_s <= to_unsigned(20, 8);
    dll_err_q15_s <= 100;
    carrier_metric_s <= 22000;
    carrier_err_q15_s <= 100;
    carrier_lock_th_s <= to_signed(16384, 16);
    max_lock_fail_s <= to_unsigned(20, 8);
    lock_score_s <= 20;
    wait for 1 ns;

    assert state_o_s = TRACK_LOCKED severity failure;
    assert code_lock_o_s = '1' severity failure;
    assert carrier_lock_o_s = '1' severity failure;
    assert lock_score_o_s > lock_score_s severity failure;

    -- Fall back to pull-in when score decays below exit threshold.
    state_s <= TRACK_LOCKED;
    prompt_mag_s <= 0;
    cn0_dbhz_s <= 0;
    dll_err_q15_s <= 32000;
    carrier_metric_s <= 0;
    carrier_err_q15_s <= 30000;
    lock_score_s <= 2;
    wait for 1 ns;

    assert state_o_s = TRACK_PULLIN severity failure;
    assert code_lock_o_s = '0' severity failure;
    assert carrier_lock_o_s = '0' severity failure;

    finish;
  end process;
end architecture;
