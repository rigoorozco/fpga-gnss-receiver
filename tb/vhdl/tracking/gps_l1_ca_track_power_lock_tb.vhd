library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_power_lock_tb is
end entity;

architecture tb of gps_l1_ca_track_power_lock_tb is
  signal prompt_i_s     : integer := 0;
  signal prompt_q_s     : integer := 0;
  signal early_i_s      : integer := 0;
  signal early_q_s      : integer := 0;
  signal late_i_s       : integer := 0;
  signal late_q_s       : integer := 0;
  signal cn0_sig_avg_s  : integer := 1;
  signal cn0_noise_avg_s: integer := 1;
  signal nbd_avg_s      : integer := 0;
  signal nbp_avg_s      : integer := 1;

  signal cn0_sig_avg_o_s   : integer;
  signal cn0_noise_avg_o_s : integer;
  signal nbd_avg_o_s       : integer;
  signal nbp_avg_o_s       : integer;
  signal cn0_dbhz_o_s      : integer;
  signal carrier_metric_o_s: integer;

  function exp_metric(nbd_v : integer; nbp_v : integer) return integer is
    variable m_v : integer;
  begin
    m_v := (nbd_v * 32768) / nbp_v;
    if m_v > 32767 then
      return 32767;
    elsif m_v < -32768 then
      return -32768;
    else
      return m_v;
    end if;
  end function;
begin
  dut : entity work.gps_l1_ca_track_power_lock
    port map (
      prompt_i_s_i     => prompt_i_s,
      prompt_q_s_i     => prompt_q_s,
      early_i_s_i      => early_i_s,
      early_q_s_i      => early_q_s,
      late_i_s_i       => late_i_s,
      late_q_s_i       => late_q_s,
      cn0_sig_avg_i    => cn0_sig_avg_s,
      cn0_noise_avg_i  => cn0_noise_avg_s,
      nbd_avg_i        => nbd_avg_s,
      nbp_avg_i        => nbp_avg_s,
      cn0_sig_avg_o    => cn0_sig_avg_o_s,
      cn0_noise_avg_o  => cn0_noise_avg_o_s,
      nbd_avg_o        => nbd_avg_o_s,
      nbp_avg_o        => nbp_avg_o_s,
      cn0_dbhz_o       => cn0_dbhz_o_s,
      carrier_metric_o => carrier_metric_o_s
    );

  stim : process
    variable sig_pow_v      : integer;
    variable early_pow_v    : integer;
    variable late_pow_v     : integer;
    variable noise_sample_v : integer;
    variable sig_sample_v   : integer;
    variable exp_sig_avg_v  : integer;
    variable exp_noise_avg_v: integer;
    variable exp_nbd_avg_v  : integer;
    variable exp_nbp_avg_v  : integer;
    variable nbd_sample_v   : integer;
    variable nbp_sample_v   : integer;
    variable exp_cn0_v      : integer;
  begin
    prompt_i_s <= 80;
    prompt_q_s <= 20;
    early_i_s <= 64;
    early_q_s <= 16;
    late_i_s <= 52;
    late_q_s <= 10;
    cn0_sig_avg_s <= 1000;
    cn0_noise_avg_s <= 200;
    nbd_avg_s <= 100;
    nbp_avg_s <= 300;

    wait for 1 ns;

    sig_pow_v := 80 * 80 + 20 * 20;
    early_pow_v := 64 * 64 + 16 * 16;
    late_pow_v := 52 * 52 + 10 * 10;
    noise_sample_v := (early_pow_v + late_pow_v) / 2;
    sig_sample_v := sig_pow_v - noise_sample_v;
    if sig_sample_v < 1 then
      sig_sample_v := 1;
    end if;

    exp_sig_avg_v := 1000 + (sig_sample_v - 1000) / C_CN0_AVG_DIV;
    exp_noise_avg_v := 200 + (noise_sample_v - 200) / C_CN0_AVG_DIV;

    nbd_sample_v := (80 * 80) - (20 * 20);
    nbp_sample_v := (80 * 80) + (20 * 20);
    exp_nbd_avg_v := 100 + (nbd_sample_v - 100) / C_LOCK_SMOOTH_DIV;
    exp_nbp_avg_v := 300 + (nbp_sample_v - 300) / C_LOCK_SMOOTH_DIV;

    exp_cn0_v := cn0_dbhz_from_powers(exp_sig_avg_v, exp_noise_avg_v);

    assert cn0_sig_avg_o_s = exp_sig_avg_v severity failure;
    assert cn0_noise_avg_o_s = exp_noise_avg_v severity failure;
    assert nbd_avg_o_s = exp_nbd_avg_v severity failure;
    assert nbp_avg_o_s = exp_nbp_avg_v severity failure;
    assert cn0_dbhz_o_s = exp_cn0_v severity failure;
    assert carrier_metric_o_s = exp_metric(exp_nbd_avg_v, exp_nbp_avg_v) severity failure;

    finish;
  end process;
end architecture;
