library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_power_lock is
  port (
    prompt_i_s_i     : in  integer;
    prompt_q_s_i     : in  integer;
    early_i_s_i      : in  integer;
    early_q_s_i      : in  integer;
    late_i_s_i       : in  integer;
    late_q_s_i       : in  integer;
    cn0_sig_avg_i    : in  integer;
    cn0_noise_avg_i  : in  integer;
    nbd_avg_i        : in  integer;
    nbp_avg_i        : in  integer;
    cn0_sig_avg_o    : out integer;
    cn0_noise_avg_o  : out integer;
    nbd_avg_o        : out integer;
    nbp_avg_o        : out integer;
    cn0_dbhz_o       : out integer;
    carrier_metric_o : out integer
  );
end entity;

architecture rtl of gps_l1_ca_track_power_lock is
begin
  process (all)
    variable sig_pow_v          : integer;
    variable early_pow_v        : integer;
    variable late_pow_v         : integer;
    variable noise_sample_v     : integer;
    variable sig_sample_v       : integer;
    variable cn0_sig_avg_v      : integer;
    variable cn0_noise_avg_v    : integer;
    variable nbd_sample_v       : integer;
    variable nbp_sample_v       : integer;
    variable nbd_avg_v          : integer;
    variable nbp_avg_v          : integer;
    variable metric_v           : integer;
    variable cn0_v              : integer;
  begin
    sig_pow_v := prompt_i_s_i * prompt_i_s_i + prompt_q_s_i * prompt_q_s_i;
    early_pow_v := early_i_s_i * early_i_s_i + early_q_s_i * early_q_s_i;
    late_pow_v := late_i_s_i * late_i_s_i + late_q_s_i * late_q_s_i;

    noise_sample_v := (early_pow_v + late_pow_v) / 2;
    if noise_sample_v < 1 then
      noise_sample_v := 1;
    end if;

    sig_sample_v := sig_pow_v - noise_sample_v;
    if sig_sample_v < 1 then
      sig_sample_v := 1;
    end if;

    cn0_sig_avg_v := cn0_sig_avg_i + (sig_sample_v - cn0_sig_avg_i) / C_CN0_AVG_DIV;
    cn0_noise_avg_v := cn0_noise_avg_i + (noise_sample_v - cn0_noise_avg_i) / C_CN0_AVG_DIV;

    if cn0_sig_avg_v < 1 then
      cn0_sig_avg_v := 1;
    end if;
    if cn0_noise_avg_v < 1 then
      cn0_noise_avg_v := 1;
    end if;

    cn0_v := cn0_dbhz_from_powers(cn0_sig_avg_v, cn0_noise_avg_v);

    nbd_sample_v := (prompt_i_s_i * prompt_i_s_i) - (prompt_q_s_i * prompt_q_s_i);
    nbp_sample_v := (prompt_i_s_i * prompt_i_s_i) + (prompt_q_s_i * prompt_q_s_i);
    if nbp_sample_v < 1 then
      nbp_sample_v := 1;
    end if;

    nbd_avg_v := nbd_avg_i + (nbd_sample_v - nbd_avg_i) / C_LOCK_SMOOTH_DIV;
    nbp_avg_v := nbp_avg_i + (nbp_sample_v - nbp_avg_i) / C_LOCK_SMOOTH_DIV;
    if nbp_avg_v < 1 then
      nbp_avg_v := 1;
    end if;

    metric_v := (nbd_avg_v * 32768) / nbp_avg_v;
    if metric_v > 32767 then
      metric_v := 32767;
    elsif metric_v < -32768 then
      metric_v := -32768;
    end if;

    cn0_sig_avg_o <= cn0_sig_avg_v;
    cn0_noise_avg_o <= cn0_noise_avg_v;
    nbd_avg_o <= nbd_avg_v;
    nbp_avg_o <= nbp_avg_v;
    cn0_dbhz_o <= clamp_i(cn0_v, 0, 99);
    carrier_metric_o <= metric_v;
  end process;
end architecture;
