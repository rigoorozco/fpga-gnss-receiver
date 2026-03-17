library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_l1_ca_acq is
  generic (
    G_DWELL_MS     : integer := 2;
    G_ACQ_IMPL_FFT : boolean := false
  );
  port (
    clk                  : in  std_logic;
    rst_n                : in  std_logic;
    core_en              : in  std_logic;
    start_pulse          : in  std_logic;
    prn_start            : in  unsigned(5 downto 0);
    prn_stop             : in  unsigned(5 downto 0);
    doppler_min          : in  signed(15 downto 0);
    doppler_max          : in  signed(15 downto 0);
    doppler_step         : in  signed(15 downto 0);
    detect_thresh        : in  unsigned(31 downto 0);
    coh_ms_i             : in  unsigned(7 downto 0);
    noncoh_dwells_i      : in  unsigned(7 downto 0);
    doppler_bin_count_i  : in  unsigned(7 downto 0);
    code_bin_count_i     : in  unsigned(7 downto 0);
    code_bin_step_i      : in  unsigned(10 downto 0);
    s_valid              : in  std_logic;
    s_i                  : in  signed(15 downto 0);
    s_q                  : in  signed(15 downto 0);
    acq_done             : out std_logic;
    acq_success          : out std_logic;
    result_valid         : out std_logic;
    result_prn           : out unsigned(5 downto 0);
    result_dopp          : out signed(15 downto 0);
    result_code          : out unsigned(10 downto 0);
    result_metric        : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_acq is
begin
  gen_fft : if G_ACQ_IMPL_FFT generate
    impl_fft_u : entity work.gps_l1_ca_acq_fft
      generic map (
        G_DWELL_MS => G_DWELL_MS
      )
      port map (
        clk                 => clk,
        rst_n               => rst_n,
        core_en             => core_en,
        start_pulse         => start_pulse,
        prn_start           => prn_start,
        prn_stop            => prn_stop,
        doppler_min         => doppler_min,
        doppler_max         => doppler_max,
        doppler_step        => doppler_step,
        detect_thresh       => detect_thresh,
        coh_ms_i            => coh_ms_i,
        noncoh_dwells_i     => noncoh_dwells_i,
        doppler_bin_count_i => doppler_bin_count_i,
        code_bin_count_i    => code_bin_count_i,
        code_bin_step_i     => code_bin_step_i,
        s_valid             => s_valid,
        s_i                 => s_i,
        s_q                 => s_q,
        acq_done            => acq_done,
        acq_success         => acq_success,
        result_valid        => result_valid,
        result_prn          => result_prn,
        result_dopp         => result_dopp,
        result_code         => result_code,
        result_metric       => result_metric
      );
  end generate;

  gen_td : if not G_ACQ_IMPL_FFT generate
    impl_td_u : entity work.gps_l1_ca_acq_td
      generic map (
        G_DWELL_MS => G_DWELL_MS
      )
      port map (
        clk                 => clk,
        rst_n               => rst_n,
        core_en             => core_en,
        start_pulse         => start_pulse,
        prn_start           => prn_start,
        prn_stop            => prn_stop,
        doppler_min         => doppler_min,
        doppler_max         => doppler_max,
        doppler_step        => doppler_step,
        detect_thresh       => detect_thresh,
        coh_ms_i            => coh_ms_i,
        noncoh_dwells_i     => noncoh_dwells_i,
        doppler_bin_count_i => doppler_bin_count_i,
        code_bin_count_i    => code_bin_count_i,
        code_bin_step_i     => code_bin_step_i,
        s_valid             => s_valid,
        s_i                 => s_i,
        s_q                 => s_q,
        acq_done            => acq_done,
        acq_success         => acq_success,
        result_valid        => result_valid,
        result_prn          => result_prn,
        result_dopp         => result_dopp,
        result_code         => result_code,
        result_metric       => result_metric
      );
  end generate;
end architecture;
