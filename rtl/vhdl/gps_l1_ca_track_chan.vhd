library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_nco_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_chan is
  port (
    clk            : in  std_logic;
    rst_n          : in  std_logic;
    core_en        : in  std_logic;
    tracking_en    : in  std_logic;
    init_prn       : in  unsigned(5 downto 0);
    init_dopp      : in  signed(15 downto 0);
    acq_valid      : in  std_logic;
    acq_prn        : in  unsigned(5 downto 0);
    acq_dopp       : in  signed(15 downto 0);
    acq_code       : in  unsigned(10 downto 0);
    s_valid        : in  std_logic;
    s_i            : in  signed(15 downto 0);
    s_q            : in  signed(15 downto 0);
    min_cn0_dbhz_i : in  unsigned(7 downto 0);
    carrier_lock_th_i : in signed(15 downto 0);
    max_lock_fail_i : in unsigned(7 downto 0);
    dopp_step_pullin_i : in unsigned(15 downto 0);
    dopp_step_lock_i : in unsigned(15 downto 0);
    pll_bw_hz_i     : in unsigned(15 downto 0);
    dll_bw_hz_i     : in unsigned(15 downto 0);
    pll_bw_narrow_hz_i : in unsigned(15 downto 0);
    dll_bw_narrow_hz_i : in unsigned(15 downto 0);
    fll_bw_hz_i     : in unsigned(15 downto 0);
    track_state_o  : out track_state_t;
    code_lock_o    : out std_logic;
    carrier_lock_o : out std_logic;
    report_valid_o : out std_logic;
    prn_o          : out unsigned(5 downto 0);
    dopp_o         : out signed(15 downto 0);
    code_o         : out unsigned(10 downto 0);
    cn0_dbhz_o     : out unsigned(7 downto 0);
    prompt_i_o     : out signed(23 downto 0);
    prompt_q_o     : out signed(23 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_track_chan is
  signal state_r            : track_state_t := TRACK_IDLE;
  signal prn_r              : unsigned(5 downto 0) := (others => '0');
  signal dopp_r             : signed(15 downto 0) := (others => '0');
  signal code_phase_r       : unsigned(10 downto 0) := (others => '0');

  signal code_nco_phase_r   : unsigned(31 downto 0) := (others => '0');
  signal code_fcw_r         : unsigned(31 downto 0) := C_CODE_NCO_FCW;
  signal code_loop_i_r      : signed(31 downto 0) := (others => '0');
  signal carr_nco_phase_r   : signed(31 downto 0) := (others => '0');
  signal carr_fcw_cmd_r     : signed(31 downto 0) := (others => '0');
  signal carr_loop_i_r      : signed(31 downto 0) := (others => '0');

  signal prompt_i_acc_r     : signed(31 downto 0) := (others => '0');
  signal prompt_q_acc_r     : signed(31 downto 0) := (others => '0');
  signal early_i_acc_r      : signed(31 downto 0) := (others => '0');
  signal early_q_acc_r      : signed(31 downto 0) := (others => '0');
  signal late_i_acc_r       : signed(31 downto 0) := (others => '0');
  signal late_q_acc_r       : signed(31 downto 0) := (others => '0');

  signal prompt_i_rep_r     : signed(23 downto 0) := (others => '0');
  signal prompt_q_rep_r     : signed(23 downto 0) := (others => '0');

  signal prev_prompt_i_r    : signed(31 downto 0) := (others => '0');
  signal prev_prompt_q_r    : signed(31 downto 0) := (others => '0');
  signal prev_prompt_valid_r: std_logic := '0';

  signal ms_sample_cnt_r    : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;
  signal lock_score_r       : integer range 0 to C_LOCK_SCORE_MAX := 0;
  signal cn0_sig_avg_r      : integer := 1;
  signal cn0_noise_avg_r    : integer := 1;
  signal nbd_avg_r          : integer := 0;
  signal nbp_avg_r          : integer := 1;
  signal carrier_lock_metric_r : signed(15 downto 0) := (others => '0');

  signal report_valid_r     : std_logic := '0';
  signal code_lock_r        : std_logic := '0';
  signal carrier_lock_r     : std_logic := '0';
  signal cn0_dbhz_r         : unsigned(7 downto 0) := (others => '0');

  signal prn_init_r         : std_logic := '0';
  signal prompt_chip_adv_r  : std_logic := '0';
  signal lead_chip_adv_r    : std_logic := '0';
  signal lead_seed_pending_r: std_logic := '0';
  signal prompt_chip_s      : std_logic;
  signal lead_chip_s        : std_logic;
  signal lag_chip_r         : std_logic := '0';
  signal carr_lo_i_s        : signed(15 downto 0);
  signal carr_lo_q_s        : signed(15 downto 0);
  signal carr_wipe_i_s      : signed(15 downto 0);
  signal carr_wipe_q_s      : signed(15 downto 0);

  signal p_i_s              : signed(15 downto 0);
  signal p_q_s              : signed(15 downto 0);
  signal e_i_s              : signed(15 downto 0);
  signal e_q_s              : signed(15 downto 0);
  signal l_i_s              : signed(15 downto 0);
  signal l_q_s              : signed(15 downto 0);

  signal next_prompt_i_s    : signed(31 downto 0);
  signal next_prompt_q_s    : signed(31 downto 0);
  signal next_early_i_s     : signed(31 downto 0);
  signal next_early_q_s     : signed(31 downto 0);
  signal next_late_i_s      : signed(31 downto 0);
  signal next_late_q_s      : signed(31 downto 0);

  signal prompt_mag_s       : integer;
  signal early_mag_s        : integer;
  signal late_mag_s         : integer;
  signal dll_err_q15_s      : integer;
  signal carrier_err_pll_q15_s : integer;
  signal carrier_err_fll_q15_s : integer;
  signal carrier_err_sel_q15_s : integer;

  signal prompt_i_s32_s     : integer;
  signal prompt_q_s32_s     : integer;
  signal early_i_s32_s      : integer;
  signal early_q_s32_s      : integer;
  signal late_i_s32_s       : integer;
  signal late_q_s32_s       : integer;

  signal cn0_sig_avg_next_s    : integer;
  signal cn0_noise_avg_next_s  : integer;
  signal nbd_avg_next_s        : integer;
  signal nbp_avg_next_s        : integer;
  signal cn0_dbhz_s            : integer;
  signal carrier_metric_s      : integer;

  signal code_loop_i_next_s    : signed(31 downto 0);
  signal code_fcw_next_s       : unsigned(31 downto 0);
  signal carr_loop_i_next_s    : signed(31 downto 0);
  signal carr_fcw_cmd_next_s   : signed(31 downto 0);
  signal dopp_next_s           : signed(15 downto 0);

  signal state_next_s          : track_state_t;
  signal code_lock_next_s      : std_logic;
  signal carrier_lock_next_s   : std_logic;
  signal lock_score_next_s     : integer;
begin
  track_state_o  <= state_r;
  code_lock_o    <= code_lock_r;
  carrier_lock_o <= carrier_lock_r;
  report_valid_o <= report_valid_r;
  prn_o          <= prn_r;
  dopp_o         <= dopp_r;
  code_o         <= code_phase_r;
  cn0_dbhz_o     <= cn0_dbhz_r;
  prompt_i_o     <= prompt_i_rep_r;
  prompt_q_o     <= prompt_q_rep_r;

  carr_lo_i_s <= lo_cos_q15(unsigned(carr_nco_phase_r(31 downto 22)));
  carr_lo_q_s <= -lo_sin_q15(unsigned(carr_nco_phase_r(31 downto 22)));

  carr_wipe_u : entity work.complex_mixer
    port map (
      i_in  => s_i,
      q_in  => s_q,
      lo_i  => carr_lo_i_s,
      lo_q  => carr_lo_q_s,
      i_out => carr_wipe_i_s,
      q_out => carr_wipe_q_s
    );

  prompt_prn_u : entity work.ca_prn_gen
    port map (
      clk          => clk,
      rst_n        => rst_n,
      init         => prn_init_r,
      chip_advance => prompt_chip_adv_r,
      prn          => prn_r,
      chip         => prompt_chip_s
    );

  lead_prn_u : entity work.ca_prn_gen
    port map (
      clk          => clk,
      rst_n        => rst_n,
      init         => prn_init_r,
      chip_advance => lead_chip_adv_r,
      prn          => prn_r,
      chip         => lead_chip_s
    );

  p_i_s <= -carr_wipe_i_s when prompt_chip_s = '1' else carr_wipe_i_s;
  p_q_s <= -carr_wipe_q_s when prompt_chip_s = '1' else carr_wipe_q_s;

  e_i_s <= -carr_wipe_i_s when lag_chip_r = '1' else carr_wipe_i_s;
  e_q_s <= -carr_wipe_q_s when lag_chip_r = '1' else carr_wipe_q_s;

  l_i_s <= -carr_wipe_i_s when lead_chip_s = '1' else carr_wipe_i_s;
  l_q_s <= -carr_wipe_q_s when lead_chip_s = '1' else carr_wipe_q_s;

  next_prompt_i_s <= prompt_i_acc_r + resize(p_i_s, 32);
  next_prompt_q_s <= prompt_q_acc_r + resize(p_q_s, 32);
  next_early_i_s  <= early_i_acc_r + resize(e_i_s, 32);
  next_early_q_s  <= early_q_acc_r + resize(e_q_s, 32);
  next_late_i_s   <= late_i_acc_r + resize(l_i_s, 32);
  next_late_q_s   <= late_q_acc_r + resize(l_q_s, 32);

  discrim_u : entity work.gps_l1_ca_track_discriminators
    port map (
      state_i               => state_r,
      prompt_i_acc_i        => next_prompt_i_s,
      prompt_q_acc_i        => next_prompt_q_s,
      early_i_acc_i         => next_early_i_s,
      early_q_acc_i         => next_early_q_s,
      late_i_acc_i          => next_late_i_s,
      late_q_acc_i          => next_late_q_s,
      prev_prompt_i_i       => prev_prompt_i_r,
      prev_prompt_q_i       => prev_prompt_q_r,
      prev_prompt_valid_i   => prev_prompt_valid_r,
      prompt_mag_o          => prompt_mag_s,
      early_mag_o           => early_mag_s,
      late_mag_o            => late_mag_s,
      dll_err_q15_o         => dll_err_q15_s,
      carrier_err_pll_q15_o => carrier_err_pll_q15_s,
      carrier_err_fll_q15_o => carrier_err_fll_q15_s,
      carrier_err_sel_q15_o => carrier_err_sel_q15_s,
      prompt_i_s_o          => prompt_i_s32_s,
      prompt_q_s_o          => prompt_q_s32_s,
      early_i_s_o           => early_i_s32_s,
      early_q_s_o           => early_q_s32_s,
      late_i_s_o            => late_i_s32_s,
      late_q_s_o            => late_q_s32_s
    );

  power_lock_u : entity work.gps_l1_ca_track_power_lock
    port map (
      prompt_i_s_i     => prompt_i_s32_s,
      prompt_q_s_i     => prompt_q_s32_s,
      early_i_s_i      => early_i_s32_s,
      early_q_s_i      => early_q_s32_s,
      late_i_s_i       => late_i_s32_s,
      late_q_s_i       => late_q_s32_s,
      cn0_sig_avg_i    => cn0_sig_avg_r,
      cn0_noise_avg_i  => cn0_noise_avg_r,
      nbd_avg_i        => nbd_avg_r,
      nbp_avg_i        => nbp_avg_r,
      cn0_sig_avg_o    => cn0_sig_avg_next_s,
      cn0_noise_avg_o  => cn0_noise_avg_next_s,
      nbd_avg_o        => nbd_avg_next_s,
      nbp_avg_o        => nbp_avg_next_s,
      cn0_dbhz_o       => cn0_dbhz_s,
      carrier_metric_o => carrier_metric_s
    );

  loops_u : entity work.gps_l1_ca_track_loop_filters
    port map (
      state_i               => state_r,
      dll_err_q15_i         => dll_err_q15_s,
      carrier_err_pll_q15_i => carrier_err_pll_q15_s,
      carrier_err_fll_q15_i => carrier_err_fll_q15_s,
      dopp_step_pullin_i    => dopp_step_pullin_i,
      dopp_step_lock_i      => dopp_step_lock_i,
      pll_bw_hz_i           => pll_bw_hz_i,
      dll_bw_hz_i           => dll_bw_hz_i,
      pll_bw_narrow_hz_i    => pll_bw_narrow_hz_i,
      dll_bw_narrow_hz_i    => dll_bw_narrow_hz_i,
      fll_bw_hz_i           => fll_bw_hz_i,
      code_loop_i_i         => code_loop_i_r,
      carr_loop_i_i         => carr_loop_i_r,
      code_loop_i_o         => code_loop_i_next_s,
      code_fcw_o            => code_fcw_next_s,
      carr_loop_i_o         => carr_loop_i_next_s,
      carr_fcw_cmd_o        => carr_fcw_cmd_next_s,
      dopp_o                => dopp_next_s
    );

  lock_u : entity work.gps_l1_ca_track_lock_state
    port map (
      state_i           => state_r,
      prompt_mag_i      => prompt_mag_s,
      cn0_dbhz_i        => cn0_dbhz_s,
      min_cn0_dbhz_i    => min_cn0_dbhz_i,
      dll_err_q15_i     => dll_err_q15_s,
      carrier_metric_i  => carrier_metric_s,
      carrier_err_q15_i => carrier_err_sel_q15_s,
      carrier_lock_th_i => carrier_lock_th_i,
      max_lock_fail_i   => max_lock_fail_i,
      lock_score_i      => lock_score_r,
      state_o           => state_next_s,
      code_lock_o       => code_lock_next_s,
      carrier_lock_o    => carrier_lock_next_s,
      lock_score_o      => lock_score_next_s
    );

  process (clk)
    variable next_code_v : unsigned(31 downto 0);
    variable chip_adv_v  : std_logic;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        state_r             <= TRACK_IDLE;
        prn_r               <= (others => '0');
        dopp_r              <= (others => '0');
        code_phase_r        <= (others => '0');
        code_nco_phase_r    <= (others => '0');
        code_fcw_r          <= C_CODE_NCO_FCW;
        code_loop_i_r       <= (others => '0');
        carr_nco_phase_r    <= (others => '0');
        carr_fcw_cmd_r      <= (others => '0');
        carr_loop_i_r       <= (others => '0');
        prompt_i_acc_r      <= (others => '0');
        prompt_q_acc_r      <= (others => '0');
        early_i_acc_r       <= (others => '0');
        early_q_acc_r       <= (others => '0');
        late_i_acc_r        <= (others => '0');
        late_q_acc_r        <= (others => '0');
        prompt_i_rep_r      <= (others => '0');
        prompt_q_rep_r      <= (others => '0');
        prev_prompt_i_r     <= (others => '0');
        prev_prompt_q_r     <= (others => '0');
        prev_prompt_valid_r <= '0';
        ms_sample_cnt_r     <= 0;
        lock_score_r        <= 0;
        cn0_sig_avg_r       <= 1;
        cn0_noise_avg_r     <= 1;
        nbd_avg_r           <= 0;
        nbp_avg_r           <= 1;
        carrier_lock_metric_r <= (others => '0');
        report_valid_r      <= '0';
        code_lock_r         <= '0';
        carrier_lock_r      <= '0';
        cn0_dbhz_r          <= (others => '0');
        prn_init_r          <= '0';
        prompt_chip_adv_r   <= '0';
        lead_chip_adv_r     <= '0';
        lead_seed_pending_r <= '0';
        lag_chip_r          <= '0';
      else
        report_valid_r    <= '0';
        prn_init_r        <= '0';
        prompt_chip_adv_r <= '0';
        lead_chip_adv_r   <= '0';

        if core_en = '0' or tracking_en = '0' then
          state_r               <= TRACK_IDLE;
          code_lock_r           <= '0';
          carrier_lock_r        <= '0';
          cn0_dbhz_r            <= (others => '0');
          ms_sample_cnt_r       <= 0;
          lock_score_r          <= 0;
          cn0_sig_avg_r         <= 1;
          cn0_noise_avg_r       <= 1;
          nbd_avg_r             <= 0;
          nbp_avg_r             <= 1;
          carrier_lock_metric_r <= (others => '0');
          prev_prompt_valid_r   <= '0';
          prompt_i_acc_r        <= (others => '0');
          prompt_q_acc_r        <= (others => '0');
          early_i_acc_r         <= (others => '0');
          early_q_acc_r         <= (others => '0');
          late_i_acc_r          <= (others => '0');
          late_q_acc_r          <= (others => '0');
          carr_nco_phase_r      <= (others => '0');
          carr_fcw_cmd_r        <= (others => '0');
          carr_loop_i_r         <= (others => '0');
          code_fcw_r            <= C_CODE_NCO_FCW;
          code_loop_i_r         <= (others => '0');
        else
          if acq_valid = '1' then
            prn_r               <= acq_prn;
            dopp_r              <= acq_dopp;
            code_phase_r        <= acq_code_to_chip(acq_code);
            code_nco_phase_r    <= shift_left(resize(acq_code, 32), 21);
            carr_nco_phase_r    <= (others => '0');
            state_r             <= TRACK_PULLIN;
            code_lock_r         <= '0';
            carrier_lock_r      <= '0';
            cn0_dbhz_r          <= (others => '0');
            ms_sample_cnt_r     <= 0;
            lock_score_r        <= 0;
            cn0_sig_avg_r       <= 1;
            cn0_noise_avg_r     <= 1;
            nbd_avg_r           <= 0;
            nbp_avg_r           <= 1;
            carrier_lock_metric_r <= (others => '0');
            prev_prompt_valid_r <= '0';
            prompt_i_acc_r      <= (others => '0');
            prompt_q_acc_r      <= (others => '0');
            early_i_acc_r       <= (others => '0');
            early_q_acc_r       <= (others => '0');
            late_i_acc_r        <= (others => '0');
            late_q_acc_r        <= (others => '0');
            code_fcw_r          <= C_CODE_NCO_FCW;
            code_loop_i_r       <= (others => '0');
            carr_fcw_cmd_r      <= carr_fcw_from_hz(acq_dopp);
            carr_loop_i_r       <= carr_fcw_from_hz(acq_dopp);
            prn_init_r          <= '1';
            lead_seed_pending_r <= '1';
            lag_chip_r          <= '0';
          else
            case state_r is
              when TRACK_IDLE =>
                ms_sample_cnt_r       <= 0;
                lock_score_r          <= 0;
                code_lock_r           <= '0';
                carrier_lock_r        <= '0';
                cn0_dbhz_r            <= (others => '0');
                prev_prompt_valid_r   <= '0';
                prompt_i_acc_r        <= (others => '0');
                prompt_q_acc_r        <= (others => '0');
                early_i_acc_r         <= (others => '0');
                early_q_acc_r         <= (others => '0');
                late_i_acc_r          <= (others => '0');
                late_q_acc_r          <= (others => '0');
                cn0_sig_avg_r         <= 1;
                cn0_noise_avg_r       <= 1;
                nbd_avg_r             <= 0;
                nbp_avg_r             <= 1;
                carrier_lock_metric_r <= (others => '0');
                code_fcw_r            <= C_CODE_NCO_FCW;
                code_loop_i_r         <= (others => '0');
                prn_r                 <= init_prn;
                dopp_r                <= init_dopp;
                code_phase_r          <= (others => '0');
                code_nco_phase_r      <= (others => '0');
                carr_nco_phase_r      <= (others => '0');
                carr_fcw_cmd_r        <= carr_fcw_from_hz(init_dopp);
                carr_loop_i_r         <= carr_fcw_from_hz(init_dopp);

              when TRACK_PULLIN | TRACK_LOCKED =>
                if lead_seed_pending_r = '1' then
                  lead_chip_adv_r     <= '1';
                  lead_seed_pending_r <= '0';
                end if;

                if s_valid = '1' then
                  carr_nco_phase_r <= carr_nco_phase_r + carr_fcw_cmd_r;

                  prompt_i_acc_r <= next_prompt_i_s;
                  prompt_q_acc_r <= next_prompt_q_s;
                  early_i_acc_r  <= next_early_i_s;
                  early_q_acc_r  <= next_early_q_s;
                  late_i_acc_r   <= next_late_i_s;
                  late_q_acc_r   <= next_late_q_s;

                  next_code_v := code_nco_phase_r + code_fcw_r;
                  chip_adv_v := '0';
                  if next_code_v < code_nco_phase_r then
                    chip_adv_v := '1';
                  end if;

                  code_nco_phase_r <= next_code_v;

                  if chip_adv_v = '1' then
                    prompt_chip_adv_r <= '1';
                    lead_chip_adv_r   <= '1';
                    lag_chip_r        <= prompt_chip_s;
                    if code_phase_r = to_unsigned(1022, code_phase_r'length) then
                      code_phase_r <= (others => '0');
                    else
                      code_phase_r <= code_phase_r + 1;
                    end if;
                  end if;

                  if ms_sample_cnt_r = C_SAMPLES_PER_MS - 1 then
                    cn0_sig_avg_r <= cn0_sig_avg_next_s;
                    cn0_noise_avg_r <= cn0_noise_avg_next_s;
                    nbd_avg_r <= nbd_avg_next_s;
                    nbp_avg_r <= nbp_avg_next_s;
                    carrier_lock_metric_r <= to_signed(carrier_metric_s, carrier_lock_metric_r'length);
                    cn0_dbhz_r <= to_unsigned(clamp_i(cn0_dbhz_s, 0, 99), cn0_dbhz_r'length);

                    code_loop_i_r <= code_loop_i_next_s;
                    code_fcw_r <= code_fcw_next_s;
                    carr_loop_i_r <= carr_loop_i_next_s;
                    carr_fcw_cmd_r <= carr_fcw_cmd_next_s;
                    dopp_r <= dopp_next_s;

                    state_r <= state_next_s;
                    code_lock_r <= code_lock_next_s;
                    carrier_lock_r <= carrier_lock_next_s;
                    lock_score_r <= clamp_i(lock_score_next_s, 0, C_LOCK_SCORE_MAX);

                    report_valid_r <= '1';
                    prompt_i_rep_r <= sat_s32_to_s24(next_prompt_i_s);
                    prompt_q_rep_r <= sat_s32_to_s24(next_prompt_q_s);
                    prev_prompt_i_r <= next_prompt_i_s;
                    prev_prompt_q_r <= next_prompt_q_s;
                    prev_prompt_valid_r <= '1';

                    ms_sample_cnt_r <= 0;
                    prompt_i_acc_r <= (others => '0');
                    prompt_q_acc_r <= (others => '0');
                    early_i_acc_r <= (others => '0');
                    early_q_acc_r <= (others => '0');
                    late_i_acc_r <= (others => '0');
                    late_q_acc_r <= (others => '0');
                  else
                    ms_sample_cnt_r <= ms_sample_cnt_r + 1;
                  end if;
                end if;
            end case;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
