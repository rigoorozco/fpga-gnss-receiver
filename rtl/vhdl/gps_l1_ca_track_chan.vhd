library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_nco_pkg.all;

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
  type int_arr_t is array (natural range <>) of integer;

  constant C_PROMPT_MAG_MIN     : integer := 2500;
  constant C_CARR_LOCK_HYST_Q15 : integer := 2048; -- ~0.0625 hysteresis on lock metric.
  constant C_DLL_ERR_LOCK_MAX   : integer := 40000;
  constant C_CARR_ERR_LOCK_MAX  : integer := 40000;
  constant C_DLL_ERR_TRACK_MAX  : integer := 40000;
  constant C_CARR_ERR_TRACK_MAX : integer := 40000;
  constant C_CARR_ERR_NORM_GAIN : integer := 256; -- Q15 normalizer helper.
  constant C_LOCK_SCORE_MAX     : integer := 255;
  constant C_LOCK_SCORE_INC_BOTH: integer := 4;
  constant C_LOCK_SCORE_INC_CODE: integer := 2;
  constant C_LOCK_SCORE_DEC_CODE: integer := 6;
  constant C_LOCK_SCORE_DEC_CARR: integer := 2;
  constant C_CN0_AVG_DIV        : integer := 16;
  constant C_LOCK_SMOOTH_DIV    : integer := 8;
  constant C_CODE_LOOP_KP_PER_HZ_Q8 : integer := 32768;
  constant C_CODE_LOOP_KI_DIV    : integer := 128;
  constant C_PLL_LOOP_KI_DIV     : integer := 256;
  constant C_FLL_LOOP_STEP_SCALE : integer := 4;
  constant C_PULLIN_PLL_ASSIST_DIV : integer := 4;
  constant C_PHASE_ERR_MAX_Q15   : integer := 24576;
  constant C_CARR_FCW_DELTA_MAX  : integer := 64000000;
  constant C_PLL_GAIN_MIN_Q8_8   : integer := 64;  -- 0.25 Hz
  constant C_DLL_GAIN_MIN_Q8_8   : integer := 16;  -- 0.0625 Hz
  constant C_FLL_GAIN_MIN_Q8_8   : integer := 64;  -- 0.25 Hz
  constant C_CODE_FCW_STEP      : unsigned(31 downto 0) := x"00010000";
  constant C_CODE_FCW_DELTA_MAX : unsigned(31 downto 0) := x"00200000";
  constant C_CARR_FCW_PER_HZ    : integer := 2147;
  -- 10*log10(1 + n/16) in dB*100 for n=0..16.
  constant C_LOG10_OCTAVE_LUT_DB100 : int_arr_t(0 to 16) := (
    0, 26, 51, 75, 97, 118, 138, 158, 176, 194, 211, 227, 243, 258, 273, 287, 301
  );

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

  function abs_s32(x : signed(31 downto 0)) return unsigned is
    variable v : signed(31 downto 0);
  begin
    if x < 0 then
      v := -x;
    else
      v := x;
    end if;
    return unsigned(v);
  end function;

  function abs_i(x : integer) return integer is
  begin
    if x < 0 then
      return -x;
    end if;
    return x;
  end function;

  function clamp_s16(x : integer) return signed is
  begin
    if x > 32767 then
      return to_signed(32767, 16);
    elsif x < -32768 then
      return to_signed(-32768, 16);
    else
      return to_signed(x, 16);
    end if;
  end function;

  function sat_s32_to_s24(x : signed(31 downto 0)) return signed is
    constant C_MAX24 : signed(31 downto 0) := to_signed(8388607, 32);
    constant C_MIN24 : signed(31 downto 0) := to_signed(-8388608, 32);
  begin
    if x > C_MAX24 then
      return to_signed(8388607, 24);
    elsif x < C_MIN24 then
      return to_signed(-8388608, 24);
    else
      return resize(x, 24);
    end if;
  end function;

  function acq_code_to_chip(code_v : unsigned(10 downto 0)) return unsigned is
    variable code_i : integer;
  begin
    code_i := to_integer(code_v) mod 1023;
    if code_i < 0 then
      code_i := 0;
    end if;
    return to_unsigned(code_i, 11);
  end function;

  function carr_fcw_from_hz(dopp_hz : signed(15 downto 0)) return signed is
  begin
    return to_signed(to_integer(dopp_hz) * C_CARR_FCW_PER_HZ, 32);
  end function;

  function clamp_i(x : integer; lo : integer; hi : integer) return integer is
  begin
    if x < lo then
      return lo;
    elsif x > hi then
      return hi;
    else
      return x;
    end if;
  end function;

  function carr_fcw_from_hz_i(dopp_hz : integer) return integer is
  begin
    return dopp_hz * C_CARR_FCW_PER_HZ;
  end function;

  function dopp_hz_from_carr_fcw(fcw_v : signed(31 downto 0)) return signed is
    variable hz_i : integer;
  begin
    hz_i := to_integer(fcw_v) / C_CARR_FCW_PER_HZ;
    return clamp_s16(hz_i);
  end function;

  function ten_log10_db100(x : integer) return integer is
    variable x_i            : integer;
    variable base_i         : integer;
    variable octave_i       : integer;
    variable frac_q10_i     : integer;
    variable seg_i          : integer;
    variable seg_frac_i     : integer;
    variable y0_i           : integer;
    variable y1_i           : integer;
    variable interp_i       : integer;
    variable log10_db100_i  : integer;
  begin
    if x <= 0 then
      return 0;
    end if;

    x_i := x;
    base_i := 1;
    octave_i := 0;
    for i in 0 to 30 loop
      if base_i <= x_i / 2 then
        base_i := base_i * 2;
        octave_i := octave_i + 1;
      end if;
    end loop;

    frac_q10_i := ((x_i - base_i) * 1024) / base_i;
    if frac_q10_i < 0 then
      frac_q10_i := 0;
    elsif frac_q10_i > 1023 then
      frac_q10_i := 1023;
    end if;

    seg_i := frac_q10_i / 64;
    if seg_i < 0 then
      seg_i := 0;
    elsif seg_i > 15 then
      seg_i := 15;
    end if;
    seg_frac_i := frac_q10_i - (seg_i * 64);

    y0_i := C_LOG10_OCTAVE_LUT_DB100(seg_i);
    y1_i := C_LOG10_OCTAVE_LUT_DB100(seg_i + 1);
    interp_i := y0_i + (((y1_i - y0_i) * seg_frac_i + 32) / 64);

    log10_db100_i := (octave_i * 301) + interp_i;
    return log10_db100_i;
  end function;

  function cn0_dbhz_from_powers(sig_pow : integer; noise_pow : integer) return integer is
    variable sig_i      : integer;
    variable noise_i    : integer;
    variable cn0_db100_i: integer;
    variable cn0_i      : integer;
  begin
    if sig_pow <= 0 then
      return 0;
    end if;

    sig_i := sig_pow;
    noise_i := noise_pow;
    if noise_i < 1 then
      noise_i := 1;
    end if;

    -- C/N0[dB-Hz] = 10*log10(C/N) + 30 for 1 ms coherent integration.
    cn0_db100_i := ten_log10_db100(sig_i) - ten_log10_db100(noise_i) + 3000;
    cn0_i := (cn0_db100_i + 50) / 100;
    if cn0_i < 0 then
      return 0;
    elsif cn0_i > 99 then
      return 99;
    else
      return cn0_i;
    end if;
  end function;
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
  carr_lo_i_s    <= lo_cos_q15(unsigned(carr_nco_phase_r(31 downto 22)));
  carr_lo_q_s    <= -lo_sin_q15(unsigned(carr_nco_phase_r(31 downto 22)));

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

  process (clk)
    variable p_i                : signed(15 downto 0);
    variable p_q                : signed(15 downto 0);
    variable e_i                : signed(15 downto 0);
    variable e_q                : signed(15 downto 0);
    variable l_i                : signed(15 downto 0);
    variable l_q                : signed(15 downto 0);
    variable next_prompt_i      : signed(31 downto 0);
    variable next_prompt_q      : signed(31 downto 0);
    variable next_early_i       : signed(31 downto 0);
    variable next_early_q       : signed(31 downto 0);
    variable next_late_i        : signed(31 downto 0);
    variable next_late_q        : signed(31 downto 0);
    variable next_code          : unsigned(31 downto 0);
    variable next_carr_phase    : signed(31 downto 0);
    variable carr_fcw_v         : signed(31 downto 0);
    variable chip_adv_v         : std_logic;
    variable prompt_mag_i       : integer;
    variable early_mag_i        : integer;
    variable late_mag_i         : integer;
    variable dll_err_i          : integer;
    variable carrier_err_i      : integer;
    variable prev_i_s           : integer;
    variable prev_q_s           : integer;
    variable curr_i_s           : integer;
    variable curr_q_s           : integer;
    variable cross_i            : integer;
    variable dot_i              : integer;
    variable carrier_err_norm_den_i : integer;
    variable carrier_err_pll_q15_i : integer;
    variable carrier_err_fll_q15_i : integer;
    variable dll_err_q15_i      : integer;
    variable code_enter_v       : boolean;
    variable code_track_v       : boolean;
    variable carrier_enter_v    : boolean;
    variable carrier_track_v    : boolean;
    variable fcw_floor          : unsigned(31 downto 0);
    variable fcw_ceil           : unsigned(31 downto 0);
    variable dopp_i             : integer;
    variable carr_fcw_i         : integer;
    variable carr_fcw_max_step_i: integer;
    variable code_fcw_i         : integer;
    variable code_delta_i       : integer;
    variable code_prop_i        : integer;
    variable code_kp_i          : integer;
    variable code_ki_i          : integer;
    variable carr_prop_i        : integer;
    variable carr_kp_i          : integer;
    variable carr_ki_i          : integer;
    variable fll_step_i         : integer;
    variable pll_assist_step_i  : integer;
    variable fll_gain_i         : integer;
    variable pll_bw_sel_i       : integer;
    variable dll_bw_sel_i       : integer;
    variable den_i              : integer;
    variable ratio_q8_i         : integer;
    variable prompt_i_s         : integer;
    variable prompt_q_s         : integer;
    variable early_i_s          : integer;
    variable early_q_s          : integer;
    variable late_i_s           : integer;
    variable late_q_s           : integer;
    variable sig_pow_i          : integer;
    variable early_pow_i        : integer;
    variable late_pow_i         : integer;
    variable cn0_sig_sample_i   : integer;
    variable cn0_noise_sample_i : integer;
    variable cn0_sig_avg_i      : integer;
    variable cn0_noise_avg_i    : integer;
    variable snr_num_i          : integer;
    variable snr_den_i          : integer;
    variable nbd_sample_i       : integer;
    variable nbp_sample_i       : integer;
    variable nbd_avg_i          : integer;
    variable nbp_avg_i          : integer;
    variable carrier_metric_i   : integer;
    variable carrier_metric_eval_i : integer;
    variable carrier_enter_th_i : integer;
    variable carrier_track_th_i : integer;
    variable max_lock_fail_v    : integer;
    variable lock_enter_th_i    : integer;
    variable lock_exit_th_i     : integer;
    variable lock_score_i       : integer;
    variable cn0_inst_i         : integer;
    variable cn0_i              : integer;
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
          state_r             <= TRACK_IDLE;
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
          carr_nco_phase_r    <= (others => '0');
          carr_fcw_cmd_r      <= (others => '0');
          carr_loop_i_r       <= (others => '0');
          code_fcw_r          <= C_CODE_NCO_FCW;
          code_loop_i_r       <= (others => '0');
        else
          if acq_valid = '1' then
            -- Allow channel retune/reassignment at runtime when scheduler picks a stronger PRN.
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
                ms_sample_cnt_r     <= 0;
                lock_score_r        <= 0;
                code_lock_r         <= '0';
                carrier_lock_r      <= '0';
                cn0_dbhz_r          <= (others => '0');
                prev_prompt_valid_r <= '0';
                prompt_i_acc_r      <= (others => '0');
                prompt_q_acc_r      <= (others => '0');
                early_i_acc_r       <= (others => '0');
                early_q_acc_r       <= (others => '0');
                late_i_acc_r        <= (others => '0');
                late_q_acc_r        <= (others => '0');
                cn0_sig_avg_r       <= 1;
                cn0_noise_avg_r     <= 1;
                nbd_avg_r           <= 0;
                nbp_avg_r           <= 1;
                carrier_lock_metric_r <= (others => '0');
                code_fcw_r          <= C_CODE_NCO_FCW;
                code_loop_i_r       <= (others => '0');
                prn_r               <= init_prn;
                dopp_r              <= init_dopp;
                code_phase_r        <= (others => '0');
                code_nco_phase_r    <= (others => '0');
                carr_nco_phase_r    <= (others => '0');
                carr_fcw_cmd_r      <= carr_fcw_from_hz(init_dopp);
                carr_loop_i_r       <= carr_fcw_from_hz(init_dopp);

              when TRACK_PULLIN | TRACK_LOCKED =>
              if lead_seed_pending_r = '1' then
                lead_chip_adv_r     <= '1';
                lead_seed_pending_r <= '0';
              end if;

              if s_valid = '1' then
                carr_fcw_v := carr_fcw_cmd_r;
                next_carr_phase := carr_nco_phase_r + carr_fcw_v;
                carr_nco_phase_r <= next_carr_phase;

                if prompt_chip_s = '1' then
                  p_i := -carr_wipe_i_s;
                  p_q := -carr_wipe_q_s;
                else
                  p_i := carr_wipe_i_s;
                  p_q := carr_wipe_q_s;
                end if;

                if lag_chip_r = '1' then
                  e_i := -carr_wipe_i_s;
                  e_q := -carr_wipe_q_s;
                else
                  e_i := carr_wipe_i_s;
                  e_q := carr_wipe_q_s;
                end if;

                if lead_chip_s = '1' then
                  l_i := -carr_wipe_i_s;
                  l_q := -carr_wipe_q_s;
                else
                  l_i := carr_wipe_i_s;
                  l_q := carr_wipe_q_s;
                end if;

                next_prompt_i := prompt_i_acc_r + resize(p_i, prompt_i_acc_r'length);
                next_prompt_q := prompt_q_acc_r + resize(p_q, prompt_q_acc_r'length);
                next_early_i  := early_i_acc_r + resize(e_i, early_i_acc_r'length);
                next_early_q  := early_q_acc_r + resize(e_q, early_q_acc_r'length);
                next_late_i   := late_i_acc_r + resize(l_i, late_i_acc_r'length);
                next_late_q   := late_q_acc_r + resize(l_q, late_q_acc_r'length);

                prompt_i_acc_r <= next_prompt_i;
                prompt_q_acc_r <= next_prompt_q;
                early_i_acc_r  <= next_early_i;
                early_q_acc_r  <= next_early_q;
                late_i_acc_r   <= next_late_i;
                late_q_acc_r   <= next_late_q;

                next_code := code_nco_phase_r + code_fcw_r;
                chip_adv_v := '0';
                if next_code < code_nco_phase_r then
                  chip_adv_v := '1';
                end if;

                code_nco_phase_r <= next_code;

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
                  prompt_mag_i := to_integer(abs_s32(next_prompt_i) + abs_s32(next_prompt_q));
                  early_mag_i  := to_integer(abs_s32(next_early_i) + abs_s32(next_early_q));
                  late_mag_i   := to_integer(abs_s32(next_late_i) + abs_s32(next_late_q));
                  dll_err_i    := early_mag_i - late_mag_i;

                  -- FPGA-friendly C/N0 estimator from prompt vs early/late power.
                  prompt_i_s := to_integer(shift_right(next_prompt_i, 12));
                  prompt_q_s := to_integer(shift_right(next_prompt_q, 12));
                  early_i_s := to_integer(shift_right(next_early_i, 12));
                  early_q_s := to_integer(shift_right(next_early_q, 12));
                  late_i_s := to_integer(shift_right(next_late_i, 12));
                  late_q_s := to_integer(shift_right(next_late_q, 12));
                  sig_pow_i := prompt_i_s * prompt_i_s + prompt_q_s * prompt_q_s;
                  early_pow_i := early_i_s * early_i_s + early_q_s * early_q_s;
                  late_pow_i := late_i_s * late_i_s + late_q_s * late_q_s;
                  cn0_noise_sample_i := (early_pow_i + late_pow_i) / 2;
                  if cn0_noise_sample_i < 1 then
                    cn0_noise_sample_i := 1;
                  end if;
                  cn0_sig_sample_i := sig_pow_i - cn0_noise_sample_i;
                  if cn0_sig_sample_i < 1 then
                    cn0_sig_sample_i := 1;
                  end if;

                  cn0_sig_avg_i := cn0_sig_avg_r + (cn0_sig_sample_i - cn0_sig_avg_r) / C_CN0_AVG_DIV;
                  cn0_noise_avg_i := cn0_noise_avg_r + (cn0_noise_sample_i - cn0_noise_avg_r) / C_CN0_AVG_DIV;
                  if cn0_sig_avg_i < 1 then
                    cn0_sig_avg_i := 1;
                  end if;
                  if cn0_noise_avg_i < 1 then
                    cn0_noise_avg_i := 1;
                  end if;
                  cn0_sig_avg_r <= cn0_sig_avg_i;
                  cn0_noise_avg_r <= cn0_noise_avg_i;

                  snr_num_i := cn0_sig_avg_i;
                  snr_den_i := cn0_noise_avg_i;
                  cn0_inst_i := cn0_dbhz_from_powers(snr_num_i, snr_den_i);
                  cn0_i := cn0_inst_i;
                  if cn0_i < 0 then
                    cn0_i := 0;
                  elsif cn0_i > 99 then
                    cn0_i := 99;
                  end if;
                  -- Keep reporting the estimator output in pull-in as well to aid debug/observability.
                  cn0_dbhz_r <= to_unsigned(cn0_i, cn0_dbhz_r'length);

                  -- Carrier lock detector using cos(2*phase_error) ~= NBD/NBP.
                  nbd_sample_i := (prompt_i_s * prompt_i_s) - (prompt_q_s * prompt_q_s);
                  nbp_sample_i := (prompt_i_s * prompt_i_s) + (prompt_q_s * prompt_q_s);
                  if nbp_sample_i < 1 then
                    nbp_sample_i := 1;
                  end if;

                  nbd_avg_i := nbd_avg_r + (nbd_sample_i - nbd_avg_r) / C_LOCK_SMOOTH_DIV;
                  nbp_avg_i := nbp_avg_r + (nbp_sample_i - nbp_avg_r) / C_LOCK_SMOOTH_DIV;
                  if nbp_avg_i < 1 then
                    nbp_avg_i := 1;
                  end if;
                  nbd_avg_r <= nbd_avg_i;
                  nbp_avg_r <= nbp_avg_i;

                  carrier_metric_i := (nbd_avg_i * 32768) / nbp_avg_i;
                  if carrier_metric_i > 32767 then
                    carrier_metric_i := 32767;
                  elsif carrier_metric_i < -32768 then
                    carrier_metric_i := -32768;
                  end if;
                  carrier_lock_metric_r <= to_signed(carrier_metric_i, carrier_lock_metric_r'length);

                  if state_r = TRACK_LOCKED then
                    -- PLL mode: prompt Q/I phase discriminator.
                    curr_i_s := to_integer(shift_right(next_prompt_i, 12));
                    curr_q_s := to_integer(shift_right(next_prompt_q, 12));
                    den_i := abs_i(curr_i_s) + 1;
                    ratio_q8_i := (curr_q_s * 256) / den_i;
                    carrier_err_pll_q15_i := clamp_i(ratio_q8_i * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
                    carrier_err_fll_q15_i := carrier_err_pll_q15_i;
                    carrier_err_i := carrier_err_pll_q15_i;
                  else
                    -- FLL pull-in mode: prompt phase progression cross/dot discriminator.
                    if prev_prompt_valid_r = '1' then
                      prev_i_s := to_integer(shift_right(prev_prompt_i_r, 12));
                      prev_q_s := to_integer(shift_right(prev_prompt_q_r, 12));
                      curr_i_s := to_integer(shift_right(next_prompt_i, 12));
                      curr_q_s := to_integer(shift_right(next_prompt_q, 12));
                      cross_i := prev_i_s * curr_q_s - prev_q_s * curr_i_s;
                      dot_i   := prev_i_s * curr_i_s + prev_q_s * curr_q_s;
                      den_i := abs_i(dot_i) + 1;
                      ratio_q8_i := (cross_i * 256) / den_i;
                      carrier_err_fll_q15_i := clamp_i(ratio_q8_i * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
                    else
                      curr_i_s := to_integer(shift_right(next_prompt_i, 12));
                      curr_q_s := to_integer(shift_right(next_prompt_q, 12));
                      den_i := abs_i(curr_i_s) + 1;
                      ratio_q8_i := (curr_q_s * 256) / den_i;
                      carrier_err_fll_q15_i := clamp_i(ratio_q8_i * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
                    end if;
                    carrier_err_pll_q15_i := carrier_err_fll_q15_i;
                    carrier_err_i := carrier_err_fll_q15_i;
                  end if;

                  den_i := early_mag_i + late_mag_i + 1;
                  ratio_q8_i := (dll_err_i * 256) / den_i;
                  dll_err_q15_i := clamp_i(ratio_q8_i * 128, -32767, 32767);

                  if state_r = TRACK_LOCKED then
                    pll_bw_sel_i := to_integer(pll_bw_narrow_hz_i);
                    dll_bw_sel_i := to_integer(dll_bw_narrow_hz_i);
                  else
                    pll_bw_sel_i := to_integer(pll_bw_hz_i);
                    dll_bw_sel_i := to_integer(dll_bw_hz_i);
                  end if;
                  if pll_bw_sel_i < C_PLL_GAIN_MIN_Q8_8 then
                    pll_bw_sel_i := C_PLL_GAIN_MIN_Q8_8;
                  end if;
                  if dll_bw_sel_i < C_DLL_GAIN_MIN_Q8_8 then
                    dll_bw_sel_i := C_DLL_GAIN_MIN_Q8_8;
                  end if;

                  -- DLL PI-like fixed-point loop: bandwidth directly scales gains.
                  code_kp_i := (dll_bw_sel_i * C_CODE_LOOP_KP_PER_HZ_Q8 + 128) / 256;
                  if code_kp_i < 1 then
                    code_kp_i := 1;
                  end if;
                  code_ki_i := code_kp_i / C_CODE_LOOP_KI_DIV;
                  if code_ki_i < 1 then
                    code_ki_i := 1;
                  end if;
                  code_delta_i := to_integer(code_loop_i_r) + ((dll_err_q15_i * code_ki_i) / 32768);
                  code_delta_i := clamp_i(code_delta_i, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));
                  code_loop_i_r <= to_signed(code_delta_i, 32);
                  code_prop_i := (dll_err_q15_i * code_kp_i) / 32768;
                  code_delta_i := clamp_i(code_delta_i + code_prop_i, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));
                  if code_delta_i >= 0 then
                    code_fcw_r <= C_CODE_NCO_FCW + to_unsigned(code_delta_i, 32);
                  else
                    code_fcw_r <= C_CODE_NCO_FCW - to_unsigned(-code_delta_i, 32);
                  end if;

                  if state_r = TRACK_LOCKED then
                    -- PLL loop filter (wide/narrow selection controlled by state).
                    carr_kp_i := (pll_bw_sel_i * C_CARR_FCW_PER_HZ + 128) / 256;
                    if carr_kp_i < 1 then
                      carr_kp_i := 1;
                    end if;
                    carr_ki_i := carr_kp_i / C_PLL_LOOP_KI_DIV;
                    if carr_ki_i < 1 then
                      carr_ki_i := 1;
                    end if;
                    carr_fcw_i := to_integer(carr_loop_i_r) + ((carrier_err_pll_q15_i * carr_ki_i) / 32768);
                    carr_fcw_i := clamp_i(carr_fcw_i, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
                    carr_loop_i_r <= to_signed(carr_fcw_i, 32);

                    carr_prop_i := (carrier_err_pll_q15_i * carr_kp_i) / 32768;
                    carr_fcw_max_step_i := carr_fcw_from_hz_i(to_integer(dopp_step_lock_i));
                    if carr_fcw_max_step_i < 1 then
                      carr_fcw_max_step_i := 1;
                    end if;
                    carr_prop_i := clamp_i(carr_prop_i, -carr_fcw_max_step_i, carr_fcw_max_step_i);
                    carr_fcw_i := clamp_i(carr_fcw_i + carr_prop_i, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
                    carr_fcw_cmd_r <= to_signed(carr_fcw_i, 32);
                  else
                    -- FLL pull-in filter with configurable bandwidth and rate-limited step.
                    fll_gain_i := to_integer(fll_bw_hz_i);
                    if fll_gain_i < C_FLL_GAIN_MIN_Q8_8 then
                      fll_gain_i := C_FLL_GAIN_MIN_Q8_8;
                    end if;
                    carr_kp_i := ((fll_gain_i * C_CARR_FCW_PER_HZ * C_FLL_LOOP_STEP_SCALE) + 128) / 256;
                    if carr_kp_i < 1 then
                      carr_kp_i := 1;
                    end if;
                    fll_step_i := (carrier_err_fll_q15_i * carr_kp_i) / 32768;
                    carr_fcw_max_step_i := carr_fcw_from_hz_i(to_integer(dopp_step_pullin_i));
                    if carr_fcw_max_step_i < 1 then
                      carr_fcw_max_step_i := 1;
                    end if;
                    fll_step_i := clamp_i(fll_step_i, -carr_fcw_max_step_i, carr_fcw_max_step_i);
                    -- Add a limited PLL assist term so pull-in can also unwind static
                    -- phase offset from acquisition, not just frequency error.
                    pll_assist_step_i := (carrier_err_pll_q15_i * carr_kp_i) / 32768;
                    pll_assist_step_i := pll_assist_step_i / C_PULLIN_PLL_ASSIST_DIV;
                    pll_assist_step_i := clamp_i(pll_assist_step_i, -carr_fcw_max_step_i, carr_fcw_max_step_i);

                    carr_fcw_i := to_integer(carr_loop_i_r) + fll_step_i + pll_assist_step_i;
                    carr_fcw_i := clamp_i(carr_fcw_i, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
                    carr_loop_i_r <= to_signed(carr_fcw_i, 32);
                    carr_fcw_cmd_r <= to_signed(carr_fcw_i, 32);
                  end if;

                  dopp_i := carr_fcw_i / C_CARR_FCW_PER_HZ;
                  dopp_r <= clamp_s16(dopp_i);

                  code_enter_v := (prompt_mag_i > C_PROMPT_MAG_MIN) and
                                  (cn0_i >= to_integer(min_cn0_dbhz_i)) and
                                  (abs_i(dll_err_q15_i) < C_DLL_ERR_LOCK_MAX);
                  code_track_v := (prompt_mag_i > C_PROMPT_MAG_MIN) and
                                  (cn0_i >= to_integer(min_cn0_dbhz_i)) and
                                  (abs_i(dll_err_q15_i) < C_DLL_ERR_TRACK_MAX);
                  carrier_enter_th_i := to_integer(carrier_lock_th_i);
                  carrier_track_th_i := carrier_enter_th_i - C_CARR_LOCK_HYST_Q15;
                  if carrier_track_th_i < -32768 then
                    carrier_track_th_i := -32768;
                  end if;

                  -- In pull-in, FLL corrects frequency but not absolute phase, so
                  -- accept either Costas lobe and let PLL settle the phase sign after
                  -- we enter TRACK_LOCKED.
                  if state_r = TRACK_PULLIN then
                    carrier_metric_eval_i := abs_i(carrier_metric_i);
                  else
                    carrier_metric_eval_i := carrier_metric_i;
                  end if;

                  carrier_enter_v := (carrier_metric_eval_i >= carrier_enter_th_i) and
                                     (abs_i(carrier_err_i) < C_CARR_ERR_LOCK_MAX);
                  carrier_track_v := (carrier_metric_eval_i >= carrier_track_th_i) and
                                     (abs_i(carrier_err_i) < C_CARR_ERR_TRACK_MAX);
                  max_lock_fail_v := to_integer(max_lock_fail_i);
                  if max_lock_fail_v < 4 then
                    max_lock_fail_v := 4;
                  elsif max_lock_fail_v > (C_LOCK_SCORE_MAX - 8) then
                    max_lock_fail_v := C_LOCK_SCORE_MAX - 8;
                  end if;
                  lock_enter_th_i := max_lock_fail_v;
                  lock_exit_th_i := lock_enter_th_i / 2;
                  if lock_exit_th_i < 2 then
                    lock_exit_th_i := 2;
                  end if;

                  lock_score_i := lock_score_r;
                  if code_track_v and carrier_track_v then
                    lock_score_i := lock_score_i + C_LOCK_SCORE_INC_BOTH;
                  elsif code_track_v then
                    lock_score_i := lock_score_i + C_LOCK_SCORE_INC_CODE;
                  else
                    lock_score_i := lock_score_i - C_LOCK_SCORE_DEC_CODE;
                  end if;
                  if not carrier_track_v then
                    lock_score_i := lock_score_i - C_LOCK_SCORE_DEC_CARR;
                  end if;
                  if lock_score_i < 0 then
                    lock_score_i := 0;
                  elsif lock_score_i > C_LOCK_SCORE_MAX then
                    lock_score_i := C_LOCK_SCORE_MAX;
                  end if;
                  lock_score_r <= lock_score_i;

                  if state_r = TRACK_PULLIN then
                    if code_enter_v and carrier_enter_v and lock_score_i >= lock_enter_th_i then
                      state_r        <= TRACK_LOCKED;
                      code_lock_r    <= '1';
                      carrier_lock_r <= '1';
                    else
                      if code_enter_v then
                        code_lock_r <= '1';
                      else
                        code_lock_r <= '0';
                      end if;
                      if carrier_enter_v then
                        carrier_lock_r <= '1';
                      else
                        carrier_lock_r <= '0';
                      end if;
                    end if;
                  else
                    if lock_score_i <= lock_exit_th_i then
                      state_r        <= TRACK_PULLIN;
                      code_lock_r    <= '0';
                      carrier_lock_r <= '0';
                    else
                      code_lock_r    <= '1';
                      if carrier_track_v then
                        carrier_lock_r <= '1';
                      else
                        carrier_lock_r <= '0';
                      end if;
                    end if;
                  end if;

                  report_valid_r      <= '1';
                  prompt_i_rep_r      <= sat_s32_to_s24(next_prompt_i);
                  prompt_q_rep_r      <= sat_s32_to_s24(next_prompt_q);
                  prev_prompt_i_r     <= next_prompt_i;
                  prev_prompt_q_r     <= next_prompt_q;
                  prev_prompt_valid_r <= '1';

                  ms_sample_cnt_r <= 0;
                  prompt_i_acc_r  <= (others => '0');
                  prompt_q_acc_r  <= (others => '0');
                  early_i_acc_r   <= (others => '0');
                  early_q_acc_r   <= (others => '0');
                  late_i_acc_r    <= (others => '0');
                  late_q_acc_r    <= (others => '0');
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
