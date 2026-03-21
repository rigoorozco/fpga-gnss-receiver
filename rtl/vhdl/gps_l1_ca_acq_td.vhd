library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;
use work.gps_l1_ca_nco_pkg.all;

entity gps_l1_ca_acq_td is
  generic (
    G_DWELL_MS : integer := 2
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
    code_bin_count_i     : in  unsigned(10 downto 0);
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

architecture rtl of gps_l1_ca_acq_td is
  type state_t is (
    IDLE,
    PRN_INIT,
    CAPTURE,
    SEARCH_BIN_INIT,
    SEARCH_SAMPLES,
    SEARCH_BIN_STORE,
    COH_FINALIZE,
    PRN_EVAL,
    FINALIZE
  );

  constant C_CARR_FCW_PER_HZ : integer := 2147;
  constant C_DEF_COH_MS       : integer := 1;
  constant C_DEF_CODE_BINS    : integer := 16;
  constant C_DEF_CODE_STEP    : integer := 64;
  constant C_DEF_DOPP_BINS    : integer := 9;
  constant C_MAX_CODE_BINS    : integer := 1023;
  constant C_MAX_DOPP_BINS    : integer := 81;
  constant C_MAX_BINS         : integer := C_MAX_CODE_BINS * C_MAX_DOPP_BINS;

  type sample_arr_t is array (0 to C_SAMPLES_PER_MS - 1) of signed(15 downto 0);
  type metric_arr_t is array (0 to C_MAX_BINS - 1) of unsigned(31 downto 0);
  type corr_arr_t is array (0 to C_MAX_BINS - 1) of signed(47 downto 0);
  type code_arr_t is array (0 to C_MAX_BINS - 1) of unsigned(10 downto 0);
  type dopp_arr_t is array (0 to C_MAX_BINS - 1) of signed(15 downto 0);
  type prn_seq_t is array (0 to 1022) of std_logic;

  signal state_r              : state_t := IDLE;
  signal prn_cur_r            : unsigned(5 downto 0) := to_unsigned(1, 6);

  signal cap_i_r              : sample_arr_t;
  signal cap_q_r              : sample_arr_t;
  signal cap_sample_idx_r     : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;

  signal coh_i_acc_r          : corr_arr_t;
  signal coh_q_acc_r          : corr_arr_t;
  signal noncoh_metric_r      : metric_arr_t;
  signal bin_code_r           : code_arr_t;
  signal bin_dopp_r           : dopp_arr_t;

  signal prn_seq_r            : prn_seq_t;

  signal active_code_bins_r   : integer range 1 to C_MAX_CODE_BINS := C_DEF_CODE_BINS;
  signal active_dopp_bins_r   : integer range 1 to C_MAX_DOPP_BINS := C_DEF_DOPP_BINS;
  signal active_total_bins_r  : integer range 1 to C_MAX_BINS := C_DEF_CODE_BINS * C_DEF_DOPP_BINS;
  signal active_coh_ms_r      : integer range 1 to 255 := C_DEF_COH_MS;
  signal active_noncoh_r      : integer range 1 to 255 := G_DWELL_MS;

  signal coh_ms_idx_r         : integer range 0 to 254 := 0;
  signal noncoh_idx_r         : integer range 0 to 254 := 0;

  signal search_bin_idx_r     : integer range 0 to C_MAX_BINS - 1 := 0;
  signal search_sample_idx_r  : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;
  signal search_chip_idx_r    : integer range 0 to 1022 := 0;
  signal search_code_nco_r    : unsigned(31 downto 0) := (others => '0');
  signal search_carr_phase_r  : signed(31 downto 0) := (others => '0');
  signal search_carr_fcw_r    : signed(31 downto 0) := (others => '0');
  signal search_corr_i_r      : signed(47 downto 0) := (others => '0');
  signal search_corr_q_r      : signed(47 downto 0) := (others => '0');

  signal finalize_bin_idx_r   : integer range 0 to C_MAX_BINS - 1 := 0;
  signal prn_eval_idx_r       : integer range 0 to C_MAX_BINS - 1 := 0;
  signal prn_best_metric_r    : unsigned(31 downto 0) := (others => '0');
  signal prn_best_code_r      : unsigned(10 downto 0) := (others => '0');
  signal prn_best_dopp_r      : signed(15 downto 0) := (others => '0');

  signal best_metric_r        : unsigned(31 downto 0) := (others => '0');
  signal best_prn_r           : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal best_code_r          : unsigned(10 downto 0) := (others => '0');
  signal best_dopp_r          : signed(15 downto 0) := (others => '0');

  signal acq_done_r           : std_logic := '0';
  signal acq_success_r        : std_logic := '0';
  signal result_valid_r       : std_logic := '0';

  function abs_i(x : integer) return integer is
  begin
    if x < 0 then
      return -x;
    end if;
    return x;
  end function;

  function abs_s48(x : signed(47 downto 0)) return unsigned is
    variable v : signed(47 downto 0);
  begin
    if x < 0 then
      v := -x;
    else
      v := x;
    end if;
    return unsigned(v);
  end function;

  function abs_s48_sat_u32(x : signed(47 downto 0)) return u32_t is
    variable ax : unsigned(47 downto 0);
  begin
    ax := abs_s48(x);
    if ax(47 downto 32) /= "0000000000000000" then
      return (others => '1');
    end if;
    return ax(31 downto 0);
  end function;

  function sat_add_u32(
    a : unsigned(31 downto 0);
    b : unsigned(31 downto 0)
  ) return u32_t is
    variable sum_v : unsigned(32 downto 0);
  begin
    sum_v := ('0' & a) + ('0' & b);
    if sum_v(32) = '1' then
      return (others => '1');
    end if;
    return sum_v(31 downto 0);
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

  function u32_img(x : unsigned(31 downto 0)) return string is
  begin
    if x(31) = '1' then
      return "0x" & to_hstring(std_logic_vector(x));
    end if;
    return integer'image(to_integer(x));
  end function;

  function carr_fcw_from_hz(dopp_hz : signed(15 downto 0)) return signed is
  begin
    return to_signed(to_integer(dopp_hz) * C_CARR_FCW_PER_HZ, 32);
  end function;

  function g2_tap_a(prn_i : integer) return integer is
  begin
    case prn_i is
      when 1  => return 2;
      when 2  => return 3;
      when 3  => return 4;
      when 4  => return 5;
      when 5  => return 1;
      when 6  => return 2;
      when 7  => return 1;
      when 8  => return 2;
      when 9  => return 3;
      when 10 => return 2;
      when 11 => return 3;
      when 12 => return 5;
      when 13 => return 6;
      when 14 => return 7;
      when 15 => return 8;
      when 16 => return 9;
      when 17 => return 1;
      when 18 => return 2;
      when 19 => return 3;
      when 20 => return 4;
      when 21 => return 5;
      when 22 => return 6;
      when 23 => return 1;
      when 24 => return 4;
      when 25 => return 5;
      when 26 => return 6;
      when 27 => return 7;
      when 28 => return 8;
      when 29 => return 1;
      when 30 => return 2;
      when 31 => return 3;
      when others => return 4;
    end case;
  end function;

  function g2_tap_b(prn_i : integer) return integer is
  begin
    case prn_i is
      when 1  => return 6;
      when 2  => return 7;
      when 3  => return 8;
      when 4  => return 9;
      when 5  => return 9;
      when 6  => return 10;
      when 7  => return 8;
      when 8  => return 9;
      when 9  => return 10;
      when 10 => return 3;
      when 11 => return 4;
      when 12 => return 6;
      when 13 => return 7;
      when 14 => return 8;
      when 15 => return 9;
      when 16 => return 10;
      when 17 => return 4;
      when 18 => return 5;
      when 19 => return 6;
      when 20 => return 7;
      when 21 => return 8;
      when 22 => return 9;
      when 23 => return 3;
      when 24 => return 6;
      when 25 => return 7;
      when 26 => return 8;
      when 27 => return 9;
      when 28 => return 10;
      when 29 => return 6;
      when 30 => return 7;
      when 31 => return 8;
      when others => return 9;
    end case;
  end function;

  function build_prn_sequence(prn_i : integer) return prn_seq_t is
    variable g1 : std_logic_vector(9 downto 0);
    variable g2 : std_logic_vector(9 downto 0);
    variable g1_out : std_logic;
    variable g2_out : std_logic;
    variable fb1 : std_logic;
    variable fb2 : std_logic;
    variable ta  : integer;
    variable tb  : integer;
    variable seq_v : prn_seq_t;
  begin
    g1 := (others => '1');
    g2 := (others => '1');
    ta := g2_tap_a(prn_i);
    tb := g2_tap_b(prn_i);

    for chip in 0 to 1022 loop
      g1_out := g1(9);
      g2_out := g2(10 - ta) xor g2(10 - tb);
      seq_v(chip) := g1_out xor g2_out;

      fb1 := g1(2) xor g1(9);
      fb2 := g2(1) xor g2(2) xor g2(5) xor g2(7) xor g2(8) xor g2(9);

      g1 := g1(8 downto 0) & fb1;
      g2 := g2(8 downto 0) & fb2;
    end loop;
    return seq_v;
  end function;

begin
  acq_done      <= acq_done_r;
  acq_success   <= acq_success_r;
  result_valid  <= result_valid_r;
  result_prn    <= best_prn_r;
  result_dopp   <= best_dopp_r;
  result_code   <= best_code_r;
  result_metric <= best_metric_r;

  process (clk)
    variable coh_cfg_i        : integer;
    variable noncoh_cfg_i     : integer;
    variable code_bins_cfg_i  : integer;
    variable code_step_cfg_i  : integer;
    variable dopp_bins_cfg_i  : integer;

    variable step_i           : integer;
    variable dopp_min_i       : integer;
    variable dopp_max_i       : integer;
    variable dopp_span_i      : integer;
    variable full_dopp_bins_i : integer;
    variable active_dopp_i    : integer;
    variable start_dopp_idx_i : integer;
    variable active_code_i    : integer;
    variable total_bins_i     : integer;
    variable bin_i            : integer;
    variable d_i              : integer;
    variable c_i              : integer;
    variable code_i           : integer;
    variable dopp_hz_i        : integer;

    variable lo_i_i           : integer;
    variable lo_q_i           : integer;
    variable si_i             : integer;
    variable sq_i             : integer;
    variable mix_i_i          : integer;
    variable mix_q_i          : integer;
    variable next_code_nco_v  : unsigned(31 downto 0);
    variable carr_phase_v     : signed(31 downto 0);

    variable coh_metric_v     : unsigned(31 downto 0);
    variable noncoh_sum_v     : unsigned(31 downto 0);
    variable prn_metric_next_v : unsigned(31 downto 0);
    variable prn_code_next_v   : unsigned(10 downto 0);
    variable prn_dopp_next_v   : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        state_r             <= IDLE;
        prn_cur_r           <= to_unsigned(1, 6);
        cap_sample_idx_r    <= 0;
        active_code_bins_r  <= C_DEF_CODE_BINS;
        active_dopp_bins_r  <= C_DEF_DOPP_BINS;
        active_total_bins_r <= C_DEF_CODE_BINS * C_DEF_DOPP_BINS;
        active_coh_ms_r     <= C_DEF_COH_MS;
        active_noncoh_r     <= G_DWELL_MS;
        coh_ms_idx_r        <= 0;
        noncoh_idx_r        <= 0;
        search_bin_idx_r    <= 0;
        search_sample_idx_r <= 0;
        search_chip_idx_r   <= 0;
        search_code_nco_r   <= (others => '0');
        search_carr_phase_r <= (others => '0');
        search_carr_fcw_r   <= (others => '0');
        search_corr_i_r     <= (others => '0');
        search_corr_q_r     <= (others => '0');
        finalize_bin_idx_r  <= 0;
        prn_eval_idx_r      <= 0;
        prn_best_metric_r   <= (others => '0');
        prn_best_code_r     <= (others => '0');
        prn_best_dopp_r     <= (others => '0');
        best_metric_r       <= (others => '0');
        best_prn_r          <= to_unsigned(1, 6);
        best_code_r         <= (others => '0');
        best_dopp_r         <= (others => '0');
        acq_done_r          <= '0';
        acq_success_r       <= '0';
        result_valid_r      <= '0';

        for i in 0 to C_MAX_BINS - 1 loop
          noncoh_metric_r(i) <= (others => '0');
          coh_i_acc_r(i) <= (others => '0');
          coh_q_acc_r(i) <= (others => '0');
          bin_code_r(i) <= (others => '0');
          bin_dopp_r(i) <= (others => '0');
        end loop;
        for i in 0 to 1022 loop
          prn_seq_r(i) <= '0';
        end loop;
      else
        acq_done_r     <= '0';
        result_valid_r <= '0';

        case state_r is
          when IDLE =>
            acq_success_r <= '0';
            if core_en = '1' and start_pulse = '1' then
              coh_cfg_i := to_integer(coh_ms_i);
              if coh_cfg_i <= 0 then
                coh_cfg_i := C_DEF_COH_MS;
              elsif coh_cfg_i > 255 then
                coh_cfg_i := 255;
              end if;

              noncoh_cfg_i := to_integer(noncoh_dwells_i);
              if noncoh_cfg_i <= 0 then
                noncoh_cfg_i := G_DWELL_MS;
              elsif noncoh_cfg_i > 255 then
                noncoh_cfg_i := 255;
              end if;

              code_bins_cfg_i := to_integer(code_bin_count_i);
              if code_bins_cfg_i <= 0 then
                code_bins_cfg_i := C_DEF_CODE_BINS;
              elsif code_bins_cfg_i > C_MAX_CODE_BINS then
                code_bins_cfg_i := C_MAX_CODE_BINS;
              end if;

              code_step_cfg_i := to_integer(code_bin_step_i);
              if code_step_cfg_i <= 0 then
                code_step_cfg_i := C_DEF_CODE_STEP;
              elsif code_step_cfg_i > 1022 then
                code_step_cfg_i := 1022;
              end if;

              step_i := abs_i(to_integer(doppler_step));
              if step_i < 1 then
                step_i := 1;
              end if;

              dopp_min_i := to_integer(doppler_min);
              dopp_max_i := to_integer(doppler_max);
              if dopp_max_i < dopp_min_i then
                dopp_span_i := dopp_min_i;
                dopp_min_i := dopp_max_i;
                dopp_max_i := dopp_span_i;
              end if;

              dopp_span_i := dopp_max_i - dopp_min_i;
              full_dopp_bins_i := (dopp_span_i / step_i) + 1;
              if full_dopp_bins_i < 1 then
                full_dopp_bins_i := 1;
              end if;

              dopp_bins_cfg_i := to_integer(doppler_bin_count_i);
              if dopp_bins_cfg_i <= 0 then
                active_dopp_i := C_DEF_DOPP_BINS;
              else
                active_dopp_i := dopp_bins_cfg_i;
              end if;
              if active_dopp_i > C_MAX_DOPP_BINS then
                active_dopp_i := C_MAX_DOPP_BINS;
              end if;
              if active_dopp_i > full_dopp_bins_i then
                active_dopp_i := full_dopp_bins_i;
              end if;
              if active_dopp_i < 1 then
                active_dopp_i := 1;
              end if;

              start_dopp_idx_i := (full_dopp_bins_i - active_dopp_i) / 2;

              active_code_i := code_bins_cfg_i;
              total_bins_i := active_code_i * active_dopp_i;
              if total_bins_i < 1 then
                total_bins_i := 1;
              elsif total_bins_i > C_MAX_BINS then
                total_bins_i := C_MAX_BINS;
              end if;

              for i in 0 to C_MAX_BINS - 1 loop
                noncoh_metric_r(i) <= (others => '0');
                coh_i_acc_r(i) <= (others => '0');
                coh_q_acc_r(i) <= (others => '0');
                bin_code_r(i) <= (others => '0');
                bin_dopp_r(i) <= (others => '0');
              end loop;

              bin_i := 0;
              for d in 0 to C_MAX_DOPP_BINS - 1 loop
                exit when d >= active_dopp_i;
                d_i := start_dopp_idx_i + d;
                dopp_hz_i := dopp_min_i + d_i * step_i;
                for c in 0 to C_MAX_CODE_BINS - 1 loop
                  exit when c >= active_code_i;
                  if bin_i < total_bins_i then
                    c_i := c * code_step_cfg_i;
                    code_i := c_i mod 1023;
                    if code_i < 0 then
                      code_i := code_i + 1023;
                    end if;
                    bin_code_r(bin_i) <= to_unsigned(code_i, 11);
                    bin_dopp_r(bin_i) <= clamp_s16(dopp_hz_i);
                    bin_i := bin_i + 1;
                  end if;
                end loop;
              end loop;

              prn_seq_r <= build_prn_sequence(to_integer(prn_start));

              active_code_bins_r  <= active_code_i;
              active_dopp_bins_r  <= active_dopp_i;
              active_total_bins_r <= total_bins_i;
              active_coh_ms_r     <= coh_cfg_i;
              active_noncoh_r     <= noncoh_cfg_i;

              prn_cur_r           <= prn_start;
              cap_sample_idx_r    <= 0;
              coh_ms_idx_r        <= 0;
              noncoh_idx_r        <= 0;
              search_bin_idx_r    <= 0;
              search_sample_idx_r <= 0;
              finalize_bin_idx_r  <= 0;
              prn_eval_idx_r      <= 0;
              prn_best_metric_r   <= (others => '0');
              prn_best_code_r     <= (others => '0');
              prn_best_dopp_r     <= (others => '0');
              best_metric_r       <= (others => '0');
              best_prn_r          <= prn_start;
              best_code_r         <= (others => '0');
              best_dopp_r         <= (others => '0');

              state_r <= CAPTURE;
            end if;

          when PRN_INIT =>
            prn_seq_r <= build_prn_sequence(to_integer(prn_cur_r));
            for i in 0 to C_MAX_BINS - 1 loop
              noncoh_metric_r(i) <= (others => '0');
              coh_i_acc_r(i) <= (others => '0');
              coh_q_acc_r(i) <= (others => '0');
            end loop;
            cap_sample_idx_r <= 0;
            coh_ms_idx_r <= 0;
            noncoh_idx_r <= 0;
            state_r <= CAPTURE;

          when CAPTURE =>
            if s_valid = '1' then
              cap_i_r(cap_sample_idx_r) <= s_i;
              cap_q_r(cap_sample_idx_r) <= s_q;
              if cap_sample_idx_r = C_SAMPLES_PER_MS - 1 then
                search_bin_idx_r    <= 0;
                search_sample_idx_r <= 0;
                search_corr_i_r     <= (others => '0');
                search_corr_q_r     <= (others => '0');
                state_r             <= SEARCH_BIN_INIT;
              else
                cap_sample_idx_r <= cap_sample_idx_r + 1;
              end if;
            end if;

          when SEARCH_BIN_INIT =>
            search_sample_idx_r <= 0;
            search_corr_i_r <= (others => '0');
            search_corr_q_r <= (others => '0');
            search_chip_idx_r <= to_integer(bin_code_r(search_bin_idx_r));
            search_code_nco_r <= shift_left(resize(bin_code_r(search_bin_idx_r), 32), 21);
            search_carr_phase_r <= (others => '0');
            search_carr_fcw_r <= carr_fcw_from_hz(bin_dopp_r(search_bin_idx_r));
            state_r <= SEARCH_SAMPLES;

          when SEARCH_SAMPLES =>
            lo_i_i := to_integer(lo_cos_q15(unsigned(search_carr_phase_r(31 downto 22))));
            lo_q_i := -to_integer(lo_sin_q15(unsigned(search_carr_phase_r(31 downto 22))));

            si_i := to_integer(cap_i_r(search_sample_idx_r));
            sq_i := to_integer(cap_q_r(search_sample_idx_r));
            mix_i_i := (si_i * lo_i_i - sq_i * lo_q_i) / 32768;
            mix_q_i := (si_i * lo_q_i + sq_i * lo_i_i) / 32768;

            if prn_seq_r(search_chip_idx_r) = '1' then
              search_corr_i_r <= search_corr_i_r - to_signed(mix_i_i, search_corr_i_r'length);
              search_corr_q_r <= search_corr_q_r - to_signed(mix_q_i, search_corr_q_r'length);
            else
              search_corr_i_r <= search_corr_i_r + to_signed(mix_i_i, search_corr_i_r'length);
              search_corr_q_r <= search_corr_q_r + to_signed(mix_q_i, search_corr_q_r'length);
            end if;

            carr_phase_v := search_carr_phase_r + search_carr_fcw_r;
            search_carr_phase_r <= carr_phase_v;

            next_code_nco_v := search_code_nco_r + C_CODE_NCO_FCW;
            if next_code_nco_v < search_code_nco_r then
              if search_chip_idx_r = 1022 then
                search_chip_idx_r <= 0;
              else
                search_chip_idx_r <= search_chip_idx_r + 1;
              end if;
            end if;
            search_code_nco_r <= next_code_nco_v;

            if search_sample_idx_r = C_SAMPLES_PER_MS - 1 then
              state_r <= SEARCH_BIN_STORE;
            else
              search_sample_idx_r <= search_sample_idx_r + 1;
            end if;

          when SEARCH_BIN_STORE =>
            coh_i_acc_r(search_bin_idx_r) <= coh_i_acc_r(search_bin_idx_r) + search_corr_i_r;
            coh_q_acc_r(search_bin_idx_r) <= coh_q_acc_r(search_bin_idx_r) + search_corr_q_r;

            if search_bin_idx_r + 1 >= active_total_bins_r then
              if coh_ms_idx_r + 1 >= active_coh_ms_r then
                finalize_bin_idx_r <= 0;
                state_r <= COH_FINALIZE;
              else
                coh_ms_idx_r <= coh_ms_idx_r + 1;
                cap_sample_idx_r <= 0;
                state_r <= CAPTURE;
              end if;
            else
              search_bin_idx_r <= search_bin_idx_r + 1;
              state_r <= SEARCH_BIN_INIT;
            end if;

          when COH_FINALIZE =>
            coh_metric_v := sat_add_u32(
              abs_s48_sat_u32(coh_i_acc_r(finalize_bin_idx_r)),
              abs_s48_sat_u32(coh_q_acc_r(finalize_bin_idx_r))
            );
            noncoh_sum_v := sat_add_u32(noncoh_metric_r(finalize_bin_idx_r), coh_metric_v);
            noncoh_metric_r(finalize_bin_idx_r) <= noncoh_sum_v;
            coh_i_acc_r(finalize_bin_idx_r) <= (others => '0');
            coh_q_acc_r(finalize_bin_idx_r) <= (others => '0');

            if finalize_bin_idx_r + 1 >= active_total_bins_r then
              coh_ms_idx_r <= 0;
              if noncoh_idx_r + 1 >= active_noncoh_r then
                prn_eval_idx_r <= 0;
                prn_best_metric_r <= (others => '0');
                prn_best_code_r <= bin_code_r(0);
                prn_best_dopp_r <= bin_dopp_r(0);
                state_r <= PRN_EVAL;
              else
                noncoh_idx_r <= noncoh_idx_r + 1;
                cap_sample_idx_r <= 0;
                state_r <= CAPTURE;
              end if;
            else
              finalize_bin_idx_r <= finalize_bin_idx_r + 1;
            end if;

          when PRN_EVAL =>
            prn_metric_next_v := prn_best_metric_r;
            prn_code_next_v   := prn_best_code_r;
            prn_dopp_next_v   := prn_best_dopp_r;
            if noncoh_metric_r(prn_eval_idx_r) >= prn_metric_next_v then
              prn_metric_next_v := noncoh_metric_r(prn_eval_idx_r);
              prn_code_next_v   := bin_code_r(prn_eval_idx_r);
              prn_dopp_next_v   := bin_dopp_r(prn_eval_idx_r);
            end if;

            if prn_eval_idx_r + 1 >= active_total_bins_r then
              if prn_metric_next_v >= best_metric_r then
                best_metric_r <= prn_metric_next_v;
                best_prn_r    <= prn_cur_r;
                best_code_r   <= prn_code_next_v;
                best_dopp_r   <= prn_dopp_next_v;
              end if;

              if prn_cur_r >= prn_stop then
                state_r <= FINALIZE;
              else
                prn_cur_r <= prn_cur_r + 1;
                state_r <= PRN_INIT;
              end if;
            else
              prn_best_metric_r <= prn_metric_next_v;
              prn_best_code_r   <= prn_code_next_v;
              prn_best_dopp_r   <= prn_dopp_next_v;
              prn_eval_idx_r <= prn_eval_idx_r + 1;
            end if;

          when FINALIZE =>
            acq_done_r <= '1';
            if best_metric_r >= detect_thresh then
              acq_success_r  <= '1';
              result_valid_r <= '1';
            else
              acq_success_r  <= '0';
            end if;
            state_r <= IDLE;
        end case;
      end if;
    end if;
  end process;
end architecture;
