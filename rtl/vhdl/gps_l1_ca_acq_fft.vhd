library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;
use work.gps_l1_ca_nco_pkg.all;

entity gps_l1_ca_acq_fft is
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

architecture rtl of gps_l1_ca_acq_fft is
  type state_t is (
    IDLE,
    PRN_PREP,
    CAPTURE_MS,
    DOPP_PREP,
    DOPP_PROCESS,
    COH_COMMIT,
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
  constant C_NFFT             : integer := 2048;
  constant C_FFT_BITS         : integer := 11;

  type sample_arr_t is array (0 to C_SAMPLES_PER_MS - 1) of signed(15 downto 0);
  type metric_arr_t is array (0 to C_MAX_BINS - 1) of unsigned(31 downto 0);
  type coh_arr_t is array (0 to C_MAX_BINS - 1) of signed(55 downto 0);
  type code_arr_t is array (0 to C_MAX_BINS - 1) of unsigned(10 downto 0);
  type dopp_arr_t is array (0 to C_MAX_BINS - 1) of signed(15 downto 0);
  type prn_seq_t is array (0 to 1022) of std_logic;

  type cpx32_t is record
    re : signed(31 downto 0);
    im : signed(31 downto 0);
  end record;

  type cpx32_vec_t is array (0 to C_NFFT - 1) of cpx32_t;
  type cpx32_bank_t is array (0 to C_MAX_CODE_BINS - 1) of cpx32_vec_t;

  constant C_CPX_ZERO : cpx32_t := (
    re => (others => '0'),
    im => (others => '0')
  );
  constant C_S32_MAX : signed(31 downto 0) := signed'(x"7FFFFFFF");
  constant C_S32_MIN : signed(31 downto 0) := signed'(x"80000000");

  signal state_r              : state_t := IDLE;
  signal prn_cur_r            : unsigned(5 downto 0) := to_unsigned(1, 6);

  signal cap_i_r              : sample_arr_t;
  signal cap_q_r              : sample_arr_t;
  signal cap_sample_idx_r     : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;

  signal coh_i_acc_r          : coh_arr_t;
  signal coh_q_acc_r          : coh_arr_t;
  signal noncoh_metric_r      : metric_arr_t;
  signal bin_code_r           : code_arr_t;
  signal bin_dopp_r           : dopp_arr_t;

  signal prn_seq_r            : prn_seq_t;
  signal code_fft_r           : cpx32_bank_t;
  signal signal_fft_r         : cpx32_vec_t;

  signal active_code_bins_r   : integer range 1 to C_MAX_CODE_BINS := C_DEF_CODE_BINS;
  signal active_dopp_bins_r   : integer range 1 to C_MAX_DOPP_BINS := C_DEF_DOPP_BINS;
  signal active_total_bins_r  : integer range 1 to C_MAX_BINS := C_DEF_CODE_BINS * C_DEF_DOPP_BINS;
  signal active_coh_ms_r      : integer range 1 to 255 := C_DEF_COH_MS;
  signal active_noncoh_r      : integer range 1 to 255 := G_DWELL_MS;

  signal coh_ms_idx_r         : integer range 0 to 254 := 0;
  signal noncoh_idx_r         : integer range 0 to 254 := 0;

  signal prep_code_idx_r      : integer range 0 to C_MAX_CODE_BINS - 1 := 0;
  signal dopp_idx_r           : integer range 0 to C_MAX_DOPP_BINS - 1 := 0;
  signal code_eval_idx_r      : integer range 0 to C_MAX_CODE_BINS - 1 := 0;

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

  function abs_s56_sat_u32(x : signed(55 downto 0)) return u32_t is
    variable ax : unsigned(55 downto 0);
  begin
    if x < 0 then
      ax := unsigned(-x);
    else
      ax := unsigned(x);
    end if;

    if ax(55 downto 32) /= (ax(55 downto 32)'range => '0') then
      return (others => '1');
    end if;
    return ax(31 downto 0);
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

  function sat_resize_s32(x : signed) return signed is
    variable lo : signed(31 downto 0);
  begin
    if x'length <= 32 then
      return resize(x, 32);
    end if;

    lo := x(31 downto 0);
    if resize(lo, x'length) /= x then
      if x(x'high) = '1' then
        return C_S32_MIN;
      else
        return C_S32_MAX;
      end if;
    end if;

    return lo;
  end function;

  function div_pow2_tz(x : signed; sh : natural) return signed is
  begin
    if sh = 0 then
      return x;
    end if;

    if x < 0 then
      return -shift_right(-x, sh);
    end if;
    return shift_right(x, sh);
  end function;

  function cpx_add_sat(a : cpx32_t; b : cpx32_t) return cpx32_t is
    variable out_v  : cpx32_t;
    variable sum_re : signed(32 downto 0);
    variable sum_im : signed(32 downto 0);
  begin
    sum_re := resize(a.re, 33) + resize(b.re, 33);
    sum_im := resize(a.im, 33) + resize(b.im, 33);
    out_v.re := sat_resize_s32(sum_re);
    out_v.im := sat_resize_s32(sum_im);
    return out_v;
  end function;

  function cpx_sub_sat(a : cpx32_t; b : cpx32_t) return cpx32_t is
    variable out_v  : cpx32_t;
    variable sum_re : signed(32 downto 0);
    variable sum_im : signed(32 downto 0);
  begin
    sum_re := resize(a.re, 33) - resize(b.re, 33);
    sum_im := resize(a.im, 33) - resize(b.im, 33);
    out_v.re := sat_resize_s32(sum_re);
    out_v.im := sat_resize_s32(sum_im);
    return out_v;
  end function;

  function cpx_mul_q15(
    a  : cpx32_t;
    wr : signed(15 downto 0);
    wi : signed(15 downto 0)
  ) return cpx32_t is
    variable out_v     : cpx32_t;
    variable rr        : signed(47 downto 0);
    variable ii        : signed(47 downto 0);
    variable ri        : signed(47 downto 0);
    variable ir        : signed(47 downto 0);
    variable sum_re    : signed(48 downto 0);
    variable sum_im    : signed(48 downto 0);
    variable scale_re  : signed(48 downto 0);
    variable scale_im  : signed(48 downto 0);
  begin
    rr := a.re * wr;
    ii := a.im * wi;
    ri := a.re * wi;
    ir := a.im * wr;

    sum_re := resize(rr, 49) - resize(ii, 49);
    sum_im := resize(ri, 49) + resize(ir, 49);

    scale_re := div_pow2_tz(sum_re, 15);
    scale_im := div_pow2_tz(sum_im, 15);

    out_v.re := sat_resize_s32(scale_re);
    out_v.im := sat_resize_s32(scale_im);
    return out_v;
  end function;

  function bit_reverse(i : integer; bits : integer) return integer is
    variable in_v  : integer := i;
    variable out_v : integer := 0;
  begin
    for b in 0 to bits - 1 loop
      out_v := (out_v * 2) + (in_v mod 2);
      in_v := in_v / 2;
    end loop;
    return out_v;
  end function;

  function fft_radix2(x : cpx32_vec_t; inverse : boolean) return cpx32_vec_t is
    variable a      : cpx32_vec_t := (others => C_CPX_ZERO);
    variable len_i  : integer;
    variable half_i : integer;
    variable base_i : integer;
    variable j      : integer;
    variable idx    : integer;
    variable tw_idx : integer;
    variable wr     : signed(15 downto 0);
    variable wi     : signed(15 downto 0);
    variable u      : cpx32_t;
    variable t      : cpx32_t;
  begin
    for i in 0 to C_NFFT - 1 loop
      a(bit_reverse(i, C_FFT_BITS)) := x(i);
    end loop;

    len_i := 2;
    while len_i <= C_NFFT loop
      half_i := len_i / 2;
      base_i := 0;
      while base_i < C_NFFT loop
        for k in 0 to half_i - 1 loop
          j := base_i + k;
          tw_idx := (k * 1024) / len_i;
          wr := lo_cos_q15(to_unsigned(tw_idx, 10));
          if inverse then
            wi := lo_sin_q15(to_unsigned(tw_idx, 10));
          else
            wi := -lo_sin_q15(to_unsigned(tw_idx, 10));
          end if;

          t := cpx_mul_q15(a(j + half_i), wr, wi);
          u := a(j);

          a(j)          := cpx_add_sat(u, t);
          a(j + half_i) := cpx_sub_sat(u, t);
        end loop;
        base_i := base_i + len_i;
      end loop;
      len_i := len_i * 2;
    end loop;

    if inverse then
      for i in 0 to C_NFFT - 1 loop
        a(i).re := sat_resize_s32(div_pow2_tz(a(i).re, C_FFT_BITS));
        a(i).im := sat_resize_s32(div_pow2_tz(a(i).im, C_FFT_BITS));
      end loop;
    end if;

    return a;
  end function;

  function build_code_fft_input(prn_seq : prn_seq_t; code_start : integer) return cpx32_vec_t is
    variable out_v         : cpx32_vec_t := (others => C_CPX_ZERO);
    variable chip_idx      : integer;
    variable code_nco      : unsigned(31 downto 0);
    variable next_code_nco : unsigned(31 downto 0);
  begin
    chip_idx := code_start mod 1023;
    if chip_idx < 0 then
      chip_idx := chip_idx + 1023;
    end if;

    code_nco := shift_left(to_unsigned(chip_idx, 32), 21);

    for s in 0 to C_SAMPLES_PER_MS - 1 loop
      if prn_seq(chip_idx) = '1' then
        out_v(s).re := to_signed(-1, 32);
      else
        out_v(s).re := to_signed(1, 32);
      end if;
      out_v(s).im := (others => '0');

      next_code_nco := code_nco + C_CODE_NCO_FCW;
      if next_code_nco < code_nco then
        if chip_idx = 1022 then
          chip_idx := 0;
        else
          chip_idx := chip_idx + 1;
        end if;
      end if;
      code_nco := next_code_nco;
    end loop;

    return out_v;
  end function;

  function build_mixed_fft_input(
    cap_i   : sample_arr_t;
    cap_q   : sample_arr_t;
    dopp_hz : signed(15 downto 0)
  ) return cpx32_vec_t is
    variable out_v      : cpx32_vec_t := (others => C_CPX_ZERO);
    variable carr_phase : signed(31 downto 0) := (others => '0');
    variable carr_fcw   : signed(31 downto 0);
    variable phase_idx  : integer;
    variable lo_i       : signed(15 downto 0);
    variable lo_q       : signed(15 downto 0);
    variable prod_ii    : signed(31 downto 0);
    variable prod_qq    : signed(31 downto 0);
    variable prod_iq    : signed(31 downto 0);
    variable prod_qi    : signed(31 downto 0);
    variable mix_re     : signed(32 downto 0);
    variable mix_im     : signed(32 downto 0);
  begin
    carr_fcw := carr_fcw_from_hz(dopp_hz);

    for s in 0 to C_SAMPLES_PER_MS - 1 loop
      phase_idx := to_integer(unsigned(carr_phase(31 downto 22)));
      lo_i := lo_cos_q15(to_unsigned(phase_idx, 10));
      lo_q := -lo_sin_q15(to_unsigned(phase_idx, 10));

      prod_ii := cap_i(s) * lo_i;
      prod_qq := cap_q(s) * lo_q;
      prod_iq := cap_i(s) * lo_q;
      prod_qi := cap_q(s) * lo_i;

      mix_re := resize(prod_ii, 33) - resize(prod_qq, 33);
      mix_im := resize(prod_iq, 33) + resize(prod_qi, 33);

      out_v(s).re := sat_resize_s32(div_pow2_tz(mix_re, 15));
      out_v(s).im := sat_resize_s32(div_pow2_tz(mix_im, 15));

      carr_phase := carr_phase + carr_fcw;
    end loop;

    return out_v;
  end function;

  function corr0_from_spectra(
    sig_fft  : cpx32_vec_t;
    code_fft : cpx32_vec_t
  ) return cpx32_t is
    variable out_v   : cpx32_t := C_CPX_ZERO;
    variable acc_re  : signed(79 downto 0) := (others => '0');
    variable acc_im  : signed(79 downto 0) := (others => '0');
    variable prod_rr : signed(63 downto 0);
    variable prod_ii : signed(63 downto 0);
    variable prod_ir : signed(63 downto 0);
    variable prod_ri : signed(63 downto 0);
    variable term_re : signed(64 downto 0);
    variable term_im : signed(64 downto 0);
  begin
    for k in 0 to C_NFFT - 1 loop
      prod_rr := sig_fft(k).re * code_fft(k).re;
      prod_ii := sig_fft(k).im * code_fft(k).im;
      prod_ir := sig_fft(k).im * code_fft(k).re;
      prod_ri := sig_fft(k).re * code_fft(k).im;

      term_re := resize(prod_rr, 65) + resize(prod_ii, 65);
      term_im := resize(prod_ir, 65) - resize(prod_ri, 65);

      acc_re := acc_re + resize(term_re, 80);
      acc_im := acc_im + resize(term_im, 80);
    end loop;

    out_v.re := sat_resize_s32(div_pow2_tz(acc_re, C_FFT_BITS));
    out_v.im := sat_resize_s32(div_pow2_tz(acc_im, C_FFT_BITS));
    return out_v;
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
    variable coh_cfg_i         : integer;
    variable noncoh_cfg_i      : integer;
    variable code_bins_cfg_i   : integer;
    variable code_step_cfg_i   : integer;
    variable dopp_bins_cfg_i   : integer;

    variable step_i            : integer;
    variable dopp_min_i        : integer;
    variable dopp_max_i        : integer;
    variable dopp_span_i       : integer;
    variable full_dopp_bins_i  : integer;
    variable active_dopp_i     : integer;
    variable start_dopp_idx_i  : integer;
    variable active_code_i     : integer;
    variable total_bins_i      : integer;
    variable bin_i             : integer;
    variable d_i               : integer;
    variable c_i               : integer;
    variable code_i            : integer;
    variable dopp_hz_i         : integer;
    variable dopp_bin_base_i   : integer;
    variable next_prn_i        : integer;

    variable coh_metric_v      : unsigned(31 downto 0);
    variable noncoh_sum_v      : unsigned(31 downto 0);
    variable prn_metric_next_v : unsigned(31 downto 0);
    variable prn_code_next_v   : unsigned(10 downto 0);
    variable prn_dopp_next_v   : signed(15 downto 0);
    variable corr_v            : cpx32_t;
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
        prep_code_idx_r     <= 0;
        dopp_idx_r          <= 0;
        code_eval_idx_r     <= 0;
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
              prep_code_idx_r     <= 0;
              dopp_idx_r          <= 0;
              code_eval_idx_r     <= 0;
              finalize_bin_idx_r  <= 0;
              prn_eval_idx_r      <= 0;
              prn_best_metric_r   <= (others => '0');
              prn_best_code_r     <= (others => '0');
              prn_best_dopp_r     <= (others => '0');
              best_metric_r       <= (others => '0');
              best_prn_r          <= prn_start;
              best_code_r         <= (others => '0');
              best_dopp_r         <= (others => '0');

              state_r <= PRN_PREP;
            end if;

          when PRN_PREP =>
            code_fft_r(prep_code_idx_r) <= fft_radix2(
              build_code_fft_input(prn_seq_r, to_integer(bin_code_r(prep_code_idx_r))),
              false
            );

            if prep_code_idx_r + 1 >= active_code_bins_r then
              cap_sample_idx_r <= 0;
              dopp_idx_r <= 0;
              code_eval_idx_r <= 0;
              state_r <= CAPTURE_MS;
            else
              prep_code_idx_r <= prep_code_idx_r + 1;
            end if;

          when CAPTURE_MS =>
            if s_valid = '1' then
              cap_i_r(cap_sample_idx_r) <= s_i;
              cap_q_r(cap_sample_idx_r) <= s_q;
              if cap_sample_idx_r = C_SAMPLES_PER_MS - 1 then
                dopp_idx_r <= 0;
                code_eval_idx_r <= 0;
                state_r <= DOPP_PREP;
              else
                cap_sample_idx_r <= cap_sample_idx_r + 1;
              end if;
            end if;

          when DOPP_PREP =>
            dopp_bin_base_i := dopp_idx_r * active_code_bins_r;
            signal_fft_r <= fft_radix2(
              build_mixed_fft_input(cap_i_r, cap_q_r, bin_dopp_r(dopp_bin_base_i)),
              false
            );
            code_eval_idx_r <= 0;
            state_r <= DOPP_PROCESS;

          when DOPP_PROCESS =>
            bin_i := (dopp_idx_r * active_code_bins_r) + code_eval_idx_r;
            corr_v := corr0_from_spectra(signal_fft_r, code_fft_r(code_eval_idx_r));
            coh_i_acc_r(bin_i) <= coh_i_acc_r(bin_i) + resize(corr_v.re, coh_i_acc_r(bin_i)'length);
            coh_q_acc_r(bin_i) <= coh_q_acc_r(bin_i) + resize(corr_v.im, coh_q_acc_r(bin_i)'length);

            if code_eval_idx_r + 1 >= active_code_bins_r then
              if dopp_idx_r + 1 >= active_dopp_bins_r then
                if coh_ms_idx_r + 1 >= active_coh_ms_r then
                  finalize_bin_idx_r <= 0;
                  state_r <= COH_COMMIT;
                else
                  coh_ms_idx_r <= coh_ms_idx_r + 1;
                  cap_sample_idx_r <= 0;
                  state_r <= CAPTURE_MS;
                end if;
              else
                dopp_idx_r <= dopp_idx_r + 1;
                state_r <= DOPP_PREP;
              end if;
            else
              code_eval_idx_r <= code_eval_idx_r + 1;
            end if;

          when COH_COMMIT =>
            coh_metric_v := sat_add_u32(
              abs_s56_sat_u32(coh_i_acc_r(finalize_bin_idx_r)),
              abs_s56_sat_u32(coh_q_acc_r(finalize_bin_idx_r))
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
                dopp_idx_r <= 0;
                code_eval_idx_r <= 0;
                state_r <= CAPTURE_MS;
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
                next_prn_i := to_integer(prn_cur_r) + 1;
                if next_prn_i > 63 then
                  next_prn_i := 63;
                end if;

                prn_cur_r <= to_unsigned(next_prn_i, prn_cur_r'length);
                prn_seq_r <= build_prn_sequence(next_prn_i);
                prep_code_idx_r <= 0;
                cap_sample_idx_r <= 0;
                coh_ms_idx_r <= 0;
                noncoh_idx_r <= 0;
                dopp_idx_r <= 0;
                code_eval_idx_r <= 0;

                for i in 0 to C_MAX_BINS - 1 loop
                  noncoh_metric_r(i) <= (others => '0');
                  coh_i_acc_r(i) <= (others => '0');
                  coh_q_acc_r(i) <= (others => '0');
                end loop;

                state_r <= PRN_PREP;
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
