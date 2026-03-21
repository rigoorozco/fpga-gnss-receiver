library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

package gps_l1_ca_track_pkg is
  type int_arr_t is array (natural range <>) of integer;

  constant C_PROMPT_MAG_MIN      : integer := 2500;
  constant C_CARR_LOCK_HYST_Q15  : integer := 2048;
  constant C_DLL_ERR_LOCK_MAX    : integer := 40000;
  constant C_CARR_ERR_LOCK_MAX   : integer := 40000;
  constant C_DLL_ERR_TRACK_MAX   : integer := 40000;
  constant C_CARR_ERR_TRACK_MAX  : integer := 40000;
  constant C_LOCK_SCORE_MAX      : integer := 255;
  constant C_LOCK_SCORE_INC_BOTH : integer := 4;
  constant C_LOCK_SCORE_INC_CODE : integer := 2;
  constant C_LOCK_SCORE_DEC_CODE : integer := 6;
  constant C_LOCK_SCORE_DEC_CARR : integer := 2;
  constant C_CN0_AVG_DIV         : integer := 16;
  constant C_LOCK_SMOOTH_DIV     : integer := 8;
  constant C_CODE_LOOP_KP_PER_HZ_Q8 : integer := 32768;
  constant C_CODE_LOOP_KI_DIV       : integer := 128;
  constant C_PLL_LOOP_KI_DIV        : integer := 256;
  constant C_FLL_LOOP_STEP_SCALE    : integer := 4;
  constant C_PULLIN_PLL_ASSIST_DIV  : integer := 4;
  constant C_PHASE_ERR_MAX_Q15      : integer := 24576;
  constant C_CARR_FCW_DELTA_MAX     : integer := 64000000;
  constant C_PLL_GAIN_MIN_Q8_8      : integer := 64;
  constant C_DLL_GAIN_MIN_Q8_8      : integer := 16;
  constant C_FLL_GAIN_MIN_Q8_8      : integer := 64;
  constant C_CODE_FCW_DELTA_MAX     : unsigned(31 downto 0) := x"00200000";
  constant C_CARR_FCW_PER_HZ        : integer := 2147;

  constant C_LOG10_OCTAVE_LUT_DB100 : int_arr_t(0 to 16) := (
    0, 26, 51, 75, 97, 118, 138, 158, 176, 194, 211, 227, 243, 258, 273, 287, 301
  );

  function abs_s32(x : signed(31 downto 0)) return unsigned;
  function abs_i(x : integer) return integer;
  function clamp_i(x : integer; lo : integer; hi : integer) return integer;
  function clamp_s16(x : integer) return signed;
  function sat_s32_to_s24(x : signed(31 downto 0)) return signed;
  function acq_code_to_chip(code_v : unsigned(10 downto 0)) return unsigned;
  function carr_fcw_from_hz(dopp_hz : signed(15 downto 0)) return signed;
  function carr_fcw_from_hz_i(dopp_hz : integer) return integer;
  function ten_log10_db100(x : integer) return integer;
  function cn0_dbhz_from_powers(sig_pow : integer; noise_pow : integer) return integer;
end package;

package body gps_l1_ca_track_pkg is
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

  function carr_fcw_from_hz_i(dopp_hz : integer) return integer is
  begin
    return dopp_hz * C_CARR_FCW_PER_HZ;
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
    variable sig_i       : integer;
    variable noise_i     : integer;
    variable cn0_db100_i : integer;
    variable cn0_i       : integer;
  begin
    if sig_pow <= 0 then
      return 0;
    end if;

    sig_i := sig_pow;
    noise_i := noise_pow;
    if noise_i < 1 then
      noise_i := 1;
    end if;

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
end package body;
