library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_loop_filters_tb is
end entity;

architecture tb of gps_l1_ca_track_loop_filters_tb is
  signal state_s              : track_state_t := TRACK_PULLIN;
  signal dll_err_q15_s        : integer := 0;
  signal carrier_err_pll_q15_s: integer := 0;
  signal carrier_err_fll_q15_s: integer := 0;
  signal dopp_step_pullin_s   : unsigned(15 downto 0) := (others => '0');
  signal dopp_step_lock_s     : unsigned(15 downto 0) := (others => '0');
  signal pll_bw_hz_s          : unsigned(15 downto 0) := (others => '0');
  signal dll_bw_hz_s          : unsigned(15 downto 0) := (others => '0');
  signal pll_bw_narrow_hz_s   : unsigned(15 downto 0) := (others => '0');
  signal dll_bw_narrow_hz_s   : unsigned(15 downto 0) := (others => '0');
  signal fll_bw_hz_s          : unsigned(15 downto 0) := (others => '0');
  signal code_loop_i_s        : signed(31 downto 0) := (others => '0');
  signal carr_loop_i_s        : signed(31 downto 0) := (others => '0');

  signal code_loop_i_o_s      : signed(31 downto 0);
  signal code_fcw_o_s         : unsigned(31 downto 0);
  signal carr_loop_i_o_s      : signed(31 downto 0);
  signal carr_fcw_cmd_o_s     : signed(31 downto 0);
  signal dopp_o_s             : signed(15 downto 0);
begin
  dut : entity work.gps_l1_ca_track_loop_filters
    port map (
      state_i               => state_s,
      dll_err_q15_i         => dll_err_q15_s,
      carrier_err_pll_q15_i => carrier_err_pll_q15_s,
      carrier_err_fll_q15_i => carrier_err_fll_q15_s,
      dopp_step_pullin_i    => dopp_step_pullin_s,
      dopp_step_lock_i      => dopp_step_lock_s,
      pll_bw_hz_i           => pll_bw_hz_s,
      dll_bw_hz_i           => dll_bw_hz_s,
      pll_bw_narrow_hz_i    => pll_bw_narrow_hz_s,
      dll_bw_narrow_hz_i    => dll_bw_narrow_hz_s,
      fll_bw_hz_i           => fll_bw_hz_s,
      code_loop_i_i         => code_loop_i_s,
      carr_loop_i_i         => carr_loop_i_s,
      code_loop_i_o         => code_loop_i_o_s,
      code_fcw_o            => code_fcw_o_s,
      carr_loop_i_o         => carr_loop_i_o_s,
      carr_fcw_cmd_o        => carr_fcw_cmd_o_s,
      dopp_o                => dopp_o_s
    );

  stim : process
    variable code_kp_v         : integer;
    variable code_ki_v         : integer;
    variable code_int_v        : integer;
    variable code_prop_v       : integer;
    variable code_delta_v      : integer;
    variable exp_code_fcw_v    : unsigned(31 downto 0);
    variable carr_kp_v         : integer;
    variable carr_ki_v         : integer;
    variable carr_int_v        : integer;
    variable carr_prop_v       : integer;
    variable carr_cmd_v        : integer;
    variable max_step_v        : integer;
    variable fll_gain_v        : integer;
    variable pll_bw_sel_v      : integer;
    variable dll_bw_sel_v      : integer;
    variable fll_step_v        : integer;
    variable pll_assist_v      : integer;
    variable exp_dopp_v        : integer;
  begin
    -- Case 1: pull-in mode.
    state_s <= TRACK_PULLIN;
    dll_err_q15_s <= 3000;
    carrier_err_pll_q15_s <= 2000;
    carrier_err_fll_q15_s <= 4000;
    dopp_step_pullin_s <= to_unsigned(80, 16);
    dopp_step_lock_s <= to_unsigned(20, 16);
    pll_bw_hz_s <= to_unsigned(8960, 16);
    dll_bw_hz_s <= to_unsigned(512, 16);
    pll_bw_narrow_hz_s <= to_unsigned(1280, 16);
    dll_bw_narrow_hz_s <= to_unsigned(128, 16);
    fll_bw_hz_s <= to_unsigned(2560, 16);
    code_loop_i_s <= to_signed(10000, 32);
    carr_loop_i_s <= to_signed(20000, 32);
    wait for 1 ns;

    pll_bw_sel_v := to_integer(pll_bw_hz_s);
    dll_bw_sel_v := to_integer(dll_bw_hz_s);
    if pll_bw_sel_v < C_PLL_GAIN_MIN_Q8_8 then
      pll_bw_sel_v := C_PLL_GAIN_MIN_Q8_8;
    end if;
    if dll_bw_sel_v < C_DLL_GAIN_MIN_Q8_8 then
      dll_bw_sel_v := C_DLL_GAIN_MIN_Q8_8;
    end if;

    code_kp_v := (dll_bw_sel_v * C_CODE_LOOP_KP_PER_HZ_Q8 + 128) / 256;
    code_ki_v := code_kp_v / C_CODE_LOOP_KI_DIV;
    if code_ki_v < 1 then
      code_ki_v := 1;
    end if;
    code_int_v := 10000 + ((3000 * code_ki_v) / 32768);
    code_int_v := clamp_i(code_int_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));
    code_prop_v := (3000 * code_kp_v) / 32768;
    code_delta_v := clamp_i(code_int_v + code_prop_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));

    if code_delta_v >= 0 then
      exp_code_fcw_v := C_CODE_NCO_FCW + to_unsigned(code_delta_v, 32);
    else
      exp_code_fcw_v := C_CODE_NCO_FCW - to_unsigned(-code_delta_v, 32);
    end if;

    fll_gain_v := to_integer(fll_bw_hz_s);
    if fll_gain_v < C_FLL_GAIN_MIN_Q8_8 then
      fll_gain_v := C_FLL_GAIN_MIN_Q8_8;
    end if;
    carr_kp_v := ((fll_gain_v * C_CARR_FCW_PER_HZ * C_FLL_LOOP_STEP_SCALE) + 128) / 256;
    if carr_kp_v < 1 then
      carr_kp_v := 1;
    end if;

    fll_step_v := (4000 * carr_kp_v) / 32768;
    max_step_v := carr_fcw_from_hz_i(80);
    fll_step_v := clamp_i(fll_step_v, -max_step_v, max_step_v);

    pll_assist_v := (2000 * carr_kp_v) / 32768;
    pll_assist_v := pll_assist_v / C_PULLIN_PLL_ASSIST_DIV;
    pll_assist_v := clamp_i(pll_assist_v, -max_step_v, max_step_v);

    carr_cmd_v := 20000 + fll_step_v + pll_assist_v;
    carr_cmd_v := clamp_i(carr_cmd_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
    carr_int_v := carr_cmd_v;
    exp_dopp_v := carr_cmd_v / C_CARR_FCW_PER_HZ;

    assert to_integer(code_loop_i_o_s) = code_int_v severity failure;
    assert code_fcw_o_s = exp_code_fcw_v severity failure;
    assert to_integer(carr_loop_i_o_s) = carr_int_v severity failure;
    assert to_integer(carr_fcw_cmd_o_s) = carr_cmd_v severity failure;
    assert to_integer(dopp_o_s) = to_integer(clamp_s16(exp_dopp_v)) severity failure;

    -- Case 2: locked mode.
    state_s <= TRACK_LOCKED;
    dll_err_q15_s <= -2500;
    carrier_err_pll_q15_s <= -1500;
    carrier_err_fll_q15_s <= 500;
    code_loop_i_s <= to_signed(2000, 32);
    carr_loop_i_s <= to_signed(3000, 32);
    wait for 1 ns;

    pll_bw_sel_v := to_integer(pll_bw_narrow_hz_s);
    dll_bw_sel_v := to_integer(dll_bw_narrow_hz_s);
    if pll_bw_sel_v < C_PLL_GAIN_MIN_Q8_8 then
      pll_bw_sel_v := C_PLL_GAIN_MIN_Q8_8;
    end if;
    if dll_bw_sel_v < C_DLL_GAIN_MIN_Q8_8 then
      dll_bw_sel_v := C_DLL_GAIN_MIN_Q8_8;
    end if;

    code_kp_v := (dll_bw_sel_v * C_CODE_LOOP_KP_PER_HZ_Q8 + 128) / 256;
    code_ki_v := code_kp_v / C_CODE_LOOP_KI_DIV;
    if code_ki_v < 1 then
      code_ki_v := 1;
    end if;
    code_int_v := 2000 + ((-2500 * code_ki_v) / 32768);
    code_int_v := clamp_i(code_int_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));
    code_prop_v := (-2500 * code_kp_v) / 32768;
    code_delta_v := clamp_i(code_int_v + code_prop_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));

    if code_delta_v >= 0 then
      exp_code_fcw_v := C_CODE_NCO_FCW + to_unsigned(code_delta_v, 32);
    else
      exp_code_fcw_v := C_CODE_NCO_FCW - to_unsigned(-code_delta_v, 32);
    end if;

    carr_kp_v := (pll_bw_sel_v * C_CARR_FCW_PER_HZ + 128) / 256;
    if carr_kp_v < 1 then
      carr_kp_v := 1;
    end if;
    carr_ki_v := carr_kp_v / C_PLL_LOOP_KI_DIV;
    if carr_ki_v < 1 then
      carr_ki_v := 1;
    end if;

    carr_int_v := 3000 + ((-1500 * carr_ki_v) / 32768);
    carr_int_v := clamp_i(carr_int_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
    carr_prop_v := (-1500 * carr_kp_v) / 32768;
    max_step_v := carr_fcw_from_hz_i(20);
    carr_prop_v := clamp_i(carr_prop_v, -max_step_v, max_step_v);
    carr_cmd_v := clamp_i(carr_int_v + carr_prop_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);

    exp_dopp_v := carr_cmd_v / C_CARR_FCW_PER_HZ;

    assert to_integer(code_loop_i_o_s) = code_int_v severity failure;
    assert code_fcw_o_s = exp_code_fcw_v severity failure;
    assert to_integer(carr_loop_i_o_s) = carr_int_v severity failure;
    assert to_integer(carr_fcw_cmd_o_s) = carr_cmd_v severity failure;
    assert to_integer(dopp_o_s) = to_integer(clamp_s16(exp_dopp_v)) severity failure;

    finish;
  end process;
end architecture;
