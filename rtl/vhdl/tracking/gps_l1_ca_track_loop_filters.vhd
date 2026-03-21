library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_loop_filters is
  port (
    state_i              : in  track_state_t;
    dll_err_q15_i        : in  integer;
    carrier_err_pll_q15_i: in  integer;
    carrier_err_fll_q15_i: in  integer;
    dopp_step_pullin_i   : in  unsigned(15 downto 0);
    dopp_step_lock_i     : in  unsigned(15 downto 0);
    pll_bw_hz_i          : in  unsigned(15 downto 0);
    dll_bw_hz_i          : in  unsigned(15 downto 0);
    pll_bw_narrow_hz_i   : in  unsigned(15 downto 0);
    dll_bw_narrow_hz_i   : in  unsigned(15 downto 0);
    fll_bw_hz_i          : in  unsigned(15 downto 0);
    code_loop_i_i        : in  signed(31 downto 0);
    carr_loop_i_i        : in  signed(31 downto 0);
    code_loop_i_o        : out signed(31 downto 0);
    code_fcw_o           : out unsigned(31 downto 0);
    carr_loop_i_o        : out signed(31 downto 0);
    carr_fcw_cmd_o       : out signed(31 downto 0);
    dopp_o               : out signed(15 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_track_loop_filters is
begin
  process (all)
    variable code_kp_v           : integer;
    variable code_ki_v           : integer;
    variable code_int_v          : integer;
    variable code_prop_v         : integer;
    variable code_delta_v        : integer;
    variable code_fcw_v          : unsigned(31 downto 0);
    variable carr_kp_v           : integer;
    variable carr_ki_v           : integer;
    variable carr_int_v          : integer;
    variable carr_prop_v         : integer;
    variable carr_cmd_v          : integer;
    variable carr_max_step_v     : integer;
    variable fll_gain_v          : integer;
    variable fll_step_v          : integer;
    variable pll_assist_step_v   : integer;
    variable pll_bw_sel_v        : integer;
    variable dll_bw_sel_v        : integer;
    variable dopp_v              : integer;
  begin
    if state_i = TRACK_LOCKED then
      pll_bw_sel_v := to_integer(pll_bw_narrow_hz_i);
      dll_bw_sel_v := to_integer(dll_bw_narrow_hz_i);
    else
      pll_bw_sel_v := to_integer(pll_bw_hz_i);
      dll_bw_sel_v := to_integer(dll_bw_hz_i);
    end if;

    if pll_bw_sel_v < C_PLL_GAIN_MIN_Q8_8 then
      pll_bw_sel_v := C_PLL_GAIN_MIN_Q8_8;
    end if;
    if dll_bw_sel_v < C_DLL_GAIN_MIN_Q8_8 then
      dll_bw_sel_v := C_DLL_GAIN_MIN_Q8_8;
    end if;

    code_kp_v := (dll_bw_sel_v * C_CODE_LOOP_KP_PER_HZ_Q8 + 128) / 256;
    if code_kp_v < 1 then
      code_kp_v := 1;
    end if;
    code_ki_v := code_kp_v / C_CODE_LOOP_KI_DIV;
    if code_ki_v < 1 then
      code_ki_v := 1;
    end if;

    code_int_v := to_integer(code_loop_i_i) + ((dll_err_q15_i * code_ki_v) / 32768);
    code_int_v := clamp_i(code_int_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));
    code_prop_v := (dll_err_q15_i * code_kp_v) / 32768;
    code_delta_v := clamp_i(code_int_v + code_prop_v, -to_integer(C_CODE_FCW_DELTA_MAX), to_integer(C_CODE_FCW_DELTA_MAX));

    if code_delta_v >= 0 then
      code_fcw_v := C_CODE_NCO_FCW + to_unsigned(code_delta_v, 32);
    else
      code_fcw_v := C_CODE_NCO_FCW - to_unsigned(-code_delta_v, 32);
    end if;

    if state_i = TRACK_LOCKED then
      carr_kp_v := (pll_bw_sel_v * C_CARR_FCW_PER_HZ + 128) / 256;
      if carr_kp_v < 1 then
        carr_kp_v := 1;
      end if;
      carr_ki_v := carr_kp_v / C_PLL_LOOP_KI_DIV;
      if carr_ki_v < 1 then
        carr_ki_v := 1;
      end if;

      carr_int_v := to_integer(carr_loop_i_i) + ((carrier_err_pll_q15_i * carr_ki_v) / 32768);
      carr_int_v := clamp_i(carr_int_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);

      carr_prop_v := (carrier_err_pll_q15_i * carr_kp_v) / 32768;
      carr_max_step_v := carr_fcw_from_hz_i(to_integer(dopp_step_lock_i));
      if carr_max_step_v < 1 then
        carr_max_step_v := 1;
      end if;
      carr_prop_v := clamp_i(carr_prop_v, -carr_max_step_v, carr_max_step_v);
      carr_cmd_v := clamp_i(carr_int_v + carr_prop_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
    else
      fll_gain_v := to_integer(fll_bw_hz_i);
      if fll_gain_v < C_FLL_GAIN_MIN_Q8_8 then
        fll_gain_v := C_FLL_GAIN_MIN_Q8_8;
      end if;

      carr_kp_v := ((fll_gain_v * C_CARR_FCW_PER_HZ * C_FLL_LOOP_STEP_SCALE) + 128) / 256;
      if carr_kp_v < 1 then
        carr_kp_v := 1;
      end if;

      fll_step_v := (carrier_err_fll_q15_i * carr_kp_v) / 32768;
      carr_max_step_v := carr_fcw_from_hz_i(to_integer(dopp_step_pullin_i));
      if carr_max_step_v < 1 then
        carr_max_step_v := 1;
      end if;
      fll_step_v := clamp_i(fll_step_v, -carr_max_step_v, carr_max_step_v);

      pll_assist_step_v := (carrier_err_pll_q15_i * carr_kp_v) / 32768;
      pll_assist_step_v := pll_assist_step_v / C_PULLIN_PLL_ASSIST_DIV;
      pll_assist_step_v := clamp_i(pll_assist_step_v, -carr_max_step_v, carr_max_step_v);

      carr_cmd_v := to_integer(carr_loop_i_i) + fll_step_v + pll_assist_step_v;
      carr_cmd_v := clamp_i(carr_cmd_v, -C_CARR_FCW_DELTA_MAX, C_CARR_FCW_DELTA_MAX);
      carr_int_v := carr_cmd_v;
    end if;

    dopp_v := carr_cmd_v / C_CARR_FCW_PER_HZ;

    code_loop_i_o <= to_signed(code_int_v, 32);
    code_fcw_o <= code_fcw_v;
    carr_loop_i_o <= to_signed(carr_int_v, 32);
    carr_fcw_cmd_o <= to_signed(carr_cmd_v, 32);
    dopp_o <= clamp_s16(dopp_v);
  end process;
end architecture;
