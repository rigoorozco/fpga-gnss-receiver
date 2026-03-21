library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_lock_state is
  port (
    state_i           : in  track_state_t;
    prompt_mag_i      : in  integer;
    cn0_dbhz_i        : in  integer;
    min_cn0_dbhz_i    : in  unsigned(7 downto 0);
    dll_err_q15_i     : in  integer;
    carrier_metric_i  : in  integer;
    carrier_err_q15_i : in  integer;
    carrier_lock_th_i : in  signed(15 downto 0);
    max_lock_fail_i   : in  unsigned(7 downto 0);
    lock_score_i      : in  integer;
    state_o           : out track_state_t;
    code_lock_o       : out std_logic;
    carrier_lock_o    : out std_logic;
    lock_score_o      : out integer
  );
end entity;

architecture rtl of gps_l1_ca_track_lock_state is
begin
  process (all)
    variable code_enter_v         : boolean;
    variable code_track_v         : boolean;
    variable carrier_enter_v      : boolean;
    variable carrier_track_v      : boolean;
    variable carrier_metric_eval_v: integer;
    variable carrier_enter_th_v   : integer;
    variable carrier_track_th_v   : integer;
    variable max_lock_fail_v      : integer;
    variable lock_enter_th_v      : integer;
    variable lock_exit_th_v       : integer;
    variable lock_score_v         : integer;
    variable state_v              : track_state_t;
    variable code_lock_v          : std_logic;
    variable carrier_lock_v       : std_logic;
  begin
    code_enter_v := (prompt_mag_i > C_PROMPT_MAG_MIN) and
                    (cn0_dbhz_i >= to_integer(min_cn0_dbhz_i)) and
                    (abs_i(dll_err_q15_i) < C_DLL_ERR_LOCK_MAX);

    code_track_v := (prompt_mag_i > C_PROMPT_MAG_MIN) and
                    (cn0_dbhz_i >= to_integer(min_cn0_dbhz_i)) and
                    (abs_i(dll_err_q15_i) < C_DLL_ERR_TRACK_MAX);

    carrier_enter_th_v := to_integer(carrier_lock_th_i);
    carrier_track_th_v := carrier_enter_th_v - C_CARR_LOCK_HYST_Q15;
    if carrier_track_th_v < -32768 then
      carrier_track_th_v := -32768;
    end if;

    if state_i = TRACK_PULLIN then
      carrier_metric_eval_v := abs_i(carrier_metric_i);
    else
      carrier_metric_eval_v := carrier_metric_i;
    end if;

    carrier_enter_v := (carrier_metric_eval_v >= carrier_enter_th_v) and
                       (abs_i(carrier_err_q15_i) < C_CARR_ERR_LOCK_MAX);

    carrier_track_v := (carrier_metric_eval_v >= carrier_track_th_v) and
                       (abs_i(carrier_err_q15_i) < C_CARR_ERR_TRACK_MAX);

    max_lock_fail_v := to_integer(max_lock_fail_i);
    if max_lock_fail_v < 4 then
      max_lock_fail_v := 4;
    elsif max_lock_fail_v > (C_LOCK_SCORE_MAX - 8) then
      max_lock_fail_v := C_LOCK_SCORE_MAX - 8;
    end if;

    lock_enter_th_v := max_lock_fail_v;
    lock_exit_th_v := lock_enter_th_v / 2;
    if lock_exit_th_v < 2 then
      lock_exit_th_v := 2;
    end if;

    lock_score_v := lock_score_i;
    if code_track_v and carrier_track_v then
      lock_score_v := lock_score_v + C_LOCK_SCORE_INC_BOTH;
    elsif code_track_v then
      lock_score_v := lock_score_v + C_LOCK_SCORE_INC_CODE;
    else
      lock_score_v := lock_score_v - C_LOCK_SCORE_DEC_CODE;
    end if;

    if not carrier_track_v then
      lock_score_v := lock_score_v - C_LOCK_SCORE_DEC_CARR;
    end if;

    if lock_score_v < 0 then
      lock_score_v := 0;
    elsif lock_score_v > C_LOCK_SCORE_MAX then
      lock_score_v := C_LOCK_SCORE_MAX;
    end if;

    state_v := state_i;
    code_lock_v := '0';
    carrier_lock_v := '0';

    if state_i = TRACK_PULLIN then
      if code_enter_v and carrier_enter_v and lock_score_v >= lock_enter_th_v then
        state_v := TRACK_LOCKED;
        code_lock_v := '1';
        carrier_lock_v := '1';
      else
        if code_enter_v then
          code_lock_v := '1';
        end if;
        if carrier_enter_v then
          carrier_lock_v := '1';
        end if;
      end if;
    elsif state_i = TRACK_LOCKED then
      if lock_score_v <= lock_exit_th_v then
        state_v := TRACK_PULLIN;
        code_lock_v := '0';
        carrier_lock_v := '0';
      else
        state_v := TRACK_LOCKED;
        code_lock_v := '1';
        if carrier_track_v then
          carrier_lock_v := '1';
        else
          carrier_lock_v := '0';
        end if;
      end if;
    end if;

    state_o <= state_v;
    code_lock_o <= code_lock_v;
    carrier_lock_o <= carrier_lock_v;
    lock_score_o <= lock_score_v;
  end process;
end architecture;
