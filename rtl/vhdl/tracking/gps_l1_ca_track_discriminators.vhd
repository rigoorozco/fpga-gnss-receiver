library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_track_pkg.all;

entity gps_l1_ca_track_discriminators is
  port (
    state_i            : in  track_state_t;
    prompt_i_acc_i     : in  signed(31 downto 0);
    prompt_q_acc_i     : in  signed(31 downto 0);
    early_i_acc_i      : in  signed(31 downto 0);
    early_q_acc_i      : in  signed(31 downto 0);
    late_i_acc_i       : in  signed(31 downto 0);
    late_q_acc_i       : in  signed(31 downto 0);
    prev_prompt_i_i    : in  signed(31 downto 0);
    prev_prompt_q_i    : in  signed(31 downto 0);
    prev_prompt_valid_i: in  std_logic;
    prompt_mag_o       : out integer;
    early_mag_o        : out integer;
    late_mag_o         : out integer;
    dll_err_q15_o      : out integer;
    carrier_err_pll_q15_o : out integer;
    carrier_err_fll_q15_o : out integer;
    carrier_err_sel_q15_o : out integer;
    prompt_i_s_o       : out integer;
    prompt_q_s_o       : out integer;
    early_i_s_o        : out integer;
    early_q_s_o        : out integer;
    late_i_s_o         : out integer;
    late_q_s_o         : out integer
  );
end entity;

architecture rtl of gps_l1_ca_track_discriminators is
begin
  process (all)
    variable prompt_mag_v     : integer;
    variable early_mag_v      : integer;
    variable late_mag_v       : integer;
    variable dll_err_v        : integer;
    variable dll_den_v        : integer;
    variable ratio_q8_v       : integer;
    variable dll_err_q15_v    : integer;
    variable curr_i_v         : integer;
    variable curr_q_v         : integer;
    variable prev_i_v         : integer;
    variable prev_q_v         : integer;
    variable cross_v          : integer;
    variable dot_v            : integer;
    variable den_v            : integer;
    variable carrier_fll_v    : integer;
    variable carrier_pll_v    : integer;
  begin
    prompt_mag_v := to_integer(abs_s32(prompt_i_acc_i) + abs_s32(prompt_q_acc_i));
    early_mag_v := to_integer(abs_s32(early_i_acc_i) + abs_s32(early_q_acc_i));
    late_mag_v := to_integer(abs_s32(late_i_acc_i) + abs_s32(late_q_acc_i));

    curr_i_v := to_integer(shift_right(prompt_i_acc_i, 12));
    curr_q_v := to_integer(shift_right(prompt_q_acc_i, 12));

    dll_err_v := early_mag_v - late_mag_v;
    dll_den_v := early_mag_v + late_mag_v + 1;
    ratio_q8_v := (dll_err_v * 256) / dll_den_v;
    dll_err_q15_v := clamp_i(ratio_q8_v * 128, -32767, 32767);

    if state_i = TRACK_LOCKED then
      den_v := abs_i(curr_i_v) + 1;
      ratio_q8_v := (curr_q_v * 256) / den_v;
      carrier_pll_v := clamp_i(ratio_q8_v * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
      carrier_fll_v := carrier_pll_v;
    else
      if prev_prompt_valid_i = '1' then
        prev_i_v := to_integer(shift_right(prev_prompt_i_i, 12));
        prev_q_v := to_integer(shift_right(prev_prompt_q_i, 12));
        cross_v := prev_i_v * curr_q_v - prev_q_v * curr_i_v;
        dot_v := prev_i_v * curr_i_v + prev_q_v * curr_q_v;
        den_v := abs_i(dot_v) + 1;
        ratio_q8_v := (cross_v * 256) / den_v;
        carrier_fll_v := clamp_i(ratio_q8_v * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
      else
        den_v := abs_i(curr_i_v) + 1;
        ratio_q8_v := (curr_q_v * 256) / den_v;
        carrier_fll_v := clamp_i(ratio_q8_v * 128, -C_PHASE_ERR_MAX_Q15, C_PHASE_ERR_MAX_Q15);
      end if;
      carrier_pll_v := carrier_fll_v;
    end if;

    prompt_mag_o <= prompt_mag_v;
    early_mag_o <= early_mag_v;
    late_mag_o <= late_mag_v;
    dll_err_q15_o <= dll_err_q15_v;
    carrier_err_pll_q15_o <= carrier_pll_v;
    carrier_err_fll_q15_o <= carrier_fll_v;
    carrier_err_sel_q15_o <= carrier_fll_v when state_i = TRACK_PULLIN else carrier_pll_v;

    prompt_i_s_o <= curr_i_v;
    prompt_q_s_o <= curr_q_v;
    early_i_s_o <= to_integer(shift_right(early_i_acc_i, 12));
    early_q_s_o <= to_integer(shift_right(early_q_acc_i, 12));
    late_i_s_o <= to_integer(shift_right(late_i_acc_i, 12));
    late_q_s_o <= to_integer(shift_right(late_q_acc_i, 12));
  end process;
end architecture;
