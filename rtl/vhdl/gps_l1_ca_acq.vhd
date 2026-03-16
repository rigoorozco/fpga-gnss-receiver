library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_acq is
  generic (
    G_DWELL_MS : integer := 2
  );
  port (
    clk            : in  std_logic;
    rst_n          : in  std_logic;
    core_en        : in  std_logic;
    start_pulse    : in  std_logic;
    prn_start      : in  unsigned(5 downto 0);
    prn_stop       : in  unsigned(5 downto 0);
    doppler_min    : in  signed(15 downto 0);
    doppler_max    : in  signed(15 downto 0);
    doppler_step   : in  signed(15 downto 0);
    detect_thresh  : in  unsigned(31 downto 0);
    s_valid        : in  std_logic;
    s_i            : in  signed(15 downto 0);
    s_q            : in  signed(15 downto 0);
    acq_done       : out std_logic;
    acq_success    : out std_logic;
    result_valid   : out std_logic;
    result_prn     : out unsigned(5 downto 0);
    result_dopp    : out signed(15 downto 0);
    result_code    : out unsigned(10 downto 0);
    result_metric  : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_acq is
  type state_t is (IDLE, COLLECT, FINALIZE);

  signal state_r             : state_t := IDLE;
  signal prn_cur_r           : unsigned(5 downto 0) := to_unsigned(1, 6);

  signal sample_cnt_r        : integer range 0 to C_SAMPLES_PER_MS - 1 := 0;
  signal dwell_cnt_r         : integer range 0 to G_DWELL_MS - 1 := 0;

  signal code_nco_phase_r    : unsigned(31 downto 0) := (others => '0');
  signal code_chip_idx_r     : unsigned(10 downto 0) := (others => '0');

  signal corr_i_acc_r        : signed(31 downto 0) := (others => '0');
  signal corr_q_acc_r        : signed(31 downto 0) := (others => '0');
  signal noncoh_metric_r     : unsigned(31 downto 0) := (others => '0');

  signal prev_corr_i_r       : signed(31 downto 0) := (others => '0');
  signal prev_corr_q_r       : signed(31 downto 0) := (others => '0');
  signal prev_corr_valid_r   : std_logic := '0';

  signal best_metric_r       : unsigned(31 downto 0) := (others => '0');
  signal best_prn_r          : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal best_code_r         : unsigned(10 downto 0) := (others => '0');
  signal best_dopp_r         : signed(15 downto 0) := (others => '0');

  signal peak_metric_r       : unsigned(31 downto 0) := (others => '0');
  signal peak_code_r         : unsigned(10 downto 0) := (others => '0');

  signal acq_done_r          : std_logic := '0';
  signal acq_success_r       : std_logic := '0';
  signal result_valid_r      : std_logic := '0';

  signal prn_init_r          : std_logic := '0';
  signal prn_chip_advance_r  : std_logic := '0';
  signal prn_chip_s          : std_logic;

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

begin
  acq_done      <= acq_done_r;
  acq_success   <= acq_success_r;
  result_valid  <= result_valid_r;
  result_prn    <= best_prn_r;
  result_dopp   <= best_dopp_r;
  result_code   <= best_code_r;
  result_metric <= best_metric_r;

  prn_gen_u : entity work.ca_prn_gen
    port map (
      clk          => clk,
      rst_n        => rst_n,
      init         => prn_init_r,
      chip_advance => prn_chip_advance_r,
      prn          => prn_cur_r,
      chip         => prn_chip_s
    );

  process (clk)
    variable d_i              : signed(15 downto 0);
    variable d_q              : signed(15 downto 0);
    variable next_i           : signed(31 downto 0);
    variable next_q           : signed(31 downto 0);
    variable next_code_nco    : unsigned(31 downto 0);
    variable chip_adv_v       : std_logic;
    variable code_idx_v       : unsigned(10 downto 0);
    variable coh_metric_v     : unsigned(31 downto 0);
    variable noncoh_sum_v     : unsigned(31 downto 0);
    variable prev_i_int       : integer;
    variable prev_q_int       : integer;
    variable curr_i_int       : integer;
    variable curr_q_int       : integer;
    variable cross_i          : integer;
    variable dot_i            : integer;
    variable abs_cross_i      : integer;
    variable abs_dot_i        : integer;
    variable step_mag_i       : integer;
    variable doppler_step_i   : integer;
    variable doppler_min_i    : integer;
    variable doppler_max_i    : integer;
    variable doppler_mid_i    : integer;
    variable doppler_est_i    : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        state_r            <= IDLE;
        prn_cur_r          <= to_unsigned(1, 6);
        sample_cnt_r       <= 0;
        dwell_cnt_r        <= 0;
        code_nco_phase_r   <= (others => '0');
        code_chip_idx_r    <= (others => '0');
        corr_i_acc_r       <= (others => '0');
        corr_q_acc_r       <= (others => '0');
        noncoh_metric_r    <= (others => '0');
        prev_corr_i_r      <= (others => '0');
        prev_corr_q_r      <= (others => '0');
        prev_corr_valid_r  <= '0';
        best_metric_r      <= (others => '0');
        best_prn_r         <= to_unsigned(1, 6);
        best_code_r        <= (others => '0');
        best_dopp_r        <= (others => '0');
        peak_metric_r      <= (others => '0');
        peak_code_r        <= (others => '0');
        acq_done_r         <= '0';
        acq_success_r      <= '0';
        result_valid_r     <= '0';
        prn_init_r         <= '0';
        prn_chip_advance_r <= '0';
      else
        acq_done_r         <= '0';
        result_valid_r     <= '0';
        prn_init_r         <= '0';
        prn_chip_advance_r <= '0';

        case state_r is
          when IDLE =>
            acq_success_r <= '0';
            if core_en = '1' and start_pulse = '1' then
              state_r           <= COLLECT;
              prn_cur_r         <= prn_start;
              sample_cnt_r      <= 0;
              dwell_cnt_r       <= 0;
              code_nco_phase_r  <= (others => '0');
              code_chip_idx_r   <= (others => '0');
              corr_i_acc_r      <= (others => '0');
              corr_q_acc_r      <= (others => '0');
              noncoh_metric_r   <= (others => '0');
              prev_corr_i_r     <= (others => '0');
              prev_corr_q_r     <= (others => '0');
              prev_corr_valid_r <= '0';
              peak_metric_r     <= (others => '0');
              peak_code_r       <= (others => '0');
              best_metric_r     <= (others => '0');
              best_prn_r        <= prn_start;
              best_code_r       <= (others => '0');
              best_dopp_r       <= (others => '0');
              prn_init_r        <= '1';
            end if;

          when COLLECT =>
            if s_valid = '1' then
              if prn_chip_s = '1' then
                d_i := -s_i;
                d_q := -s_q;
              else
                d_i := s_i;
                d_q := s_q;
              end if;

              next_i := corr_i_acc_r + resize(d_i, corr_i_acc_r'length);
              next_q := corr_q_acc_r + resize(d_q, corr_q_acc_r'length);

              next_code_nco := code_nco_phase_r + C_CODE_NCO_FCW;
              chip_adv_v := '0';
              if next_code_nco < code_nco_phase_r then
                chip_adv_v := '1';
              end if;

              code_idx_v := code_chip_idx_r;
              if chip_adv_v = '1' then
                prn_chip_advance_r <= '1';
                if code_chip_idx_r = to_unsigned(1022, code_chip_idx_r'length) then
                  code_idx_v := (others => '0');
                else
                  code_idx_v := code_chip_idx_r + 1;
                end if;
              end if;

              corr_i_acc_r     <= next_i;
              corr_q_acc_r     <= next_q;
              code_nco_phase_r <= next_code_nco;
              code_chip_idx_r  <= code_idx_v;

              if sample_cnt_r = C_SAMPLES_PER_MS - 1 then
                coh_metric_v := abs_s32(next_i) + abs_s32(next_q);
                noncoh_sum_v := noncoh_metric_r + coh_metric_v;

                if coh_metric_v > peak_metric_r then
                  peak_metric_r <= coh_metric_v;
                  peak_code_r   <= code_idx_v;
                end if;

                if prev_corr_valid_r = '1' then
                  prev_i_int := to_integer(shift_right(prev_corr_i_r, 12));
                  prev_q_int := to_integer(shift_right(prev_corr_q_r, 12));
                  curr_i_int := to_integer(shift_right(next_i, 12));
                  curr_q_int := to_integer(shift_right(next_q, 12));

                  cross_i := prev_i_int * curr_q_int - prev_q_int * curr_i_int;
                  dot_i   := prev_i_int * curr_i_int + prev_q_int * curr_q_int;

                  if cross_i < 0 then
                    abs_cross_i := -cross_i;
                  else
                    abs_cross_i := cross_i;
                  end if;

                  if dot_i < 0 then
                    abs_dot_i := -dot_i;
                  else
                    abs_dot_i := dot_i;
                  end if;

                  step_mag_i := 0;
                  if abs_cross_i > (abs_dot_i * 4) then
                    step_mag_i := 4;
                  elsif abs_cross_i > (abs_dot_i * 2) then
                    step_mag_i := 3;
                  elsif abs_cross_i > abs_dot_i then
                    step_mag_i := 2;
                  elsif abs_cross_i > (abs_dot_i / 2) then
                    step_mag_i := 1;
                  end if;

                  doppler_step_i := to_integer(doppler_step);
                  if doppler_step_i < 0 then
                    doppler_step_i := -doppler_step_i;
                  end if;
                  doppler_min_i := to_integer(doppler_min);
                  doppler_max_i := to_integer(doppler_max);
                  doppler_mid_i := (doppler_min_i + doppler_max_i) / 2;

                  if cross_i > 0 then
                    doppler_est_i := doppler_mid_i + step_mag_i * doppler_step_i;
                  elsif cross_i < 0 then
                    doppler_est_i := doppler_mid_i - step_mag_i * doppler_step_i;
                  else
                    doppler_est_i := doppler_mid_i;
                  end if;

                  if doppler_est_i < doppler_min_i then
                    doppler_est_i := doppler_min_i;
                  elsif doppler_est_i > doppler_max_i then
                    doppler_est_i := doppler_max_i;
                  end if;

                  if noncoh_sum_v >= best_metric_r then
                    best_dopp_r <= clamp_s16(doppler_est_i);
                  end if;
                end if;

                prev_corr_i_r      <= next_i;
                prev_corr_q_r      <= next_q;
                prev_corr_valid_r  <= '1';

                sample_cnt_r       <= 0;
                corr_i_acc_r       <= (others => '0');
                corr_q_acc_r       <= (others => '0');
                code_nco_phase_r   <= (others => '0');
                code_chip_idx_r    <= (others => '0');
                prn_init_r         <= '1';

                if dwell_cnt_r = G_DWELL_MS - 1 then
                  if noncoh_sum_v >= best_metric_r then
                    best_metric_r <= noncoh_sum_v;
                    best_prn_r    <= prn_cur_r;
                    best_code_r   <= peak_code_r;
                  end if;

                  noncoh_metric_r   <= (others => '0');
                  dwell_cnt_r       <= 0;
                  peak_metric_r     <= (others => '0');
                  peak_code_r       <= (others => '0');
                  prev_corr_valid_r <= '0';

                  if prn_cur_r >= prn_stop then
                    state_r <= FINALIZE;
                  else
                    prn_cur_r <= prn_cur_r + 1;
                  end if;
                else
                  dwell_cnt_r     <= dwell_cnt_r + 1;
                  noncoh_metric_r <= noncoh_sum_v;
                end if;
              else
                sample_cnt_r <= sample_cnt_r + 1;
              end if;
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
