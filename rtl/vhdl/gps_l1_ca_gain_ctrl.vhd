library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_l1_ca_gain_ctrl is
  generic (
    G_ENABLE          : boolean := true;
    G_BLOCK_SHIFT     : integer := 12;
    G_TARGET_PWR      : integer := 1048576;
    G_TARGET_HYST     : integer := 262144;
    G_MAX_GAIN_SHIFT  : integer := 6;
    G_INIT_GAIN_SHIFT : integer := 0
  );
  port (
    clk     : in  std_logic;
    rst_n   : in  std_logic;
    s_valid : in  std_logic;
    s_i     : in  signed(15 downto 0);
    s_q     : in  signed(15 downto 0);
    m_valid : out std_logic;
    m_i     : out signed(15 downto 0);
    m_q     : out signed(15 downto 0);
    gain_shift_o : out unsigned(7 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_gain_ctrl is
  constant C_SAMPLE_PWR_W : integer := 32;
  constant C_ACC_W        : integer := C_SAMPLE_PWR_W + G_BLOCK_SHIFT + 1;

  signal gain_shift_r : integer range 0 to G_MAX_GAIN_SHIFT := G_INIT_GAIN_SHIFT;
  signal blk_count_r  : unsigned(G_BLOCK_SHIFT - 1 downto 0) := (others => '0');
  signal power_acc_r  : unsigned(C_ACC_W - 1 downto 0) := (others => '0');

  function sat_shift_left_s16(x : signed(15 downto 0); sh : natural) return signed is
    variable wide_v : signed(31 downto 0);
    constant C_MAX16_32 : signed(31 downto 0) := to_signed(32767, 32);
    constant C_MIN16_32 : signed(31 downto 0) := to_signed(-32768, 32);
  begin
    wide_v := shift_left(resize(x, 32), integer(sh));
    if wide_v > C_MAX16_32 then
      return to_signed(32767, 16);
    elsif wide_v < C_MIN16_32 then
      return to_signed(-32768, 16);
    else
      return resize(wide_v, 16);
    end if;
  end function;
begin
  assert G_BLOCK_SHIFT >= 1
    report "gps_l1_ca_gain_ctrl: G_BLOCK_SHIFT must be >= 1."
    severity failure;
  assert G_TARGET_PWR > 0
    report "gps_l1_ca_gain_ctrl: G_TARGET_PWR must be > 0."
    severity failure;
  assert G_TARGET_HYST >= 0 and G_TARGET_HYST < G_TARGET_PWR
    report "gps_l1_ca_gain_ctrl: G_TARGET_HYST must be >= 0 and < G_TARGET_PWR."
    severity failure;
  assert G_MAX_GAIN_SHIFT >= 0
    report "gps_l1_ca_gain_ctrl: G_MAX_GAIN_SHIFT must be >= 0."
    severity failure;
  assert G_INIT_GAIN_SHIFT >= 0 and G_INIT_GAIN_SHIFT <= G_MAX_GAIN_SHIFT
    report "gps_l1_ca_gain_ctrl: G_INIT_GAIN_SHIFT out of range."
    severity failure;
  assert G_MAX_GAIN_SHIFT <= 255
    report "gps_l1_ca_gain_ctrl: G_MAX_GAIN_SHIFT must be <= 255."
    severity failure;

  process (all)
  begin
    m_valid <= s_valid;
    gain_shift_o <= to_unsigned(gain_shift_r, gain_shift_o'length);
    if G_ENABLE then
      if s_valid = '1' then
        m_i <= sat_shift_left_s16(s_i, gain_shift_r);
        m_q <= sat_shift_left_s16(s_q, gain_shift_r);
      else
        m_i <= (others => '0');
        m_q <= (others => '0');
      end if;
    else
      m_i <= s_i;
      m_q <= s_q;
    end if;
  end process;

  process (clk)
    variable out_i_v     : signed(15 downto 0);
    variable out_q_v     : signed(15 downto 0);
    variable i_sq_v      : signed(31 downto 0);
    variable q_sq_v      : signed(31 downto 0);
    variable samp_pow_v  : unsigned(C_SAMPLE_PWR_W - 1 downto 0);
    variable samp_pow_e  : unsigned(C_SAMPLE_PWR_W downto 0);
    variable acc_next_v  : unsigned(C_ACC_W - 1 downto 0);
    variable avg_pow_v   : unsigned(C_ACC_W - 1 downto 0);
    variable low_thr_v   : unsigned(C_ACC_W - 1 downto 0);
    variable high_thr_v  : unsigned(C_ACC_W - 1 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        gain_shift_r <= G_INIT_GAIN_SHIFT;
        blk_count_r  <= (others => '0');
        power_acc_r  <= (others => '0');
      elsif G_ENABLE and s_valid = '1' then
        -- Closed-loop control uses output power (post-gain, post-saturation).
        out_i_v := sat_shift_left_s16(s_i, gain_shift_r);
        out_q_v := sat_shift_left_s16(s_q, gain_shift_r);

        i_sq_v := out_i_v * out_i_v;
        q_sq_v := out_q_v * out_q_v;
        samp_pow_e := resize(unsigned(i_sq_v), C_SAMPLE_PWR_W + 1) + resize(unsigned(q_sq_v), C_SAMPLE_PWR_W + 1);
        samp_pow_v := samp_pow_e(C_SAMPLE_PWR_W - 1 downto 0);

        acc_next_v := power_acc_r + resize(samp_pow_v, C_ACC_W);
        if blk_count_r = (blk_count_r'range => '1') then
          avg_pow_v := shift_right(acc_next_v, G_BLOCK_SHIFT);
          low_thr_v := to_unsigned(G_TARGET_PWR - G_TARGET_HYST, C_ACC_W);
          high_thr_v := to_unsigned(G_TARGET_PWR + G_TARGET_HYST, C_ACC_W);

          if avg_pow_v < low_thr_v then
            if gain_shift_r < G_MAX_GAIN_SHIFT then
              gain_shift_r <= gain_shift_r + 1;
            end if;
          elsif avg_pow_v > high_thr_v then
            if gain_shift_r > 0 then
              gain_shift_r <= gain_shift_r - 1;
            end if;
          end if;
          blk_count_r <= (others => '0');
          power_acc_r <= (others => '0');
        else
          blk_count_r <= blk_count_r + 1;
          power_acc_r <= acc_next_v;
        end if;
      end if;
    end if;
  end process;
end architecture;
