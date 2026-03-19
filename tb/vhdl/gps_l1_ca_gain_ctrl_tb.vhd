library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_gain_ctrl_tb is
end entity;

architecture tb of gps_l1_ca_gain_ctrl_tb is
  constant C_CLK_PER : time := 20 ns;

  constant C_BLOCK_SHIFT : integer := 4;
  constant C_BLOCK_LEN   : integer := 2 ** C_BLOCK_SHIFT;
  constant C_TARGET_PWR  : integer := 1048576;
  constant C_TARGET_HYST : integer := 262144;

  signal clk        : std_logic := '0';
  signal rst_n      : std_logic := '0';
  signal s_valid    : std_logic := '0';
  signal s_i        : signed(15 downto 0) := (others => '0');
  signal s_q        : signed(15 downto 0) := (others => '0');
  signal m_valid    : std_logic;
  signal m_i        : signed(15 downto 0);
  signal m_q        : signed(15 downto 0);
  signal gain_shift : unsigned(7 downto 0);
begin
  clk <= not clk after C_CLK_PER / 2;

  dut : entity work.gps_l1_ca_gain_ctrl
    generic map (
      G_ENABLE          => true,
      G_BLOCK_SHIFT     => C_BLOCK_SHIFT,
      G_TARGET_PWR      => C_TARGET_PWR,
      G_TARGET_HYST     => C_TARGET_HYST,
      G_MAX_GAIN_SHIFT  => 5,
      G_INIT_GAIN_SHIFT => 0
    )
    port map (
      clk          => clk,
      rst_n        => rst_n,
      s_valid      => s_valid,
      s_i          => s_i,
      s_q          => s_q,
      m_valid      => m_valid,
      m_i          => m_i,
      m_q          => m_q,
      gain_shift_o => gain_shift
    );

  stim : process
    variable sample_pow_v      : integer := 0;
    variable block_acc_v       : integer := 0;
    variable block_cnt_v       : integer := 0;
    variable last_block_avg_v  : integer := 0;
    variable final_gain_v      : integer := 0;
  begin
    rst_n <= '0';
    s_valid <= '0';
    s_i <= (others => '0');
    s_q <= (others => '0');
    wait for 200 ns;
    rst_n <= '1';
    wait for 100 ns;

    -- Phase A: low-amplitude input should force gain upward.
    s_valid <= '1';
    s_i <= to_signed(256, 16);
    s_q <= (others => '0');
    for n in 0 to (12 * C_BLOCK_LEN) - 1 loop
      wait until rising_edge(clk);
      wait for 0 ns;
    end loop;

    assert to_integer(gain_shift) > 0
      report "Phase A failed: gain did not increase for weak input."
      severity failure;

    -- Phase B: high-amplitude input should reduce gain toward 0 (no negative gain).
    s_i <= to_signed(10000, 16);
    s_q <= (others => '0');
    for n in 0 to (12 * C_BLOCK_LEN) - 1 loop
      wait until rising_edge(clk);
      wait for 0 ns;
    end loop;

    assert to_integer(gain_shift) = 0
      report "Phase B failed: gain did not return to 0 under strong input."
      severity failure;

    -- Phase C: weak input again, verify output power converges around target band.
    s_i <= to_signed(256, 16);
    s_q <= (others => '0');
    block_acc_v := 0;
    block_cnt_v := 0;
    last_block_avg_v := 0;
    for n in 0 to (20 * C_BLOCK_LEN) - 1 loop
      wait until rising_edge(clk);
      wait for 0 ns;
      if m_valid = '1' then
        sample_pow_v := to_integer(m_i) * to_integer(m_i) + to_integer(m_q) * to_integer(m_q);
        block_acc_v := block_acc_v + sample_pow_v;
        block_cnt_v := block_cnt_v + 1;
        if block_cnt_v = C_BLOCK_LEN then
          last_block_avg_v := block_acc_v / C_BLOCK_LEN;
          block_acc_v := 0;
          block_cnt_v := 0;
        end if;
      end if;
    end loop;

    assert last_block_avg_v >= (C_TARGET_PWR - C_TARGET_HYST) and
           last_block_avg_v <= (C_TARGET_PWR + C_TARGET_HYST)
      report "Phase C failed: output average power outside target band. avg_pow=" &
             integer'image(last_block_avg_v)
      severity failure;

    final_gain_v := to_integer(gain_shift);
    assert final_gain_v >= 1 and final_gain_v <= 3
      report "Phase C failed: final gain outside expected range. gain_shift=" &
             integer'image(final_gain_v)
      severity failure;

    log_msg("gps_l1_ca_gain_ctrl_tb passed. final_gain_shift=" &
            integer'image(final_gain_v) &
            ", final_avg_pow=" &
            integer'image(last_block_avg_v));

    wait;
  end process;
end architecture;
