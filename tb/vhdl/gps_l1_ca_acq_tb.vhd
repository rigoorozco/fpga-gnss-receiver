library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_acq_tb is
end entity;

architecture tb of gps_l1_ca_acq_tb is
  constant C_CLK_PERIOD : time := 10 ns;

  signal clk            : std_logic := '0';
  signal rst_n          : std_logic := '0';
  signal core_en        : std_logic := '0';
  signal start_pulse    : std_logic := '0';
  signal prn_start      : unsigned(5 downto 0) := to_unsigned(3, 6);
  signal prn_stop       : unsigned(5 downto 0) := to_unsigned(3, 6);
  signal doppler_min    : signed(15 downto 0) := to_signed(-2000, 16);
  signal doppler_max    : signed(15 downto 0) := to_signed(2000, 16);
  signal doppler_step   : signed(15 downto 0) := to_signed(250, 16);
  signal detect_thresh  : unsigned(31 downto 0) := (others => '0');
  signal s_valid        : std_logic := '0';
  signal s_i            : signed(15 downto 0) := (others => '0');
  signal s_q            : signed(15 downto 0) := (others => '0');

  signal acq_done       : std_logic;
  signal acq_success    : std_logic;
  signal result_valid   : std_logic;
  signal result_prn     : unsigned(5 downto 0);
  signal result_dopp    : signed(15 downto 0);
  signal result_code    : unsigned(10 downto 0);
  signal result_metric  : unsigned(31 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_acq
    generic map (
      G_DWELL_MS => 1
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      core_en       => core_en,
      start_pulse   => start_pulse,
      prn_start     => prn_start,
      prn_stop      => prn_stop,
      doppler_min   => doppler_min,
      doppler_max   => doppler_max,
      doppler_step  => doppler_step,
      detect_thresh => detect_thresh,
      s_valid       => s_valid,
      s_i           => s_i,
      s_q           => s_q,
      acq_done      => acq_done,
      acq_success   => acq_success,
      result_valid  => result_valid,
      result_prn    => result_prn,
      result_dopp   => result_dopp,
      result_code   => result_code,
      result_metric => result_metric
    );

  stim_proc : process
    variable seen_done : boolean;
  begin
    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    core_en <= '1';
    s_valid <= '1';
    s_i <= to_signed(300, 16);
    s_q <= to_signed(50, 16);

    -- First run: threshold 0 should pass.
    detect_thresh <= (others => '0');
    start_pulse <= '1';
    wait until rising_edge(clk);
    start_pulse <= '0';

    seen_done := false;
    for i in 0 to 5000 loop
      wait until rising_edge(clk);
      if acq_done = '1' then
        seen_done := true;
        exit;
      end if;
    end loop;

    assert seen_done report "Acquisition did not finish in first run." severity failure;
    assert acq_success = '1' report "Expected acquisition success with zero threshold." severity failure;
    assert result_valid = '1' report "Expected result_valid on successful acquisition." severity failure;
    assert to_integer(result_prn) = 3 report "Expected detected PRN to match configured PRN." severity failure;

    -- Second run: very high threshold should fail.
    detect_thresh <= (others => '1');
    start_pulse <= '1';
    wait until rising_edge(clk);
    start_pulse <= '0';

    seen_done := false;
    for i in 0 to 5000 loop
      wait until rising_edge(clk);
      if acq_done = '1' then
        seen_done := true;
        exit;
      end if;
    end loop;

    assert seen_done report "Acquisition did not finish in second run." severity failure;
    assert acq_success = '0' report "Expected acquisition failure with max threshold." severity failure;
    assert result_valid = '0' report "Did not expect result_valid on failed acquisition." severity failure;
    assert result_metric /= to_unsigned(0, result_metric'length)
      report "Expected non-zero metric for exercised acquisition path." severity failure;
    assert result_dopp >= doppler_min and result_dopp <= doppler_max
      report "Estimated Doppler should stay within configured range." severity failure;

    log_msg("gps_l1_ca_acq_tb completed");
    wait;
  end process;
end architecture;
