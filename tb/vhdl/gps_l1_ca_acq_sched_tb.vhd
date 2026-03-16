library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_acq_sched_tb is
end entity;

architecture tb of gps_l1_ca_acq_sched_tb is
  constant C_CLK_PERIOD : time := 10 ns;
  constant C_NUM_CH     : integer := 2;

  signal clk                 : std_logic := '0';
  signal rst_n               : std_logic := '0';
  signal core_en             : std_logic := '0';
  signal rescan_pulse        : std_logic := '0';
  signal prn_start_cfg       : unsigned(5 downto 0) := to_unsigned(5, 6);
  signal prn_stop_cfg        : unsigned(5 downto 0) := to_unsigned(7, 6);
  signal chan_enable_mask    : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '1');
  signal chan_alloc_i        : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_lock_i         : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_carrier_lock_i : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal acq_done_i          : std_logic := '0';
  signal acq_success_i       : std_logic := '0';
  signal acq_result_prn_i    : unsigned(5 downto 0) := (others => '0');
  signal acq_result_dopp_i   : signed(15 downto 0) := (others => '0');
  signal acq_result_code_i   : unsigned(10 downto 0) := (others => '0');
  signal acq_result_metric_i : unsigned(31 downto 0) := (others => '0');

  signal acq_start_pulse_o   : std_logic;
  signal acq_prn_start_o     : unsigned(5 downto 0);
  signal acq_prn_stop_o      : unsigned(5 downto 0);
  signal assign_valid_o      : std_logic;
  signal assign_ch_idx_o     : unsigned(7 downto 0);
  signal assign_prn_o        : unsigned(5 downto 0);
  signal assign_dopp_o       : signed(15 downto 0);
  signal assign_code_o       : unsigned(10 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_acq_sched
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk                 => clk,
      rst_n               => rst_n,
      core_en             => core_en,
      rescan_pulse        => rescan_pulse,
      prn_start_cfg       => prn_start_cfg,
      prn_stop_cfg        => prn_stop_cfg,
      chan_enable_mask    => chan_enable_mask,
      chan_alloc_i        => chan_alloc_i,
      chan_lock_i         => chan_lock_i,
      chan_carrier_lock_i => chan_carrier_lock_i,
      acq_done_i          => acq_done_i,
      acq_success_i       => acq_success_i,
      acq_result_prn_i    => acq_result_prn_i,
      acq_result_dopp_i   => acq_result_dopp_i,
      acq_result_code_i   => acq_result_code_i,
      acq_result_metric_i => acq_result_metric_i,
      acq_start_pulse_o   => acq_start_pulse_o,
      acq_prn_start_o     => acq_prn_start_o,
      acq_prn_stop_o      => acq_prn_stop_o,
      assign_valid_o      => assign_valid_o,
      assign_ch_idx_o     => assign_ch_idx_o,
      assign_prn_o        => assign_prn_o,
      assign_dopp_o       => assign_dopp_o,
      assign_code_o       => assign_code_o
    );

  stim_proc : process
    variable seen_start : boolean;
  begin
    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    rescan_pulse <= '1';
    wait until rising_edge(clk);
    rescan_pulse <= '0';
    core_en <= '1';

    -- Expect first acquisition start at PRN 5.
    seen_start := false;
    for i in 0 to 10 loop
      wait until rising_edge(clk);
      if acq_start_pulse_o = '1' then
        seen_start := true;
        exit;
      end if;
    end loop;
    assert seen_start report "Scheduler did not start acquisition." severity failure;
    assert to_integer(acq_prn_start_o) = 5 and to_integer(acq_prn_stop_o) = 5
      report "Expected first PRN attempt to be 5." severity failure;

    -- First success should fill channel 0.
    acq_result_prn_i <= to_unsigned(5, 6);
    acq_result_dopp_i <= to_signed(111, 16);
    acq_result_code_i <= to_unsigned(77, 11);
    acq_result_metric_i <= to_unsigned(50, 32);
    acq_success_i <= '1';
    acq_done_i <= '1';
    wait until rising_edge(clk);
    acq_done_i <= '0';
    acq_success_i <= '0';
    wait for 1 ns;
    assert assign_valid_o = '1' report "Expected channel assignment on first success." severity failure;
    assert to_integer(assign_ch_idx_o) = 0 report "Expected first fill to target channel 0." severity failure;
    assert to_integer(assign_prn_o) = 5 report "Assigned PRN mismatch for first fill." severity failure;

    chan_alloc_i <= "01";
    chan_lock_i <= "00";

    -- Next start should be PRN 6.
    seen_start := false;
    for i in 0 to 10 loop
      wait until rising_edge(clk);
      if acq_start_pulse_o = '1' then
        seen_start := true;
        exit;
      end if;
    end loop;
    assert seen_start report "Scheduler did not restart after first done." severity failure;
    assert to_integer(acq_prn_start_o) = 6 report "Expected second PRN attempt to be 6." severity failure;

    -- Second success should fill channel 1.
    acq_result_prn_i <= to_unsigned(6, 6);
    acq_result_dopp_i <= to_signed(222, 16);
    acq_result_code_i <= to_unsigned(88, 11);
    acq_result_metric_i <= to_unsigned(60, 32);
    acq_success_i <= '1';
    acq_done_i <= '1';
    wait until rising_edge(clk);
    acq_done_i <= '0';
    acq_success_i <= '0';
    wait for 1 ns;
    assert assign_valid_o = '1' report "Expected second channel assignment." severity failure;
    assert to_integer(assign_ch_idx_o) = 1 report "Expected second fill to target channel 1." severity failure;

    chan_alloc_i <= "11";
    chan_lock_i <= "11";

    -- Third start should be PRN 7.
    seen_start := false;
    for i in 0 to 10 loop
      wait until rising_edge(clk);
      if acq_start_pulse_o = '1' then
        seen_start := true;
        exit;
      end if;
    end loop;
    assert seen_start report "Scheduler did not issue third acquisition start." severity failure;
    assert to_integer(acq_prn_start_o) = 7 report "Expected third PRN attempt to be 7." severity failure;

    -- With all channels locked, a success should not replace anything.
    acq_result_prn_i <= to_unsigned(7, 6);
    acq_result_metric_i <= to_unsigned(70, 32);
    acq_success_i <= '1';
    acq_done_i <= '1';
    wait until rising_edge(clk);
    acq_done_i <= '0';
    acq_success_i <= '0';
    wait for 1 ns;
    assert assign_valid_o = '0' report "Did not expect replacement when all channels are locked." severity failure;

    -- Wrap should go back to PRN 5.
    seen_start := false;
    for i in 0 to 10 loop
      wait until rising_edge(clk);
      if acq_start_pulse_o = '1' then
        seen_start := true;
        exit;
      end if;
    end loop;
    assert seen_start report "Scheduler did not wrap acquisition sequence." severity failure;
    assert to_integer(acq_prn_start_o) = 5 report "Expected PRN scan wrap to 5." severity failure;

    -- Unlock channel 0 and allow stronger candidate replacement.
    chan_lock_i <= "10";
    acq_result_prn_i <= to_unsigned(5, 6);
    acq_result_metric_i <= to_unsigned(100, 32);
    acq_success_i <= '1';
    acq_done_i <= '1';
    wait until rising_edge(clk);
    acq_done_i <= '0';
    acq_success_i <= '0';
    wait for 1 ns;
    assert assign_valid_o = '1' report "Expected replacement for unlocked weak channel." severity failure;
    assert to_integer(assign_ch_idx_o) = 0 report "Expected unlocked channel 0 to be replaced first." severity failure;

    log_msg("gps_l1_ca_acq_sched_tb completed");
    wait;
  end process;
end architecture;
