library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_chan_bank_tb is
end entity;

architecture tb of gps_l1_ca_chan_bank_tb is
  constant C_CLK_PERIOD : time := 10 ns;
  constant C_NUM_CH     : integer := 2;

  signal clk                : std_logic := '0';
  signal rst_n              : std_logic := '0';
  signal core_en            : std_logic := '0';
  signal tracking_en        : std_logic := '0';
  signal chan_enable_mask   : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '1');
  signal s_valid            : std_logic := '0';
  signal s_i                : signed(15 downto 0) := (others => '0');
  signal s_q                : signed(15 downto 0) := (others => '0');
  signal min_cn0_dbhz_i     : unsigned(7 downto 0) := to_unsigned(20, 8);
  signal carrier_lock_th_i  : signed(15 downto 0) := to_signed(19661, 16);
  signal max_lock_fail_i    : unsigned(7 downto 0) := to_unsigned(20, 8);
  signal assign_valid_i     : std_logic := '0';
  signal assign_ch_idx_i    : unsigned(7 downto 0) := (others => '0');
  signal assign_prn_i       : unsigned(5 downto 0) := (others => '0');
  signal assign_dopp_i      : signed(15 downto 0) := (others => '0');
  signal assign_code_i      : unsigned(10 downto 0) := (others => '0');

  signal chan_alloc_o       : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_state_o       : track_state_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_lock_o   : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_carrier_lock_o: std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_report_valid_o: std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_prn_o         : u6_arr_t(0 to C_NUM_CH - 1);
  signal chan_dopp_o        : s16_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_o        : u11_arr_t(0 to C_NUM_CH - 1);
  signal chan_cn0_dbhz_o    : u8_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_i_o    : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_q_o    : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_nav_valid_o   : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_nav_bit_o     : std_logic_vector(C_NUM_CH - 1 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_chan_bank
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk                => clk,
      rst_n              => rst_n,
      core_en            => core_en,
      tracking_en        => tracking_en,
      chan_enable_mask   => chan_enable_mask,
      s_valid            => s_valid,
      s_i                => s_i,
      s_q                => s_q,
      min_cn0_dbhz_i     => min_cn0_dbhz_i,
      carrier_lock_th_i  => carrier_lock_th_i,
      max_lock_fail_i    => max_lock_fail_i,
      assign_valid_i     => assign_valid_i,
      assign_ch_idx_i    => assign_ch_idx_i,
      assign_prn_i       => assign_prn_i,
      assign_dopp_i      => assign_dopp_i,
      assign_code_i      => assign_code_i,
      chan_alloc_o       => chan_alloc_o,
      chan_state_o       => chan_state_o,
      chan_code_lock_o   => chan_code_lock_o,
      chan_carrier_lock_o=> chan_carrier_lock_o,
      chan_report_valid_o=> chan_report_valid_o,
      chan_prn_o         => chan_prn_o,
      chan_dopp_o        => chan_dopp_o,
      chan_code_o        => chan_code_o,
      chan_cn0_dbhz_o    => chan_cn0_dbhz_o,
      chan_prompt_i_o    => chan_prompt_i_o,
      chan_prompt_q_o    => chan_prompt_q_o,
      chan_nav_valid_o   => chan_nav_valid_o,
      chan_nav_bit_o     => chan_nav_bit_o
    );

  stim_proc : process
  begin
    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    core_en <= '1';
    tracking_en <= '1';

    for i in 0 to 4 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "00" report "Expected all channels deallocated after reset." severity failure;

    -- Assign channel 1.
    assign_ch_idx_i <= to_unsigned(1, assign_ch_idx_i'length);
    assign_prn_i <= to_unsigned(21, 6);
    assign_dopp_i <= to_signed(350, 16);
    assign_code_i <= to_unsigned(100, 11);
    assign_valid_i <= '1';
    wait until rising_edge(clk);
    assign_valid_i <= '0';

    for i in 0 to 4 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "10" report "Expected channel 1 allocation bit to be set." severity failure;
    assert to_integer(chan_prn_o(1)) = 21 report "Expected channel 1 PRN to retune to assigned PRN." severity failure;

    -- Out-of-range index must be ignored.
    assign_ch_idx_i <= to_unsigned(3, assign_ch_idx_i'length);
    assign_prn_i <= to_unsigned(31, 6);
    assign_valid_i <= '1';
    wait until rising_edge(clk);
    assign_valid_i <= '0';

    for i in 0 to 2 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "10" report "Out-of-range assignment should not modify allocation map." severity failure;

    -- Assign channel 0.
    assign_ch_idx_i <= to_unsigned(0, assign_ch_idx_i'length);
    assign_prn_i <= to_unsigned(6, 6);
    assign_dopp_i <= to_signed(-120, 16);
    assign_code_i <= to_unsigned(55, 11);
    assign_valid_i <= '1';
    wait until rising_edge(clk);
    assign_valid_i <= '0';

    for i in 0 to 4 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "11" report "Expected both channels allocated after two valid assignments." severity failure;
    assert to_integer(chan_prn_o(0)) = 6 report "Expected channel 0 PRN to retune to assigned PRN." severity failure;

    log_msg("gps_l1_ca_chan_bank_tb completed");
    wait;
  end process;
end architecture;
