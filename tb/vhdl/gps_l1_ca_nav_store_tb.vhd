library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_nav_store_tb is
end entity;

architecture tb of gps_l1_ca_nav_store_tb is
  constant C_CLK_PERIOD : time := 10 ns;
  constant C_NUM_CH     : integer := 1;

  signal clk              : std_logic := '0';
  signal rst_n            : std_logic := '0';
  signal nav_en_i         : std_logic := '0';
  signal chan_nav_valid_i : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_nav_bit_i   : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
  signal chan_prn_i       : u6_arr_t(0 to C_NUM_CH - 1);

  signal eph_valid_prn_o  : std_logic_vector(31 downto 0);
  signal nav_word_count_o : unsigned(31 downto 0);
  signal tow_seconds_o    : unsigned(31 downto 0);
  signal sat_x_ecef_o     : s32_arr_t(0 to 31);
  signal sat_y_ecef_o     : s32_arr_t(0 to 31);
  signal sat_z_ecef_o     : s32_arr_t(0 to 31);
  signal sat_clk_corr_m_o : s32_arr_t(0 to 31);

  function nav_parity_bits(
    data_v   : std_logic_vector(23 downto 0);
    prev_d29 : std_logic;
    prev_d30 : std_logic
  ) return std_logic_vector is
    variable p : std_logic_vector(5 downto 0);
    variable d1  : std_logic;
    variable d2  : std_logic;
    variable d3  : std_logic;
    variable d4  : std_logic;
    variable d5  : std_logic;
    variable d6  : std_logic;
    variable d7  : std_logic;
    variable d8  : std_logic;
    variable d9  : std_logic;
    variable d10 : std_logic;
    variable d11 : std_logic;
    variable d12 : std_logic;
    variable d13 : std_logic;
    variable d14 : std_logic;
    variable d15 : std_logic;
    variable d16 : std_logic;
    variable d17 : std_logic;
    variable d18 : std_logic;
    variable d19 : std_logic;
    variable d20 : std_logic;
    variable d21 : std_logic;
    variable d22 : std_logic;
    variable d23 : std_logic;
    variable d24 : std_logic;
  begin
    d1  := data_v(23);
    d2  := data_v(22);
    d3  := data_v(21);
    d4  := data_v(20);
    d5  := data_v(19);
    d6  := data_v(18);
    d7  := data_v(17);
    d8  := data_v(16);
    d9  := data_v(15);
    d10 := data_v(14);
    d11 := data_v(13);
    d12 := data_v(12);
    d13 := data_v(11);
    d14 := data_v(10);
    d15 := data_v(9);
    d16 := data_v(8);
    d17 := data_v(7);
    d18 := data_v(6);
    d19 := data_v(5);
    d20 := data_v(4);
    d21 := data_v(3);
    d22 := data_v(2);
    d23 := data_v(1);
    d24 := data_v(0);

    p(5) := d1 xor d2 xor d3 xor d5 xor d6 xor d10 xor d11 xor d12 xor d13 xor d14 xor
            d15 xor d17 xor d18 xor d20 xor d23 xor d24 xor prev_d29;
    p(4) := d2 xor d3 xor d4 xor d6 xor d7 xor d11 xor d12 xor d13 xor d14 xor d15 xor
            d16 xor d18 xor d19 xor d21 xor d24 xor prev_d29 xor prev_d30;
    p(3) := d1 xor d3 xor d4 xor d5 xor d7 xor d8 xor d12 xor d13 xor d14 xor d15 xor
            d16 xor d17 xor d19 xor d20 xor d22 xor prev_d29 xor prev_d30;
    p(2) := d2 xor d4 xor d5 xor d6 xor d8 xor d9 xor d13 xor d14 xor d15 xor d16 xor
            d17 xor d18 xor d20 xor d21 xor d23 xor prev_d30;
    p(1) := d1 xor d3 xor d5 xor d6 xor d7 xor d9 xor d10 xor d14 xor d15 xor d16 xor
            d17 xor d18 xor d19 xor d21 xor d22 xor d24 xor prev_d29;
    p(0) := d1 xor d2 xor d4 xor d6 xor d7 xor d8 xor d10 xor d11 xor d13 xor d15 xor
            d19 xor d22 xor d23 xor d24 xor prev_d30;
    return p;
  end function;

  function encode_nav_word(
    data_decoded : std_logic_vector(23 downto 0);
    prev_d29     : std_logic;
    prev_d30     : std_logic
  ) return std_logic_vector is
    variable raw_data_v : std_logic_vector(23 downto 0);
    variable parity_v   : std_logic_vector(5 downto 0);
  begin
    raw_data_v := data_decoded;
    if prev_d30 = '1' then
      raw_data_v := not raw_data_v;
    end if;
    parity_v := nav_parity_bits(data_decoded, prev_d29, prev_d30);
    return raw_data_v & parity_v;
  end function;
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_nav_store
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk              => clk,
      rst_n            => rst_n,
      nav_en_i         => nav_en_i,
      chan_nav_valid_i => chan_nav_valid_i,
      chan_nav_bit_i   => chan_nav_bit_i,
      chan_prn_i       => chan_prn_i,
      eph_valid_prn_o  => eph_valid_prn_o,
      nav_word_count_o => nav_word_count_o,
      tow_seconds_o    => tow_seconds_o,
      sat_x_ecef_o     => sat_x_ecef_o,
      sat_y_ecef_o     => sat_y_ecef_o,
      sat_z_ecef_o     => sat_z_ecef_o,
      sat_clk_corr_m_o => sat_clk_corr_m_o
    );

  stim_proc : process
    type w_arr_t is array (0 to 9) of std_logic_vector(23 downto 0);
    variable w_dec       : w_arr_t;
    variable w_raw       : std_logic_vector(29 downto 0);
    variable prev_d29_v  : std_logic;
    variable prev_d30_v  : std_logic;
    variable word_before : unsigned(31 downto 0);
  begin
    chan_prn_i(0) <= to_unsigned(1, 6);

    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';

    -- Confirm fallback satellite seeds are initialized.
    wait until rising_edge(clk);
    assert to_integer(sat_x_ecef_o(0)) = -13000000 report "Unexpected seeded sat_x for PRN1." severity failure;
    assert to_integer(sat_y_ecef_o(0)) = 10000000 report "Unexpected seeded sat_y for PRN1." severity failure;
    assert to_integer(sat_z_ecef_o(0)) = -9000000 report "Unexpected seeded sat_z for PRN1." severity failure;

    nav_en_i <= '1';

    -- Build one complete 10-word subframe with valid parity.
    w_dec(0) := x"8B0000"; -- TLM preamble in bits [23:16]
    w_dec(1) := std_logic_vector(to_unsigned((100 * 128) + 4, 24)); -- TOW z-count=100, SF ID=1
    w_dec(2) := x"123456";
    w_dec(3) := x"234567";
    w_dec(4) := x"345678";
    w_dec(5) := x"456789";
    w_dec(6) := x"56789A";
    w_dec(7) := x"6789AB";
    w_dec(8) := x"789ABC";
    w_dec(9) := x"89ABCD";

    prev_d29_v := '0';
    prev_d30_v := '0';

    for w in 0 to 9 loop
      w_raw := encode_nav_word(w_dec(w), prev_d29_v, prev_d30_v);
      for b in 29 downto 0 loop
        chan_nav_valid_i(0) <= '1';
        chan_nav_bit_i(0) <= w_raw(b);
        wait until rising_edge(clk);
      end loop;
      prev_d29_v := w_raw(1);
      prev_d30_v := w_raw(0);
    end loop;

    chan_nav_valid_i(0) <= '0';
    chan_nav_bit_i(0) <= '0';

    for i in 0 to 8 loop
      wait until rising_edge(clk);
    end loop;

    assert to_integer(nav_word_count_o) >= 10 report "Expected NAV word counter to advance after valid subframe." severity failure;
    assert to_integer(tow_seconds_o) = 600 report "Expected TOW to decode from W2 z-count (100*6)." severity failure;
    assert eph_valid_prn_o(0) = '0' report "Did not expect ephemeris-valid with only subframe 1." severity failure;

    -- With nav disabled, incoming bits must not increment counters.
    word_before := nav_word_count_o;
    nav_en_i <= '0';
    w_raw := encode_nav_word(x"8B0000", '0', '0');
    for b in 29 downto 0 loop
      chan_nav_valid_i(0) <= '1';
      chan_nav_bit_i(0) <= w_raw(b);
      wait until rising_edge(clk);
    end loop;
    chan_nav_valid_i(0) <= '0';
    wait until rising_edge(clk);

    assert nav_word_count_o = word_before report "NAV counter should hold when nav_en_i=0." severity failure;

    log_msg("gps_l1_ca_nav_store_tb completed");
    wait;
  end process;
end architecture;
