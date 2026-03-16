library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_report_phase2 is
  port (
    clk                : in  std_logic;
    rst_n              : in  std_logic;
    report_enable_i    : in  std_logic;
    sample_counter_i   : in  unsigned(31 downto 0);

    chan_event_valid_i : in  std_logic;
    chan_idx_i         : in  unsigned(7 downto 0);
    chan_state_i       : in  track_state_t;
    chan_code_lock_i   : in  std_logic;
    chan_carrier_lock_i: in  std_logic;
    chan_prn_i         : in  unsigned(5 downto 0);
    chan_dopp_i        : in  signed(15 downto 0);
    chan_code_i        : in  unsigned(10 downto 0);
    chan_cn0_dbhz_i    : in  unsigned(7 downto 0);
    chan_nav_valid_i   : in  std_logic;
    chan_nav_bit_i     : in  std_logic;

    obs_event_valid_i  : in  std_logic;
    obs_epoch_i        : in  unsigned(31 downto 0);
    obs_count_i        : in  unsigned(7 downto 0);
    obs_first_prn_i    : in  unsigned(5 downto 0);
    obs_first_range_i  : in  unsigned(31 downto 0);

    pvt_event_valid_i  : in  std_logic;
    pvt_sats_used_i    : in  unsigned(7 downto 0);
    pvt_lat_e7_i       : in  signed(31 downto 0);
    pvt_lon_e7_i       : in  signed(31 downto 0);
    pvt_height_mm_i    : in  signed(31 downto 0);
    pvt_cbias_i        : in  signed(31 downto 0);

    tx_ready_i         : in  std_logic;
    tx_valid_o         : out std_logic;
    tx_data_o          : out std_logic_vector(7 downto 0);
    tx_last_o          : out std_logic
  );
end entity;

architecture rtl of gps_l1_ca_report_phase2 is
  constant C_PKT_LEN : integer := 24;
  type pkt_t is array (0 to C_PKT_LEN - 1) of std_logic_vector(7 downto 0);

  signal pkt_r         : pkt_t := (others => (others => '0'));
  signal pkt_busy_r    : std_logic := '0';
  signal pkt_index_r   : integer range 0 to C_PKT_LEN - 1 := 0;
  signal tx_valid_r    : std_logic := '0';
  signal tx_data_r     : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_last_r     : std_logic := '0';

  -- pragma translate_off
  constant C_LOG_UART_BYTES : boolean := false;

  function trim_image(x : integer) return string is
    constant img : string := integer'image(x);
  begin
    if img'length > 0 and img(img'left) = ' ' then
      return img(img'left + 1 to img'right);
    end if;
    return img;
  end function;

  function zpad(x : integer; width : positive) return string is
    constant src : string := trim_image(x);
    variable out_s : string(1 to width) := (others => '0');
    variable j : integer := src'right;
  begin
    for i in width downto 1 loop
      exit when j < src'left;
      out_s(i) := src(j);
      j := j - 1;
    end loop;
    return out_s;
  end function;

  function format_deg_e7(v : signed(31 downto 0)) return string is
    variable iv      : integer;
    variable abs_v   : integer;
    variable deg_v   : integer;
    variable frac_v  : integer;
  begin
    iv := to_integer(v);
    if iv < 0 then
      abs_v := -iv;
    else
      abs_v := iv;
    end if;

    deg_v := abs_v / 10000000;
    frac_v := abs_v mod 10000000;

    if iv < 0 then
      return "-" & trim_image(deg_v) & "." & zpad(frac_v, 7);
    end if;
    return trim_image(deg_v) & "." & zpad(frac_v, 7);
  end function;

  function format_height_m(v : signed(31 downto 0)) return string is
    variable iv      : integer;
    variable abs_v   : integer;
    variable int_v   : integer;
    variable frac_v  : integer;
  begin
    iv := to_integer(v);
    if iv < 0 then
      abs_v := -iv;
    else
      abs_v := iv;
    end if;

    int_v := abs_v / 1000;
    frac_v := abs_v mod 1000;

    if iv < 0 then
      return "-" & trim_image(int_v) & "." & zpad(frac_v, 3);
    end if;
    return trim_image(int_v) & "." & zpad(frac_v, 3);
  end function;

  function format_runtime_from_samples(sample_v : unsigned(31 downto 0)) return string is
    variable samples_per_sec_u : unsigned(31 downto 0);
    variable sec_offset_u      : unsigned(31 downto 0);
    variable subsec_samples_u  : unsigned(31 downto 0);
    variable sec_v             : integer;
    variable usec_v            : integer;
  begin
    samples_per_sec_u := to_unsigned(C_SAMPLE_RATE_HZ, sample_v'length);
    sec_offset_u := sample_v / samples_per_sec_u;
    subsec_samples_u := sample_v mod samples_per_sec_u;
    sec_v := to_integer(sec_offset_u);
    usec_v := integer(
      (real(to_integer(subsec_samples_u)) * 1000000.0 / real(C_SAMPLE_RATE_HZ)) + 0.5
    );
    if usec_v >= 1000000 then
      sec_v := sec_v + 1;
      usec_v := usec_v - 1000000;
    end if;
    return trim_image(sec_v) & "." & zpad(usec_v, 6) & " s";
  end function;
  -- pragma translate_on
begin
  tx_valid_o <= tx_valid_r;
  tx_data_o  <= tx_data_r;
  tx_last_o  <= tx_last_r;

  process (clk)
    variable pkt_v      : pkt_t;
    variable checksum_v : std_logic_vector(7 downto 0);
    variable state_v    : std_logic_vector(7 downto 0);
    variable pkt_type_v : std_logic_vector(7 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        pkt_r       <= (others => (others => '0'));
        pkt_busy_r  <= '0';
        pkt_index_r <= 0;
        tx_valid_r  <= '0';
        tx_data_r   <= (others => '0');
        tx_last_r   <= '0';
      else
        tx_valid_r <= '0';
        tx_last_r  <= '0';

        if report_enable_i = '1' and pkt_busy_r = '0' and
           (pvt_event_valid_i = '1' or obs_event_valid_i = '1' or chan_event_valid_i = '1') then
          pkt_v := (others => (others => '0'));
          pkt_v(0) := x"A5";
          pkt_v(1) := x"5A";
          pkt_v(3) := std_logic_vector(sample_counter_i(7 downto 0));
          pkt_v(4) := std_logic_vector(sample_counter_i(15 downto 8));
          pkt_v(5) := std_logic_vector(sample_counter_i(23 downto 16));
          pkt_v(6) := std_logic_vector(sample_counter_i(31 downto 24));

          if pvt_event_valid_i = '1' then
            pkt_type_v := x"30";
            pkt_v(7)  := std_logic_vector(pvt_sats_used_i);
            pkt_v(8)  := std_logic_vector(pvt_lat_e7_i(31 downto 24));
            pkt_v(9)  := std_logic_vector(pvt_lat_e7_i(23 downto 16));
            pkt_v(10) := std_logic_vector(pvt_lat_e7_i(15 downto 8));
            pkt_v(11) := std_logic_vector(pvt_lat_e7_i(7 downto 0));
            pkt_v(12) := std_logic_vector(pvt_lon_e7_i(31 downto 24));
            pkt_v(13) := std_logic_vector(pvt_lon_e7_i(23 downto 16));
            pkt_v(14) := std_logic_vector(pvt_lon_e7_i(15 downto 8));
            pkt_v(15) := std_logic_vector(pvt_lon_e7_i(7 downto 0));
            pkt_v(16) := std_logic_vector(pvt_height_mm_i(31 downto 24));
            pkt_v(17) := std_logic_vector(pvt_height_mm_i(23 downto 16));
            pkt_v(18) := std_logic_vector(pvt_height_mm_i(15 downto 8));
            pkt_v(19) := std_logic_vector(pvt_height_mm_i(7 downto 0));
            pkt_v(20) := std_logic_vector(pvt_cbias_i(31 downto 24));
            pkt_v(21) := std_logic_vector(pvt_cbias_i(23 downto 16));
            pkt_v(22) := std_logic_vector(pvt_cbias_i(15 downto 8));

            -- pragma translate_off
            log_msg("Position at receiver_t=" & format_runtime_from_samples(sample_counter_i) &
                    ", obs_epoch=" & integer'image(to_integer(obs_epoch_i)) &
                    " using " & integer'image(to_integer(pvt_sats_used_i)) &
                    " observations is Lat = " & format_deg_e7(pvt_lat_e7_i) &
                    " [deg], Long = " & format_deg_e7(pvt_lon_e7_i) &
                    " [deg], Height = " & format_height_m(pvt_height_mm_i) &
                    " [m], cbias=" & integer'image(to_integer(pvt_cbias_i)));
            -- pragma translate_on
          elsif obs_event_valid_i = '1' then
            pkt_type_v := x"20";
            pkt_v(7)  := std_logic_vector(obs_count_i);
            pkt_v(8)  := "00" & std_logic_vector(obs_first_prn_i);
            pkt_v(9)  := std_logic_vector(obs_epoch_i(31 downto 24));
            pkt_v(10) := std_logic_vector(obs_epoch_i(23 downto 16));
            pkt_v(11) := std_logic_vector(obs_epoch_i(15 downto 8));
            pkt_v(12) := std_logic_vector(obs_epoch_i(7 downto 0));
            pkt_v(13) := std_logic_vector(obs_first_range_i(31 downto 24));
            pkt_v(14) := std_logic_vector(obs_first_range_i(23 downto 16));
            pkt_v(15) := std_logic_vector(obs_first_range_i(15 downto 8));
            pkt_v(16) := std_logic_vector(obs_first_range_i(7 downto 0));

            -- pragma translate_off
            log_msg("UART observables report (pre-encoding): receiver_t=" &
                    format_runtime_from_samples(sample_counter_i) &
                    " epoch=" &
                    integer'image(to_integer(obs_epoch_i)) &
                    " count=" & integer'image(to_integer(obs_count_i)) &
                    " first_prn=" & integer'image(to_integer(obs_first_prn_i)) &
                    " first_range=" & integer'image(to_integer(obs_first_range_i)));
            -- pragma translate_on
          else
            pkt_type_v := x"10";
            state_v := (others => '0');
            state_v(1 downto 0) := state_to_slv(chan_state_i);
            state_v(2) := chan_code_lock_i;
            state_v(3) := chan_carrier_lock_i;
            state_v(4) := chan_nav_valid_i;
            state_v(5) := chan_nav_bit_i;

            pkt_v(7)  := std_logic_vector(chan_idx_i);
            pkt_v(8)  := "00" & std_logic_vector(chan_prn_i);
            pkt_v(9)  := state_v;
            pkt_v(10) := std_logic_vector(chan_dopp_i(15 downto 8));
            pkt_v(11) := std_logic_vector(chan_dopp_i(7 downto 0));
            pkt_v(12) := "00000" & std_logic_vector(chan_code_i(10 downto 8));
            pkt_v(13) := std_logic_vector(chan_code_i(7 downto 0));

            -- pragma translate_off
            log_msg("UART channel report (pre-encoding): ch=" &
                    integer'image(to_integer(chan_idx_i)) &
                    " prn=" & integer'image(to_integer(chan_prn_i)) &
                    " state=" & integer'image(to_integer(unsigned(state_to_slv(chan_state_i)))) &
                    " code_lock=" & std_logic'image(chan_code_lock_i) &
                    " carrier_lock=" & std_logic'image(chan_carrier_lock_i) &
                    " cn0_dbhz=" & integer'image(to_integer(chan_cn0_dbhz_i)) &
                    " doppler=" & integer'image(to_integer(chan_dopp_i)) &
                    " code_phase=" & integer'image(to_integer(chan_code_i)));
            -- pragma translate_on
          end if;

          pkt_v(2) := pkt_type_v;
          checksum_v := (others => '0');
          for i in 0 to C_PKT_LEN - 2 loop
            checksum_v := checksum_v xor pkt_v(i);
          end loop;
          pkt_v(C_PKT_LEN - 1) := checksum_v;
          pkt_r <= pkt_v;

          pkt_busy_r  <= '1';
          pkt_index_r <= 0;
        end if;

        if pkt_busy_r = '1' and tx_ready_i = '1' then
          tx_valid_r <= '1';
          tx_data_r  <= pkt_r(pkt_index_r);

          -- pragma translate_off
          if C_LOG_UART_BYTES then
            log_msg("UART PH2 byte[" & integer'image(pkt_index_r) & "] sent");
          end if;
          -- pragma translate_on

          if pkt_index_r = C_PKT_LEN - 1 then
            tx_last_r   <= '1';
            pkt_busy_r  <= '0';
            pkt_index_r <= 0;
          else
            pkt_index_r <= pkt_index_r + 1;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
