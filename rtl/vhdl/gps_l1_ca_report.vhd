library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_report is
  port (
    clk              : in  std_logic;
    rst_n            : in  std_logic;
    report_enable    : in  std_logic;
    sample_counter   : in  unsigned(31 downto 0);
    track_state      : in  track_state_t;
    code_lock        : in  std_logic;
    carrier_lock     : in  std_logic;
    report_valid_in  : in  std_logic;
    prn              : in  unsigned(5 downto 0);
    doppler_hz       : in  signed(15 downto 0);
    code_phase       : in  unsigned(10 downto 0);
    prompt_i         : in  signed(23 downto 0);
    prompt_q         : in  signed(23 downto 0);
    nav_valid        : in  std_logic;
    nav_bit          : in  std_logic;
    tx_ready         : in  std_logic;
    tx_valid         : out std_logic;
    tx_data          : out std_logic_vector(7 downto 0);
    tx_last          : out std_logic
  );
end entity;

architecture rtl of gps_l1_ca_report is
  type pkt_t is array (0 to 15) of std_logic_vector(7 downto 0);
  signal pkt_r          : pkt_t := (others => (others => '0'));
  signal pkt_busy_r     : std_logic := '0';
  signal pkt_index_r    : integer range 0 to 15 := 0;
  signal tx_valid_r     : std_logic := '0';
  signal tx_data_r      : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_last_r      : std_logic := '0';

  -- pragma translate_off
  constant C_LOG_UART_BYTES : boolean := false;

  function nibble_to_hex(n : unsigned(3 downto 0)) return character is
  begin
    case to_integer(n) is
      when 0  => return '0';
      when 1  => return '1';
      when 2  => return '2';
      when 3  => return '3';
      when 4  => return '4';
      when 5  => return '5';
      when 6  => return '6';
      when 7  => return '7';
      when 8  => return '8';
      when 9  => return '9';
      when 10 => return 'A';
      when 11 => return 'B';
      when 12 => return 'C';
      when 13 => return 'D';
      when 14 => return 'E';
      when others => return 'F';
    end case;
  end function;

  function to_hex8(v : std_logic_vector(7 downto 0)) return string is
    variable s : string(1 to 2);
  begin
    s(1) := nibble_to_hex(unsigned(v(7 downto 4)));
    s(2) := nibble_to_hex(unsigned(v(3 downto 0)));
    return s;
  end function;

  function sl_to_text(s : std_logic) return string is
  begin
    if s = '1' then
      return "yes";
    else
      return "no";
    end if;
  end function;

  function state_to_text(s : track_state_t) return string is
  begin
    case s is
      when TRACK_IDLE   => return "IDLE";
      when TRACK_PULLIN => return "PULLIN";
      when TRACK_LOCKED => return "LOCKED";
    end case;
  end function;
  -- pragma translate_on
begin
  tx_valid <= tx_valid_r;
  tx_data  <= tx_data_r;
  tx_last  <= tx_last_r;

  process (clk)
    variable pkt_v      : pkt_t;
    variable checksum_v : std_logic_vector(7 downto 0);
    variable state_v    : std_logic_vector(7 downto 0);
    variable trig_v     : std_logic;
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
        trig_v := report_valid_in or nav_valid;

        if report_enable = '1' and pkt_busy_r = '0' and trig_v = '1' then
          state_v := (others => '0');
          state_v(1 downto 0) := state_to_slv(track_state);
          state_v(2) := code_lock;
          state_v(3) := carrier_lock;
          state_v(4) := nav_valid;
          state_v(5) := nav_bit;

          pkt_v(0)  := x"A5";
          pkt_v(1)  := x"5A";
          pkt_v(2)  := "00" & std_logic_vector(prn);
          pkt_v(3)  := state_v;
          pkt_v(4)  := std_logic_vector(sample_counter(31 downto 24));
          pkt_v(5)  := std_logic_vector(sample_counter(23 downto 16));
          pkt_v(6)  := std_logic_vector(sample_counter(15 downto 8));
          pkt_v(7)  := std_logic_vector(sample_counter(7 downto 0));
          pkt_v(8)  := std_logic_vector(doppler_hz(15 downto 8));
          pkt_v(9)  := std_logic_vector(doppler_hz(7 downto 0));
          pkt_v(10) := "00000" & std_logic_vector(code_phase(10 downto 8));
          pkt_v(11) := std_logic_vector(code_phase(7 downto 0));
          pkt_v(12) := std_logic_vector(prompt_i(15 downto 8));
          pkt_v(13) := std_logic_vector(prompt_i(7 downto 0));
          pkt_v(14) := std_logic_vector(prompt_q(7 downto 0));
          pkt_v(15) := (others => '0');

          checksum_v := (others => '0');
          for i in 0 to 14 loop
            checksum_v := checksum_v xor pkt_v(i);
          end loop;
          pkt_v(15) := checksum_v;
          pkt_r <= pkt_v;

          pkt_busy_r  <= '1';
          pkt_index_r <= 0;

          -- pragma translate_off
          log_msg("UART report (pre-encoding): " &
                  "PRN=" & integer'image(to_integer(prn)) &
                  ", state=" & state_to_text(track_state) &
                  ", sample_counter=" & integer'image(to_integer(sample_counter)) &
                  ", doppler_hz=" & integer'image(to_integer(doppler_hz)) &
                  ", code_phase=" & integer'image(to_integer(code_phase)) &
                  ", prompt_i=" & integer'image(to_integer(prompt_i)) &
                  ", prompt_q=" & integer'image(to_integer(prompt_q)) &
                  ", code_lock=" & sl_to_text(code_lock) &
                  ", carrier_lock=" & sl_to_text(carrier_lock) &
                  ", nav_valid=" & sl_to_text(nav_valid) &
                  ", nav_bit=" & std_logic'image(nav_bit));
          -- pragma translate_on
        end if;

        if pkt_busy_r = '1' and tx_ready = '1' then
          tx_valid_r <= '1';
          tx_data_r  <= pkt_r(pkt_index_r);

          -- pragma translate_off
          if C_LOG_UART_BYTES then
            log_msg("UART byte[" & integer'image(pkt_index_r) &
                    "] = 0x" & to_hex8(pkt_r(pkt_index_r)));
          end if;
          -- pragma translate_on

          if pkt_index_r = 15 then
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
