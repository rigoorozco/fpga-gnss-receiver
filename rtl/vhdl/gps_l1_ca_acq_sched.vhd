library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_l1_ca_acq_sched is
  generic (
    G_NUM_CHANNELS : integer := 5
  );
  port (
    clk                : in  std_logic;
    rst_n              : in  std_logic;
    core_en            : in  std_logic;
    rescan_pulse       : in  std_logic;
    prn_start_cfg      : in  unsigned(5 downto 0);
    prn_stop_cfg       : in  unsigned(5 downto 0);
    chan_enable_mask   : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_alloc_i       : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    acq_done_i         : in  std_logic;
    acq_success_i      : in  std_logic;
    acq_result_prn_i   : in  unsigned(5 downto 0);
    acq_result_dopp_i  : in  signed(15 downto 0);
    acq_result_code_i  : in  unsigned(10 downto 0);
    acq_start_pulse_o  : out std_logic;
    acq_prn_start_o    : out unsigned(5 downto 0);
    acq_prn_stop_o     : out unsigned(5 downto 0);
    assign_valid_o     : out std_logic;
    assign_ch_idx_o    : out unsigned(7 downto 0);
    assign_prn_o       : out unsigned(5 downto 0);
    assign_dopp_o      : out signed(15 downto 0);
    assign_code_o      : out unsigned(10 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_acq_sched is
  signal waiting_r           : std_logic := '0';
  signal next_prn_r          : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal target_ch_idx_r     : unsigned(7 downto 0) := (others => '0');
  signal acq_start_pulse_r   : std_logic := '0';
  signal acq_prn_start_r     : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal acq_prn_stop_r      : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal assign_valid_r      : std_logic := '0';
  signal assign_ch_idx_r     : unsigned(7 downto 0) := (others => '0');
  signal assign_prn_r        : unsigned(5 downto 0) := (others => '0');
  signal assign_dopp_r       : signed(15 downto 0) := (others => '0');
  signal assign_code_r       : unsigned(10 downto 0) := (others => '0');
begin
  acq_start_pulse_o <= acq_start_pulse_r;
  acq_prn_start_o   <= acq_prn_start_r;
  acq_prn_stop_o    <= acq_prn_stop_r;
  assign_valid_o    <= assign_valid_r;
  assign_ch_idx_o   <= assign_ch_idx_r;
  assign_prn_o      <= assign_prn_r;
  assign_dopp_o     <= assign_dopp_r;
  assign_code_o     <= assign_code_r;

  process (clk)
    variable free_found  : boolean;
    variable free_idx    : integer;
    variable prn_next_v  : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        waiting_r         <= '0';
        next_prn_r        <= to_unsigned(1, 6);
        target_ch_idx_r   <= (others => '0');
        acq_start_pulse_r <= '0';
        acq_prn_start_r   <= to_unsigned(1, 6);
        acq_prn_stop_r    <= to_unsigned(1, 6);
        assign_valid_r    <= '0';
        assign_ch_idx_r   <= (others => '0');
        assign_prn_r      <= (others => '0');
        assign_dopp_r     <= (others => '0');
        assign_code_r     <= (others => '0');
      else
        acq_start_pulse_r <= '0';
        assign_valid_r    <= '0';

        if rescan_pulse = '1' then
          waiting_r  <= '0';
          next_prn_r <= prn_start_cfg;
        end if;

        if core_en = '1' then
          if waiting_r = '0' then
            free_found := false;
            free_idx := 0;
            for i in 0 to G_NUM_CHANNELS - 1 loop
              if chan_enable_mask(i) = '1' and chan_alloc_i(i) = '0' and not free_found then
                free_found := true;
                free_idx := i;
              end if;
            end loop;

            if free_found then
              acq_start_pulse_r <= '1';
              acq_prn_start_r   <= next_prn_r;
              acq_prn_stop_r    <= next_prn_r;
              target_ch_idx_r   <= to_unsigned(free_idx, target_ch_idx_r'length);
              waiting_r         <= '1';
            end if;
          elsif acq_done_i = '1' then
            waiting_r <= '0';
            if acq_success_i = '1' then
              assign_valid_r  <= '1';
              assign_ch_idx_r <= target_ch_idx_r;
              assign_prn_r    <= acq_result_prn_i;
              assign_dopp_r   <= acq_result_dopp_i;
              assign_code_r   <= acq_result_code_i;
            end if;

            if next_prn_r >= prn_stop_cfg then
              prn_next_v := prn_start_cfg;
            else
              prn_next_v := next_prn_r + 1;
            end if;
            next_prn_r <= prn_next_v;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
