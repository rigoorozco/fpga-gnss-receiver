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
    chan_lock_i        : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_carrier_lock_i: in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    acq_done_i         : in  std_logic;
    acq_success_i      : in  std_logic;
    acq_result_prn_i   : in  unsigned(5 downto 0);
    acq_result_dopp_i  : in  signed(15 downto 0);
    acq_result_code_i  : in  unsigned(10 downto 0);
    acq_result_metric_i: in  unsigned(31 downto 0);
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
  type u32_arr_t is array (natural range <>) of unsigned(31 downto 0);
  constant C_REPLACE_MARGIN_SHIFT : integer := 3; -- 12.5% better to replace locked channel.

  signal waiting_r           : std_logic := '0';
  signal start_pending_r     : std_logic := '0';
  signal next_prn_r          : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal chan_active_r       : std_logic_vector(G_NUM_CHANNELS - 1 downto 0) := (others => '0');
  signal chan_metric_r       : u32_arr_t(0 to G_NUM_CHANNELS - 1) := (others => (others => '0'));
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
    variable prn_next_v      : unsigned(5 downto 0);
    variable fill_found_v    : boolean;
    variable fill_idx_v      : integer;
    variable unlock_found_v  : boolean;
    variable unlock_idx_v    : integer;
    variable weak_found_v    : boolean;
    variable weak_idx_v      : integer;
    variable weak_metric_v   : unsigned(31 downto 0);
    variable do_assign_v     : boolean;
    variable target_idx_v    : integer;
    variable old_metric_ext  : unsigned(32 downto 0);
    variable new_metric_ext  : unsigned(32 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        waiting_r         <= '0';
        start_pending_r   <= '0';
        next_prn_r        <= to_unsigned(1, 6);
        chan_active_r     <= (others => '0');
        chan_metric_r     <= (others => (others => '0'));
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
          waiting_r       <= '0';
          start_pending_r <= '0';
          next_prn_r      <= prn_start_cfg;
          chan_active_r   <= (others => '0');
          chan_metric_r   <= (others => (others => '0'));
        end if;

        if core_en = '1' then
          if waiting_r = '0' and start_pending_r = '0' then
            acq_prn_start_r <= next_prn_r;
            acq_prn_stop_r  <= next_prn_r;
            start_pending_r <= '1';
          elsif waiting_r = '0' and start_pending_r = '1' then
            acq_start_pulse_r <= '1';
            waiting_r         <= '1';
            start_pending_r   <= '0';
          elsif acq_done_i = '1' then
            waiting_r <= '0';
            if acq_success_i = '1' then
              fill_found_v := false;
              fill_idx_v := 0;
              unlock_found_v := false;
              unlock_idx_v := 0;
              weak_found_v := false;
              weak_idx_v := 0;
              weak_metric_v := (others => '0');
              do_assign_v := false;
              target_idx_v := 0;

              for i in 0 to G_NUM_CHANNELS - 1 loop
                if chan_enable_mask(i) = '1' then
                  if (chan_active_r(i) = '0' or chan_alloc_i(i) = '0') and not fill_found_v then
                    fill_found_v := true;
                    fill_idx_v := i;
                  end if;

                  if chan_active_r(i) = '1' and chan_lock_i(i) = '0' and not unlock_found_v then
                    unlock_found_v := true;
                    unlock_idx_v := i;
                  end if;

                  if chan_lock_i(i) = '0' then
                    if not weak_found_v then
                      weak_found_v := true;
                      weak_idx_v := i;
                      weak_metric_v := chan_metric_r(i);
                    elsif chan_metric_r(i) < weak_metric_v then
                      weak_idx_v := i;
                      weak_metric_v := chan_metric_r(i);
                    end if;
                  end if;
                end if;
              end loop;

              if fill_found_v then
                do_assign_v := true;
                target_idx_v := fill_idx_v;
              elsif unlock_found_v then
                if acq_result_metric_i > chan_metric_r(unlock_idx_v) then
                  do_assign_v := true;
                  target_idx_v := unlock_idx_v;
                end if;
              elsif weak_found_v then
                old_metric_ext := '0' & weak_metric_v;
                new_metric_ext := '0' & acq_result_metric_i;
                if new_metric_ext > old_metric_ext + shift_right(old_metric_ext, C_REPLACE_MARGIN_SHIFT) then
                  do_assign_v := true;
                  target_idx_v := weak_idx_v;
                end if;
              end if;

              if do_assign_v then
                assign_valid_r  <= '1';
                assign_ch_idx_r <= to_unsigned(target_idx_v, assign_ch_idx_r'length);
                assign_prn_r    <= acq_result_prn_i;
                assign_dopp_r   <= acq_result_dopp_i;
                assign_code_r   <= acq_result_code_i;
                chan_active_r(target_idx_v) <= '1';
                chan_metric_r(target_idx_v) <= acq_result_metric_i;
                target_ch_idx_r <= to_unsigned(target_idx_v, target_ch_idx_r'length);
              end if;
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
