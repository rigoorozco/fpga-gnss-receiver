library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_chan_bank is
  generic (
    G_NUM_CHANNELS : integer := 5
  );
  port (
    clk               : in  std_logic;
    rst_n             : in  std_logic;
    core_en           : in  std_logic;
    tracking_en       : in  std_logic;
    chan_enable_mask  : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    s_valid           : in  std_logic;
    s_i               : in  signed(15 downto 0);
    s_q               : in  signed(15 downto 0);
    min_cn0_dbhz_i    : in  unsigned(7 downto 0);
    carrier_lock_th_i : in  signed(15 downto 0);
    max_lock_fail_i   : in  unsigned(7 downto 0);
    assign_valid_i    : in  std_logic;
    assign_ch_idx_i   : in  unsigned(7 downto 0);
    assign_prn_i      : in  unsigned(5 downto 0);
    assign_dopp_i     : in  signed(15 downto 0);
    assign_code_i     : in  unsigned(10 downto 0);
    chan_alloc_o      : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_state_o      : out track_state_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_code_lock_o  : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_carrier_lock_o : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_report_valid_o : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_prn_o        : out u6_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_dopp_o       : out s16_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_code_o       : out u11_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_cn0_dbhz_o   : out u8_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_prompt_i_o   : out s24_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_prompt_q_o   : out s24_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_nav_valid_o  : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_nav_bit_o    : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_chan_bank is
  signal chan_alloc_r        : std_logic_vector(G_NUM_CHANNELS - 1 downto 0) := (others => '0');
  signal chan_init_prn_r     : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_init_dopp_r    : s16_arr_t(0 to G_NUM_CHANNELS - 1);

  signal chan_state_r        : track_state_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_code_lock_r    : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_carrier_lock_r : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_report_valid_r : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_prn_r          : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_dopp_r         : s16_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_code_r         : u11_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_cn0_dbhz_r     : u8_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_prompt_i_r     : s24_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_prompt_q_r     : s24_arr_t(0 to G_NUM_CHANNELS - 1);
  signal chan_nav_valid_r    : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
  signal chan_nav_bit_r      : std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
begin
  chan_alloc_o        <= chan_alloc_r;
  chan_state_o        <= chan_state_r;
  chan_code_lock_o    <= chan_code_lock_r;
  chan_carrier_lock_o <= chan_carrier_lock_r;
  chan_report_valid_o <= chan_report_valid_r;
  chan_prn_o          <= chan_prn_r;
  chan_dopp_o         <= chan_dopp_r;
  chan_code_o         <= chan_code_r;
  chan_cn0_dbhz_o     <= chan_cn0_dbhz_r;
  chan_prompt_i_o     <= chan_prompt_i_r;
  chan_prompt_q_o     <= chan_prompt_q_r;
  chan_nav_valid_o    <= chan_nav_valid_r;
  chan_nav_bit_o      <= chan_nav_bit_r;

  assign_proc : process (clk)
    variable idx : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        chan_alloc_r <= (others => '0');
        for i in 0 to G_NUM_CHANNELS - 1 loop
          chan_init_prn_r(i)  <= to_unsigned(i + 1, 6);
          chan_init_dopp_r(i) <= (others => '0');
        end loop;
      else
        if assign_valid_i = '1' then
          idx := to_integer(assign_ch_idx_i);
          if idx >= 0 and idx < G_NUM_CHANNELS then
            chan_alloc_r(idx)     <= '1';
            chan_init_prn_r(idx)  <= assign_prn_i;
            chan_init_dopp_r(idx) <= assign_dopp_i;
          end if;
        end if;
      end if;
    end if;
  end process;

  gen_channels : for i in 0 to G_NUM_CHANNELS - 1 generate
    signal acq_valid_s : std_logic;
  begin
    acq_valid_s <= '1' when assign_valid_i = '1' and to_integer(assign_ch_idx_i) = i else '0';

    track_u : entity work.gps_l1_ca_track_chan
      port map (
        clk            => clk,
        rst_n          => rst_n,
        core_en        => core_en,
        tracking_en    => tracking_en and chan_enable_mask(i),
        init_prn       => chan_init_prn_r(i),
        init_dopp      => chan_init_dopp_r(i),
        acq_valid      => acq_valid_s,
        acq_prn        => assign_prn_i,
        acq_dopp       => assign_dopp_i,
        acq_code       => assign_code_i,
        s_valid        => s_valid,
        s_i            => s_i,
        s_q            => s_q,
        min_cn0_dbhz_i => min_cn0_dbhz_i,
        carrier_lock_th_i => carrier_lock_th_i,
        max_lock_fail_i => max_lock_fail_i,
        track_state_o  => chan_state_r(i),
        code_lock_o    => chan_code_lock_r(i),
        carrier_lock_o => chan_carrier_lock_r(i),
        report_valid_o => chan_report_valid_r(i),
        prn_o          => chan_prn_r(i),
        dopp_o         => chan_dopp_r(i),
        code_o         => chan_code_r(i),
        cn0_dbhz_o     => chan_cn0_dbhz_r(i),
        prompt_i_o     => chan_prompt_i_r(i),
        prompt_q_o     => chan_prompt_q_r(i)
      );

    nav_u : entity work.gps_l1_ca_nav
      port map (
        clk          => clk,
        rst_n        => rst_n,
        code_lock    => chan_code_lock_r(i),
        prompt_valid => chan_report_valid_r(i),
        prompt_i     => chan_prompt_i_r(i),
        nav_valid    => chan_nav_valid_r(i),
        nav_bit      => chan_nav_bit_r(i)
      );
  end generate;
end architecture;
