library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_observables is
  generic (
    G_NUM_CHANNELS : integer := 5
  );
  port (
    clk               : in  std_logic;
    rst_n             : in  std_logic;
    obs_en_i          : in  std_logic;
    epoch_tick_i      : in  std_logic;
    sample_counter_i  : in  unsigned(31 downto 0);
    tow_seconds_i     : in  unsigned(31 downto 0);
    chan_alloc_i      : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_code_lock_i  : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_prn_i        : in  u6_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_dopp_i       : in  s16_arr_t(0 to G_NUM_CHANNELS - 1);
    chan_code_i       : in  u11_arr_t(0 to G_NUM_CHANNELS - 1);
    eph_valid_prn_i   : in  std_logic_vector(31 downto 0);
    sat_x_ecef_i      : in  s32_arr_t(0 to 31);
    sat_y_ecef_i      : in  s32_arr_t(0 to 31);
    sat_z_ecef_i      : in  s32_arr_t(0 to 31);
    sat_clk_corr_m_i  : in  s32_arr_t(0 to 31);
    obs_valid_o       : out std_logic;
    obs_epoch_o       : out unsigned(31 downto 0);
    obs_count_o       : out unsigned(7 downto 0);
    obs_valid_mask_o  : out std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    obs_prn_o         : out u6_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_dopp_o        : out s16_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_range_o       : out u32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_x_o       : out s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_y_o       : out s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_z_o       : out s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_clk_corr_o    : out s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_first_prn_o   : out unsigned(5 downto 0);
    obs_first_range_o : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_observables is
  signal obs_valid_r       : std_logic := '0';
  signal obs_epoch_r       : unsigned(31 downto 0) := (others => '0');
  signal obs_count_r       : unsigned(7 downto 0) := (others => '0');
  signal obs_valid_mask_r  : std_logic_vector(G_NUM_CHANNELS - 1 downto 0) := (others => '0');
  signal obs_prn_r         : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_dopp_r        : s16_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_range_r       : u32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_sat_x_r       : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_sat_y_r       : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_sat_z_r       : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_clk_corr_r    : s32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal obs_first_prn_r   : unsigned(5 downto 0) := (others => '0');
  signal obs_first_range_r : unsigned(31 downto 0) := (others => '0');
begin
  obs_valid_o       <= obs_valid_r;
  obs_epoch_o       <= obs_epoch_r;
  obs_count_o       <= obs_count_r;
  obs_valid_mask_o  <= obs_valid_mask_r;
  obs_prn_o         <= obs_prn_r;
  obs_dopp_o        <= obs_dopp_r;
  obs_range_o       <= obs_range_r;
  obs_sat_x_o       <= obs_sat_x_r;
  obs_sat_y_o       <= obs_sat_y_r;
  obs_sat_z_o       <= obs_sat_z_r;
  obs_clk_corr_o    <= obs_clk_corr_r;
  obs_first_prn_o   <= obs_first_prn_r;
  obs_first_range_o <= obs_first_range_r;

  process (clk)
    variable valid_count_v : integer;
    variable first_found_v : boolean;
    variable range_v       : unsigned(31 downto 0);
    variable prn_idx_v     : integer;
    variable code_us_v     : integer;
    variable coarse_us_v   : integer;
    variable raw_pr_m_v    : integer;
    variable corr_pr_m_v   : integer;
    variable clk_corr_v    : integer;
    variable sat_x_v       : integer;
    variable sat_y_v       : integer;
    variable sat_z_v       : integer;
    variable rx_ms_v       : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        obs_valid_r       <= '0';
        obs_epoch_r       <= (others => '0');
        obs_count_r       <= (others => '0');
        obs_valid_mask_r  <= (others => '0');
        obs_first_prn_r   <= (others => '0');
        obs_first_range_r <= (others => '0');
        for i in 0 to G_NUM_CHANNELS - 1 loop
          obs_prn_r(i)   <= (others => '0');
          obs_dopp_r(i)  <= (others => '0');
          obs_range_r(i) <= (others => '0');
          obs_sat_x_r(i) <= (others => '0');
          obs_sat_y_r(i) <= (others => '0');
          obs_sat_z_r(i) <= (others => '0');
          obs_clk_corr_r(i) <= (others => '0');
        end loop;
      else
        obs_valid_r <= '0';
        if obs_en_i = '1' and epoch_tick_i = '1' then
          valid_count_v := 0;
          first_found_v := false;
          obs_valid_mask_r <= (others => '0');
          obs_epoch_r <= obs_epoch_r + 1;
          rx_ms_v := to_integer(sample_counter_i / to_unsigned(C_SAMPLES_PER_MS, sample_counter_i'length));

          for i in 0 to G_NUM_CHANNELS - 1 loop
            obs_prn_r(i)  <= chan_prn_i(i);
            obs_dopp_r(i) <= chan_dopp_i(i);
            prn_idx_v := to_integer(chan_prn_i(i)) - 1;
            if prn_idx_v >= 0 and prn_idx_v <= 31 then
              sat_x_v := to_integer(sat_x_ecef_i(prn_idx_v));
              sat_y_v := to_integer(sat_y_ecef_i(prn_idx_v));
              sat_z_v := to_integer(sat_z_ecef_i(prn_idx_v));
              clk_corr_v := to_integer(sat_clk_corr_m_i(prn_idx_v));
            else
              sat_x_v := 0;
              sat_y_v := 0;
              sat_z_v := 0;
              clk_corr_v := 0;
            end if;
            obs_sat_x_r(i) <= to_signed(sat_x_v, 32);
            obs_sat_y_r(i) <= to_signed(sat_y_v, 32);
            obs_sat_z_r(i) <= to_signed(sat_z_v, 32);
            obs_clk_corr_r(i) <= to_signed(clk_corr_v, 32);

            -- Measured TOF model:
            -- coarse term from nav timing + PRN ambiguity, fine term from code phase.
            code_us_v := (to_integer(chan_code_i(i)) * 1000) / 2048;
            coarse_us_v := 66000 +
                           ((to_integer(tow_seconds_i(2 downto 0)) * 1000 + rx_ms_v) mod 1000) +
                           (prn_idx_v * 180);
            raw_pr_m_v := integer(
              (real(coarse_us_v + code_us_v) * 299792.0 / 1000.0) + 0.5
            );

            if prn_idx_v >= 0 and prn_idx_v <= 31 and eph_valid_prn_i(prn_idx_v) = '1' then
              corr_pr_m_v := raw_pr_m_v - clk_corr_v;
            else
              corr_pr_m_v := raw_pr_m_v;
            end if;

            if corr_pr_m_v < 0 then
              corr_pr_m_v := 0;
            end if;
            range_v := to_unsigned(corr_pr_m_v, 32);
            obs_range_r(i) <= range_v;

            if chan_alloc_i(i) = '1' and chan_code_lock_i(i) = '1' then
              obs_valid_mask_r(i) <= '1';
              valid_count_v := valid_count_v + 1;
              if not first_found_v then
                first_found_v := true;
                obs_first_prn_r <= chan_prn_i(i);
                obs_first_range_r <= range_v;
              end if;
            end if;
          end loop;

          obs_count_r <= to_unsigned(valid_count_v, obs_count_r'length);
          if valid_count_v > 0 then
            obs_valid_r <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
