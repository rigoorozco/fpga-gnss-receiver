library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_nav_store is
  generic (
    G_NUM_CHANNELS : integer := 5
  );
  port (
    clk               : in  std_logic;
    rst_n             : in  std_logic;
    nav_en_i          : in  std_logic;
    chan_nav_valid_i  : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_nav_bit_i    : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    chan_prn_i        : in  u6_arr_t(0 to G_NUM_CHANNELS - 1);
    eph_valid_prn_o   : out std_logic_vector(31 downto 0);
    nav_word_count_o  : out unsigned(31 downto 0);
    tow_seconds_o     : out unsigned(31 downto 0);
    sat_x_ecef_o      : out s32_arr_t(0 to 31);
    sat_y_ecef_o      : out s32_arr_t(0 to 31);
    sat_z_ecef_o      : out s32_arr_t(0 to 31);
    sat_clk_corr_m_o  : out s32_arr_t(0 to 31)
  );
end entity;

architecture rtl of gps_l1_ca_nav_store is
  type word30_arr_t is array (0 to G_NUM_CHANNELS - 1) of std_logic_vector(29 downto 0);
  type bitcnt_arr_t is array (0 to G_NUM_CHANNELS - 1) of integer range 0 to 29;
  type sl_ch_arr_t is array (0 to G_NUM_CHANNELS - 1) of std_logic;
  type sfmask_arr_t is array (0 to 31) of std_logic_vector(2 downto 0);

  signal eph_valid_prn_r  : std_logic_vector(31 downto 0) := (others => '0');
  signal nav_word_count_r : unsigned(31 downto 0) := (others => '0');
  signal tow_seconds_r    : unsigned(31 downto 0) := (others => '0');
  signal sat_x_ecef_r     : s32_arr_t(0 to 31);
  signal sat_y_ecef_r     : s32_arr_t(0 to 31);
  signal sat_z_ecef_r     : s32_arr_t(0 to 31);
  signal sat_clk_corr_m_r : s32_arr_t(0 to 31);

  signal word_shift_r     : word30_arr_t;
  signal bit_count_r      : bitcnt_arr_t;
  signal frame_sync_r     : sl_ch_arr_t;
  signal sf_mask_r        : sfmask_arr_t;
begin
  eph_valid_prn_o  <= eph_valid_prn_r;
  nav_word_count_o <= nav_word_count_r;
  tow_seconds_o    <= tow_seconds_r;
  sat_x_ecef_o     <= sat_x_ecef_r;
  sat_y_ecef_o     <= sat_y_ecef_r;
  sat_z_ecef_o     <= sat_z_ecef_r;
  sat_clk_corr_m_o <= sat_clk_corr_m_r;

  process (clk)
    variable prn_idx      : integer;
    variable word_v       : std_logic_vector(29 downto 0);
    variable subframe_id_v: integer;
    variable tow_z_v      : integer;
    variable delta_v      : integer;
    variable cur_v        : integer;
    variable sf_v         : std_logic_vector(2 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        eph_valid_prn_r  <= (others => '0');
        nav_word_count_r <= (others => '0');
        tow_seconds_r    <= (others => '0');
        for p in 0 to 31 loop
          -- Seed a deterministic satellite geometry placeholder.
          sat_x_ecef_r(p) <= to_signed((-13000000) + p * 800000, 32);
          sat_y_ecef_r(p) <= to_signed(10000000 - p * 600000, 32);
          sat_z_ecef_r(p) <= to_signed((-9000000) + p * 550000, 32);
          sat_clk_corr_m_r(p) <= (others => '0');
          sf_mask_r(p) <= (others => '0');
        end loop;
        for i in 0 to G_NUM_CHANNELS - 1 loop
          word_shift_r(i) <= (others => '0');
          bit_count_r(i) <= 0;
          frame_sync_r(i) <= '0';
        end loop;
      elsif nav_en_i = '1' then
        for i in 0 to G_NUM_CHANNELS - 1 loop
          if chan_nav_valid_i(i) = '1' then
            word_v := word_shift_r(i)(28 downto 0) & chan_nav_bit_i(i);
            word_shift_r(i) <= word_v;

            if bit_count_r(i) = 29 then
              bit_count_r(i) <= 0;
              nav_word_count_r <= nav_word_count_r + 1;

              -- Basic preamble-based frame sync (GPS L1 C/A preamble = 0x8B).
              if word_v(29 downto 22) = "10001011" then
                frame_sync_r(i) <= '1';
              end if;

              prn_idx := to_integer(chan_prn_i(i));
              if frame_sync_r(i) = '1' and prn_idx >= 1 and prn_idx <= 32 then
                subframe_id_v := to_integer(unsigned(word_v(10 downto 8)));
                tow_z_v := to_integer(unsigned(word_v(21 downto 11)));
                tow_seconds_r <= to_unsigned(tow_z_v * 6, tow_seconds_r'length);

                if subframe_id_v >= 1 and subframe_id_v <= 3 then
                  sf_v := sf_mask_r(prn_idx - 1);
                  sf_v(subframe_id_v - 1) := '1';
                  sf_mask_r(prn_idx - 1) <= sf_v;
                  if sf_v = "111" then
                    eph_valid_prn_r(prn_idx - 1) <= '1';
                  end if;
                end if;

                -- Ephemeris/clock placeholders updated from decoded words.
                delta_v := to_integer(unsigned(word_v(7 downto 0))) - 128;
                sat_clk_corr_m_r(prn_idx - 1) <=
                  sat_clk_corr_m_r(prn_idx - 1) + to_signed(delta_v, 32);

                cur_v := to_integer(sat_x_ecef_r(prn_idx - 1));
                sat_x_ecef_r(prn_idx - 1) <= to_signed(cur_v + delta_v * 64, 32);
                cur_v := to_integer(sat_y_ecef_r(prn_idx - 1));
                sat_y_ecef_r(prn_idx - 1) <= to_signed(cur_v - delta_v * 48, 32);
                cur_v := to_integer(sat_z_ecef_r(prn_idx - 1));
                sat_z_ecef_r(prn_idx - 1) <= to_signed(cur_v + delta_v * 32, 32);
              end if;
            else
              bit_count_r(i) <= bit_count_r(i) + 1;
            end if;

            prn_idx := to_integer(chan_prn_i(i));
            if prn_idx >= 1 and prn_idx <= 32 then
              -- Keep recent activity visible even before full subframe completion.
              if eph_valid_prn_r(prn_idx - 1) = '0' then
                eph_valid_prn_r(prn_idx - 1) <= '0';
              end if;
            end if;
          end if;
        end loop;
      end if;
    end if;
  end process;
end architecture;
