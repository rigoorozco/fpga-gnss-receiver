library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_observables is
  generic (
    G_NUM_CHANNELS            : integer := 5;
    G_REQUIRE_EPH_FOR_VALID   : boolean := true;
    G_IONO_CORR_MM            : integer := 0;
    G_TROPO_CORR_MM           : integer := 0
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
    rx_est_valid_i    : in  std_logic;
    rx_est_lat_e7_i   : in  signed(31 downto 0);
    rx_est_lon_e7_i   : in  signed(31 downto 0);
    rx_est_height_mm_i: in  signed(31 downto 0);
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
  constant C_LIGHT_MPS      : real := 299792458.0;
  constant C_M_PER_MS       : real := 299792.458;
  constant C_L1_FREQ_HZ     : real := 1575420000.0;
  constant C_L1_LAMBDA_M    : real := C_LIGHT_MPS / C_L1_FREQ_HZ;
  constant C_OMEGA_E_RADPS  : real := 7.2921151467E-5;
  constant C_WGS84_A        : real := 6378137.0;
  constant C_WGS84_E2       : real := 6.69437999014E-3;

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

  signal prev_range_r      : u32_arr_t(0 to G_NUM_CHANNELS - 1);
  signal prev_valid_r      : std_logic_vector(G_NUM_CHANNELS - 1 downto 0) := (others => '0');
  signal prev_sample_cnt_r : unsigned(31 downto 0) := (others => '0');
  signal prev_epoch_valid_r: std_logic := '0';

  function real_to_i32(x : real) return integer is
    variable y : real;
  begin
    y := x;
    if y > 2147483647.0 then
      return 2147483647;
    elsif y < -2147483648.0 then
      return -2147483648;
    elsif y >= 0.0 then
      return integer(y + 0.5);
    else
      return integer(y - 0.5);
    end if;
  end function;

  function real_to_u32(x : real) return u32_t is
    variable y : real;
    variable iv : integer;
  begin
    y := x;
    if y <= 0.0 then
      return (others => '0');
    elsif y >= 4294967295.0 then
      return (others => '1');
    else
      iv := integer(y + 0.5);
      return to_unsigned(iv, 32);
    end if;
  end function;

  function clamp_s16(x : integer) return signed is
  begin
    if x > 32767 then
      return to_signed(32767, 16);
    elsif x < -32768 then
      return to_signed(-32768, 16);
    else
      return to_signed(x, 16);
    end if;
  end function;
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
    variable valid_count_v    : integer;
    variable first_found_v    : boolean;
    variable prn_idx_v        : integer;

    variable rx_x_m_v         : real;
    variable rx_y_m_v         : real;
    variable rx_z_m_v         : real;
    variable lat_rad_v        : real;
    variable lon_rad_v        : real;
    variable h_m_v            : real;
    variable sin_lat_v        : real;
    variable cos_lat_v        : real;
    variable n_v              : real;

    variable sat_x_m_v        : real;
    variable sat_y_m_v        : real;
    variable sat_z_m_v        : real;
    variable sat_clk_m_v      : real;

    variable dx_v             : real;
    variable dy_v             : real;
    variable dz_v             : real;
    variable geom_range_m_v   : real;
    variable geom_ms_v        : real;
    variable coarse_ms_v      : integer;
    variable code_frac_ms_v   : real;
    variable tof_raw_m_v      : real;

    variable sagnac_m_v       : real;
    variable iono_m_v         : real;
    variable tropo_m_v        : real;
    variable corr_total_m_v   : real;
    variable pr_corr_m_v      : real;
    variable pr_range_u_v     : unsigned(31 downto 0);

    variable dt_s_v           : real;
    variable rr_meas_mps_v    : real;
    variable rr_dopp_mps_v    : real;
    variable rr_blend_mps_v   : real;
    variable dopp_corr_hz_v   : integer;

    variable eph_ok_v         : boolean;
    variable lock_ok_v        : boolean;
    variable range_ok_v       : boolean;
    variable rate_ok_v        : boolean;
    variable valid_obs_v      : boolean;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        obs_valid_r       <= '0';
        obs_epoch_r       <= (others => '0');
        obs_count_r       <= (others => '0');
        obs_valid_mask_r  <= (others => '0');
        obs_first_prn_r   <= (others => '0');
        obs_first_range_r <= (others => '0');
        prev_sample_cnt_r <= (others => '0');
        prev_epoch_valid_r<= '0';

        for i in 0 to G_NUM_CHANNELS - 1 loop
          obs_prn_r(i)      <= (others => '0');
          obs_dopp_r(i)     <= (others => '0');
          obs_range_r(i)    <= (others => '0');
          obs_sat_x_r(i)    <= (others => '0');
          obs_sat_y_r(i)    <= (others => '0');
          obs_sat_z_r(i)    <= (others => '0');
          obs_clk_corr_r(i) <= (others => '0');
          prev_range_r(i)   <= (others => '0');
          prev_valid_r(i)   <= '0';
        end loop;
      else
        obs_valid_r <= '0';

        if obs_en_i = '1' and epoch_tick_i = '1' then
          obs_epoch_r      <= obs_epoch_r + 1;
          obs_valid_mask_r <= (others => '0');
          valid_count_v    := 0;
          first_found_v    := false;

          if prev_epoch_valid_r = '1' then
            dt_s_v := real(to_integer(sample_counter_i - prev_sample_cnt_r)) / real(C_SAMPLE_RATE_HZ);
            if dt_s_v <= 0.0 then
              dt_s_v := 0.001;
            end if;
          else
            dt_s_v := 0.001;
          end if;

          if rx_est_valid_i = '1' then
            lat_rad_v := (real(to_integer(rx_est_lat_e7_i)) / 1.0E7) * MATH_PI / 180.0;
            lon_rad_v := (real(to_integer(rx_est_lon_e7_i)) / 1.0E7) * MATH_PI / 180.0;
            h_m_v := real(to_integer(rx_est_height_mm_i)) / 1000.0;
            sin_lat_v := sin(lat_rad_v);
            cos_lat_v := cos(lat_rad_v);
            n_v := C_WGS84_A / sqrt(1.0 - C_WGS84_E2 * sin_lat_v * sin_lat_v);
            rx_x_m_v := (n_v + h_m_v) * cos_lat_v * cos(lon_rad_v);
            rx_y_m_v := (n_v + h_m_v) * cos_lat_v * sin(lon_rad_v);
            rx_z_m_v := (n_v * (1.0 - C_WGS84_E2) + h_m_v) * sin_lat_v;
          else
            rx_x_m_v := 0.0;
            rx_y_m_v := 0.0;
            rx_z_m_v := 0.0;
          end if;

          iono_m_v := real(G_IONO_CORR_MM) / 1000.0;
          tropo_m_v := real(G_TROPO_CORR_MM) / 1000.0;

          for i in 0 to G_NUM_CHANNELS - 1 loop
            obs_prn_r(i) <= chan_prn_i(i);
            prn_idx_v := to_integer(chan_prn_i(i)) - 1;

            if prn_idx_v >= 0 and prn_idx_v <= 31 then
              sat_x_m_v := real(to_integer(sat_x_ecef_i(prn_idx_v)));
              sat_y_m_v := real(to_integer(sat_y_ecef_i(prn_idx_v)));
              sat_z_m_v := real(to_integer(sat_z_ecef_i(prn_idx_v)));
              sat_clk_m_v := real(to_integer(sat_clk_corr_m_i(prn_idx_v)));
              eph_ok_v := (eph_valid_prn_i(prn_idx_v) = '1');
            else
              sat_x_m_v := 0.0;
              sat_y_m_v := 0.0;
              sat_z_m_v := 0.0;
              sat_clk_m_v := 0.0;
              eph_ok_v := false;
            end if;

            obs_sat_x_r(i) <= to_signed(real_to_i32(sat_x_m_v), 32);
            obs_sat_y_r(i) <= to_signed(real_to_i32(sat_y_m_v), 32);
            obs_sat_z_r(i) <= to_signed(real_to_i32(sat_z_m_v), 32);

            dx_v := sat_x_m_v - rx_x_m_v;
            dy_v := sat_y_m_v - rx_y_m_v;
            dz_v := sat_z_m_v - rx_z_m_v;
            geom_range_m_v := sqrt(dx_v * dx_v + dy_v * dy_v + dz_v * dz_v);
            if geom_range_m_v < 1.0 then
              geom_range_m_v := 1.0;
            end if;

            geom_ms_v := geom_range_m_v / C_M_PER_MS;
            coarse_ms_v := integer(geom_ms_v + 0.5);
            if coarse_ms_v < 1 then
              coarse_ms_v := 1;
            end if;

            code_frac_ms_v := real(to_integer(chan_code_i(i))) / 1023.0;
            tof_raw_m_v := real(coarse_ms_v) * C_M_PER_MS + code_frac_ms_v * C_M_PER_MS;

            sagnac_m_v := (C_OMEGA_E_RADPS / C_LIGHT_MPS) * (sat_x_m_v * rx_y_m_v - sat_y_m_v * rx_x_m_v);
            corr_total_m_v := sat_clk_m_v - sagnac_m_v + iono_m_v + tropo_m_v;
            pr_corr_m_v := tof_raw_m_v - corr_total_m_v;
            if pr_corr_m_v < 0.0 then
              pr_corr_m_v := 0.0;
            end if;

            pr_range_u_v := real_to_u32(pr_corr_m_v);
            obs_range_r(i) <= pr_range_u_v;
            obs_clk_corr_r(i) <= to_signed(real_to_i32(corr_total_m_v), 32);

            rr_dopp_mps_v := -real(to_integer(chan_dopp_i(i))) * C_L1_LAMBDA_M;
            if prev_valid_r(i) = '1' and dt_s_v > 0.0 then
              rr_meas_mps_v := (pr_corr_m_v - real(to_integer(prev_range_r(i)))) / dt_s_v;
            else
              rr_meas_mps_v := rr_dopp_mps_v;
            end if;

            rr_blend_mps_v := 0.7 * rr_dopp_mps_v + 0.3 * rr_meas_mps_v;
            dopp_corr_hz_v := real_to_i32(-rr_blend_mps_v / C_L1_LAMBDA_M);
            obs_dopp_r(i) <= clamp_s16(dopp_corr_hz_v);

            lock_ok_v := (chan_alloc_i(i) = '1') and (chan_code_lock_i(i) = '1');
            range_ok_v := (pr_corr_m_v > 1.0E7) and (pr_corr_m_v < 5.0E7);
            rate_ok_v := abs(rr_blend_mps_v) < 5000.0;

            if G_REQUIRE_EPH_FOR_VALID then
              valid_obs_v := lock_ok_v and eph_ok_v and range_ok_v and rate_ok_v;
            else
              valid_obs_v := lock_ok_v and range_ok_v and rate_ok_v;
            end if;

            if valid_obs_v then
              obs_valid_mask_r(i) <= '1';
              valid_count_v := valid_count_v + 1;
              if not first_found_v then
                first_found_v := true;
                obs_first_prn_r <= chan_prn_i(i);
                obs_first_range_r <= pr_range_u_v;
              end if;
              prev_valid_r(i) <= '1';
            else
              prev_valid_r(i) <= '0';
            end if;
            prev_range_r(i) <= pr_range_u_v;
          end loop;

          obs_count_r <= to_unsigned(valid_count_v, obs_count_r'length);
          if valid_count_v > 0 then
            obs_valid_r <= '1';
          end if;

          prev_sample_cnt_r <= sample_counter_i;
          prev_epoch_valid_r <= '1';
        end if;
      end if;
    end if;
  end process;
end architecture;
