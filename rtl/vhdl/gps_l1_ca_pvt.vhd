library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.gps_l1_ca_pkg.all;

entity gps_l1_ca_pvt is
  generic (
    G_NUM_CHANNELS : integer := 5;
    G_LOG_INTERNAL : boolean := false
  );
  port (
    clk              : in  std_logic;
    rst_n            : in  std_logic;
    pvt_en_i         : in  std_logic;
    obs_valid_i      : in  std_logic;
    obs_count_i      : in  unsigned(7 downto 0);
    obs_valid_mask_i : in  std_logic_vector(G_NUM_CHANNELS - 1 downto 0);
    obs_prn_i        : in  u6_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_dopp_i       : in  s16_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_range_i      : in  u32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_x_i      : in  s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_y_i      : in  s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_sat_z_i      : in  s32_arr_t(0 to G_NUM_CHANNELS - 1);
    obs_clk_corr_i   : in  s32_arr_t(0 to G_NUM_CHANNELS - 1);
    pvt_valid_o      : out std_logic;
    pvt_sats_used_o  : out unsigned(7 downto 0);
    pvt_lat_e7_o     : out signed(31 downto 0);
    pvt_lon_e7_o     : out signed(31 downto 0);
    pvt_height_mm_o  : out signed(31 downto 0);
    pvt_cbias_o      : out signed(31 downto 0)
  );
end entity;

architecture rtl of gps_l1_ca_pvt is
  type real_vec4_t is array (0 to 3) of real;
  type real_mat4_t is array (0 to 3, 0 to 3) of real;

  signal pvt_valid_r     : std_logic := '0';
  signal pvt_sats_used_r : unsigned(7 downto 0) := (others => '0');
  signal pvt_lat_e7_r    : signed(31 downto 0) := (others => '0');
  signal pvt_lon_e7_r    : signed(31 downto 0) := (others => '0');
  signal pvt_height_mm_r : signed(31 downto 0) := (others => '0');
  signal pvt_cbias_r     : signed(31 downto 0) := (others => '0');

  signal rx_x_m_r        : signed(31 downto 0) := (others => '0');
  signal rx_y_m_r        : signed(31 downto 0) := (others => '0');
  signal rx_z_m_r        : signed(31 downto 0) := (others => '0');

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

  function atan2_real(y : real; x : real) return real is
  begin
    if x > 0.0 then
      return arctan(y / x);
    elsif x < 0.0 and y >= 0.0 then
      return arctan(y / x) + MATH_PI;
    elsif x < 0.0 and y < 0.0 then
      return arctan(y / x) - MATH_PI;
    elsif x = 0.0 and y > 0.0 then
      return MATH_PI / 2.0;
    elsif x = 0.0 and y < 0.0 then
      return -MATH_PI / 2.0;
    end if;
    return 0.0;
  end function;

  procedure solve_linear4(
    constant a_in : in real_mat4_t;
    constant b_in : in real_vec4_t;
    variable ok_o : out boolean;
    variable x_o  : out real_vec4_t
  ) is
    variable a      : real_mat4_t;
    variable b      : real_vec4_t;
    variable tmp_r  : real;
    variable pivot  : integer;
    variable maxabs : real;
    variable fac    : real;
  begin
    a := a_in;
    b := b_in;
    ok_o := true;
    x_o := (others => 0.0);

    for k in 0 to 3 loop
      pivot := k;
      maxabs := abs(a(k, k));
      for r in k + 1 to 3 loop
        if abs(a(r, k)) > maxabs then
          maxabs := abs(a(r, k));
          pivot := r;
        end if;
      end loop;

      if maxabs < 1.0E-9 then
        ok_o := false;
        return;
      end if;

      if pivot /= k then
        for c in 0 to 3 loop
          tmp_r := a(k, c);
          a(k, c) := a(pivot, c);
          a(pivot, c) := tmp_r;
        end loop;
        tmp_r := b(k);
        b(k) := b(pivot);
        b(pivot) := tmp_r;
      end if;

      for r in k + 1 to 3 loop
        fac := a(r, k) / a(k, k);
        a(r, k) := 0.0;
        for c in k + 1 to 3 loop
          a(r, c) := a(r, c) - fac * a(k, c);
        end loop;
        b(r) := b(r) - fac * b(k);
      end loop;
    end loop;

    for r in 3 downto 0 loop
      tmp_r := b(r);
      for c in r + 1 to 3 loop
        tmp_r := tmp_r - a(r, c) * x_o(c);
      end loop;
      x_o(r) := tmp_r / a(r, r);
    end loop;
  end procedure;
begin
  pvt_valid_o      <= pvt_valid_r;
  pvt_sats_used_o  <= pvt_sats_used_r;
  pvt_lat_e7_o     <= pvt_lat_e7_r;
  pvt_lon_e7_o     <= pvt_lon_e7_r;
  pvt_height_mm_o  <= pvt_height_mm_r;
  pvt_cbias_o      <= pvt_cbias_r;

  process (clk)
    variable hth         : real_mat4_t;
    variable htv         : real_vec4_t;
    variable dlt         : real_vec4_t;
    variable solve_ok_v  : boolean;
    variable sat_cnt     : integer;

    variable x_est       : real;
    variable y_est       : real;
    variable z_est       : real;
    variable b_est       : real;

    variable sx          : real;
    variable sy          : real;
    variable sz          : real;
    variable pr_m        : real;
    variable dx          : real;
    variable dy          : real;
    variable dz          : real;
    variable rho         : real;
    variable h0          : real;
    variable h1          : real;
    variable h2          : real;
    variable v           : real;

    variable lon_rad     : real;
    variable lat_rad     : real;
    variable p           : real;
    variable n           : real;
    variable h_m         : real;
    variable sin_lat     : real;
    variable lat_deg     : real;
    variable lon_deg     : real;

    constant C_WGS84_A   : real := 6378137.0;
    constant C_WGS84_E2  : real := 6.69437999014E-3;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        pvt_valid_r     <= '0';
        pvt_sats_used_r <= (others => '0');
        pvt_lat_e7_r    <= (others => '0');
        pvt_lon_e7_r    <= (others => '0');
        pvt_height_mm_r <= (others => '0');
        pvt_cbias_r     <= (others => '0');
        rx_x_m_r        <= (others => '0');
        rx_y_m_r        <= (others => '0');
        rx_z_m_r        <= (others => '0');
      else
        if pvt_en_i = '1' and obs_valid_i = '1' and to_integer(obs_count_i) >= 4 then
          x_est := real(to_integer(rx_x_m_r));
          y_est := real(to_integer(rx_y_m_r));
          z_est := real(to_integer(rx_z_m_r));
          b_est := real(to_integer(pvt_cbias_r));

          sat_cnt := 0;
          for i in 0 to G_NUM_CHANNELS - 1 loop
            if obs_valid_mask_i(i) = '1' then
              sat_cnt := sat_cnt + 1;
            end if;
          end loop;

          if sat_cnt >= 4 then
            for iter in 0 to 4 loop
              hth := (others => (others => 0.0));
              htv := (others => 0.0);

              for i in 0 to G_NUM_CHANNELS - 1 loop
                if obs_valid_mask_i(i) = '1' then
                  sx := real(to_integer(obs_sat_x_i(i)));
                  sy := real(to_integer(obs_sat_y_i(i)));
                  sz := real(to_integer(obs_sat_z_i(i)));
                  pr_m := real(to_integer(obs_range_i(i))) + real(to_integer(obs_clk_corr_i(i)));

                  dx := x_est - sx;
                  dy := y_est - sy;
                  dz := z_est - sz;
                  rho := sqrt(dx * dx + dy * dy + dz * dz);
                  if rho < 1.0 then
                    rho := 1.0;
                  end if;

                  h0 := -(dx / rho);
                  h1 := -(dy / rho);
                  h2 := -(dz / rho);
                  v := pr_m - (rho + b_est);

                  hth(0, 0) := hth(0, 0) + h0 * h0;
                  hth(0, 1) := hth(0, 1) + h0 * h1;
                  hth(0, 2) := hth(0, 2) + h0 * h2;
                  hth(0, 3) := hth(0, 3) + h0;
                  hth(1, 1) := hth(1, 1) + h1 * h1;
                  hth(1, 2) := hth(1, 2) + h1 * h2;
                  hth(1, 3) := hth(1, 3) + h1;
                  hth(2, 2) := hth(2, 2) + h2 * h2;
                  hth(2, 3) := hth(2, 3) + h2;
                  hth(3, 3) := hth(3, 3) + 1.0;

                  htv(0) := htv(0) + h0 * v;
                  htv(1) := htv(1) + h1 * v;
                  htv(2) := htv(2) + h2 * v;
                  htv(3) := htv(3) + v;
                end if;
              end loop;

              -- Symmetry + light damping for numerical stability.
              hth(1, 0) := hth(0, 1);
              hth(2, 0) := hth(0, 2);
              hth(3, 0) := hth(0, 3);
              hth(2, 1) := hth(1, 2);
              hth(3, 1) := hth(1, 3);
              hth(3, 2) := hth(2, 3);
              hth(0, 0) := hth(0, 0) + 1.0E-3;
              hth(1, 1) := hth(1, 1) + 1.0E-3;
              hth(2, 2) := hth(2, 2) + 1.0E-3;
              hth(3, 3) := hth(3, 3) + 1.0E-3;

              solve_linear4(hth, htv, solve_ok_v, dlt);
              if solve_ok_v then
                x_est := x_est + dlt(0);
                y_est := y_est + dlt(1);
                z_est := z_est + dlt(2);
                b_est := b_est + dlt(3);
              end if;
            end loop;

            lon_rad := atan2_real(y_est, x_est);
            p := sqrt(x_est * x_est + y_est * y_est);
            if p < 1.0 then
              p := 1.0;
            end if;

            lat_rad := atan2_real(z_est, p * (1.0 - C_WGS84_E2));
            for k in 0 to 4 loop
              sin_lat := sin(lat_rad);
              n := C_WGS84_A / sqrt(1.0 - C_WGS84_E2 * sin_lat * sin_lat);
              h_m := p / cos(lat_rad) - n;
              lat_rad := atan2_real(z_est, p * (1.0 - C_WGS84_E2 * n / (n + h_m)));
            end loop;

            sin_lat := sin(lat_rad);
            n := C_WGS84_A / sqrt(1.0 - C_WGS84_E2 * sin_lat * sin_lat);
            h_m := p / cos(lat_rad) - n;

            lat_deg := lat_rad * 180.0 / MATH_PI;
            lon_deg := lon_rad * 180.0 / MATH_PI;

            rx_x_m_r <= to_signed(real_to_i32(x_est), 32);
            rx_y_m_r <= to_signed(real_to_i32(y_est), 32);
            rx_z_m_r <= to_signed(real_to_i32(z_est), 32);

            pvt_lat_e7_r <= to_signed(real_to_i32(lat_deg * 1.0E7), 32);
            pvt_lon_e7_r <= to_signed(real_to_i32(lon_deg * 1.0E7), 32);
            pvt_height_mm_r <= to_signed(real_to_i32(h_m * 1000.0), 32);
            pvt_cbias_r <= to_signed(real_to_i32(b_est), 32);
            pvt_sats_used_r <= to_unsigned(sat_cnt, pvt_sats_used_r'length);
            pvt_valid_r <= '1';

            -- pragma translate_off
            if G_LOG_INTERNAL then
              report "PVT LS updated: sats=" & integer'image(sat_cnt) &
                     " lat_e7=" & integer'image(to_integer(pvt_lat_e7_r)) &
                     " lon_e7=" & integer'image(to_integer(pvt_lon_e7_r)) &
                     " h_mm=" & integer'image(to_integer(pvt_height_mm_r)) &
                     " cbias_m=" & integer'image(real_to_i32(b_est));
            end if;
            -- pragma translate_on
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
