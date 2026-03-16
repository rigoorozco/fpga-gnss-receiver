library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

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
  type data24_arr_t is array (0 to G_NUM_CHANNELS - 1, 0 to 9) of std_logic_vector(23 downto 0);
  type bitcnt_arr_t is array (0 to G_NUM_CHANNELS - 1) of integer range 0 to 29;
  type wordidx_arr_t is array (0 to G_NUM_CHANNELS - 1) of integer range 0 to 9;
  type smallcnt_arr_t is array (0 to G_NUM_CHANNELS - 1) of integer range 0 to 7;
  type sl_ch_arr_t is array (0 to G_NUM_CHANNELS - 1) of std_logic;

  type sfmask_arr_t is array (0 to 31) of std_logic_vector(2 downto 0);
  type u8_arr32_t is array (0 to 31) of unsigned(7 downto 0);
  type u10_arr32_t is array (0 to 31) of unsigned(9 downto 0);
  type u16_arr32_t is array (0 to 31) of unsigned(15 downto 0);
  type u32_arr32_t is array (0 to 31) of unsigned(31 downto 0);

  signal eph_valid_prn_r   : std_logic_vector(31 downto 0) := (others => '0');
  signal nav_word_count_r  : unsigned(31 downto 0) := (others => '0');
  signal tow_seconds_r     : unsigned(31 downto 0) := (others => '0');
  signal sat_x_ecef_r      : s32_arr_t(0 to 31);
  signal sat_y_ecef_r      : s32_arr_t(0 to 31);
  signal sat_z_ecef_r      : s32_arr_t(0 to 31);
  signal sat_clk_corr_m_r  : s32_arr_t(0 to 31);

  signal word_shift_r      : word30_arr_t;
  signal bit_count_r       : bitcnt_arr_t;
  signal word_sync_r       : sl_ch_arr_t;
  signal prev_d29_r        : sl_ch_arr_t;
  signal prev_d30_r        : sl_ch_arr_t;
  signal word_idx_r        : wordidx_arr_t;
  signal parity_fail_r     : smallcnt_arr_t;
  signal chan_prn_prev_r   : u6_arr_t(0 to G_NUM_CHANNELS - 1);
  signal subframe_word_r   : data24_arr_t;

  signal sf_mask_r         : sfmask_arr_t;
  signal iode_sf2_r        : u8_arr32_t;
  signal iode_sf3_r        : u8_arr32_t;
  signal iodc_r            : u10_arr32_t;
  signal week_r            : u16_arr32_t;
  signal toc_s_r           : u16_arr32_t;
  signal toe_s_r           : u16_arr32_t;
  signal tgd_r             : s32_arr_t(0 to 31);

  signal af0_r             : s32_arr_t(0 to 31);
  signal af1_r             : s32_arr_t(0 to 31);
  signal af2_r             : s32_arr_t(0 to 31);

  signal crs_r             : s32_arr_t(0 to 31);
  signal dn_r              : s32_arr_t(0 to 31);
  signal m0_r              : s32_arr_t(0 to 31);
  signal cuc_r             : s32_arr_t(0 to 31);
  signal e_u_r             : u32_arr32_t;
  signal cus_r             : s32_arr_t(0 to 31);
  signal sqrt_a_u_r        : u32_arr32_t;

  signal cic_r             : s32_arr_t(0 to 31);
  signal omega0_r          : s32_arr_t(0 to 31);
  signal cis_r             : s32_arr_t(0 to 31);
  signal i0_r              : s32_arr_t(0 to 31);
  signal crc_r             : s32_arr_t(0 to 31);
  signal omega_r           : s32_arr_t(0 to 31);
  signal omegadot_r        : s32_arr_t(0 to 31);
  signal idot_r            : s32_arr_t(0 to 31);
  -- pragma translate_off
  function hex_nibble(n : integer) return character is
  begin
    case n is
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

  function slv24_to_hex(x : std_logic_vector(23 downto 0)) return string is
    variable s : string(1 to 6);
    variable n : integer;
  begin
    for i in 0 to 5 loop
      n := to_integer(unsigned(x(23 - i * 4 downto 20 - i * 4)));
      s(i + 1) := hex_nibble(n);
    end loop;
    return s;
  end function;
  -- pragma translate_on

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

  function u32_to_real(x : unsigned(31 downto 0)) return real is
    variable lo : integer;
  begin
    lo := to_integer(x(30 downto 0));
    if x(31) = '1' then
      return real(lo) + 2147483648.0;
    end if;
    return real(lo);
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

  function nav_data24(word_raw : std_logic_vector(29 downto 0); prev_d30 : std_logic)
    return std_logic_vector is
    variable data_v : std_logic_vector(23 downto 0);
  begin
    data_v := word_raw(29 downto 6);
    if prev_d30 = '1' then
      data_v := not data_v;
    end if;
    return data_v;
  end function;

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

  function nav_parity_ok(
    word_raw  : std_logic_vector(29 downto 0);
    prev_d29  : std_logic;
    prev_d30  : std_logic
  ) return boolean is
    variable data_v : std_logic_vector(23 downto 0);
  begin
    data_v := nav_data24(word_raw, prev_d30);
    return nav_parity_bits(data_v, prev_d29, prev_d30) = word_raw(5 downto 0);
  end function;

  procedure propagate_sat_state(
    constant tow_s_i      : in integer;
    constant toc_s_i      : in unsigned(15 downto 0);
    constant toe_s_i      : in unsigned(15 downto 0);
    constant af0_i        : in signed(31 downto 0);
    constant af1_i        : in signed(31 downto 0);
    constant af2_i        : in signed(31 downto 0);
    constant tgd_i        : in signed(31 downto 0);
    constant crs_i        : in signed(31 downto 0);
    constant dn_i         : in signed(31 downto 0);
    constant m0_i         : in signed(31 downto 0);
    constant cuc_i        : in signed(31 downto 0);
    constant e_u_i        : in unsigned(31 downto 0);
    constant cus_i        : in signed(31 downto 0);
    constant sqrt_a_u_i   : in unsigned(31 downto 0);
    constant cic_i        : in signed(31 downto 0);
    constant omega0_i     : in signed(31 downto 0);
    constant cis_i        : in signed(31 downto 0);
    constant i0_i         : in signed(31 downto 0);
    constant crc_i        : in signed(31 downto 0);
    constant omega_i      : in signed(31 downto 0);
    constant omegadot_i   : in signed(31 downto 0);
    constant idot_i       : in signed(31 downto 0);
    variable valid_o      : out boolean;
    variable x_m_o        : out integer;
    variable y_m_o        : out integer;
    variable z_m_o        : out integer;
    variable clk_corr_m_o : out integer
  ) is
    constant C_MU         : real := 3.986005E14;
    constant C_OMEGA_E    : real := 7.2921151467E-5;
    constant C_C          : real := 299792458.0;
    constant C_F_REL      : real := -4.442807633E-10;

    variable toe_s        : real;
    variable toc_s        : real;
    variable tk           : real;
    variable tc           : real;
    variable sqrt_a       : real;
    variable a            : real;
    variable e            : real;
    variable n0           : real;
    variable dn           : real;
    variable n            : real;
    variable m0           : real;
    variable m            : real;
    variable ecc_anom     : real;
    variable sin_e        : real;
    variable cos_e        : real;
    variable nu           : real;
    variable phi          : real;
    variable du           : real;
    variable dr           : real;
    variable di           : real;
    variable u            : real;
    variable r            : real;
    variable iang         : real;
    variable x_orb        : real;
    variable y_orb        : real;
    variable omega_asc    : real;
    variable x_ecef       : real;
    variable y_ecef       : real;
    variable z_ecef       : real;
    variable dtr          : real;
    variable clk_sec      : real;

    variable af0          : real;
    variable af1          : real;
    variable af2          : real;
    variable tgd          : real;
    variable crs          : real;
    variable cuc          : real;
    variable cus          : real;
    variable cic          : real;
    variable cis          : real;
    variable crc          : real;
    variable omega0       : real;
    variable omega_w      : real;
    variable i0           : real;
    variable omegadot     : real;
    variable idot         : real;
  begin
    valid_o := false;
    x_m_o := 0;
    y_m_o := 0;
    z_m_o := 0;
    clk_corr_m_o := 0;

    sqrt_a := u32_to_real(sqrt_a_u_i) * (2.0 ** (-19));
    e := u32_to_real(e_u_i) * (2.0 ** (-33));
    if sqrt_a <= 1.0 or e < 0.0 or e >= 1.0 then
      return;
    end if;

    toe_s := real(to_integer(toe_s_i)) * 16.0;
    toc_s := real(to_integer(toc_s_i)) * 16.0;

    af0 := real(to_integer(af0_i)) * (2.0 ** (-31));
    af1 := real(to_integer(af1_i)) * (2.0 ** (-43));
    af2 := real(to_integer(af2_i)) * (2.0 ** (-55));
    tgd := real(to_integer(tgd_i)) * (2.0 ** (-31));

    crs := real(to_integer(crs_i)) * (2.0 ** (-5));
    crc := real(to_integer(crc_i)) * (2.0 ** (-5));
    dn := real(to_integer(dn_i)) * (2.0 ** (-43)) * MATH_PI;
    m0 := real(to_integer(m0_i)) * (2.0 ** (-31)) * MATH_PI;
    cuc := real(to_integer(cuc_i)) * (2.0 ** (-29));
    cus := real(to_integer(cus_i)) * (2.0 ** (-29));
    cic := real(to_integer(cic_i)) * (2.0 ** (-29));
    cis := real(to_integer(cis_i)) * (2.0 ** (-29));
    omega0 := real(to_integer(omega0_i)) * (2.0 ** (-31)) * MATH_PI;
    omega_w := real(to_integer(omega_i)) * (2.0 ** (-31)) * MATH_PI;
    i0 := real(to_integer(i0_i)) * (2.0 ** (-31)) * MATH_PI;
    omegadot := real(to_integer(omegadot_i)) * (2.0 ** (-43)) * MATH_PI;
    idot := real(to_integer(idot_i)) * (2.0 ** (-43)) * MATH_PI;

    a := sqrt_a * sqrt_a;
    if a < 1.0 then
      return;
    end if;

    tk := real(tow_s_i) - toe_s;
    if tk > 302400.0 then
      tk := tk - 604800.0;
    elsif tk < -302400.0 then
      tk := tk + 604800.0;
    end if;

    tc := real(tow_s_i) - toc_s;
    if tc > 302400.0 then
      tc := tc - 604800.0;
    elsif tc < -302400.0 then
      tc := tc + 604800.0;
    end if;

    n0 := sqrt(C_MU / (a * a * a));
    n := n0 + dn;
    m := m0 + n * tk;

    ecc_anom := m;
    for iter in 0 to 7 loop
      ecc_anom := m + e * sin(ecc_anom);
    end loop;

    sin_e := sin(ecc_anom);
    cos_e := cos(ecc_anom);
    nu := atan2_real(sqrt(1.0 - e * e) * sin_e, cos_e - e);
    phi := nu + omega_w;

    du := cuc * cos(2.0 * phi) + cus * sin(2.0 * phi);
    dr := crc * cos(2.0 * phi) + crs * sin(2.0 * phi);
    di := cic * cos(2.0 * phi) + cis * sin(2.0 * phi);

    u := phi + du;
    r := a * (1.0 - e * cos_e) + dr;
    iang := i0 + idot * tk + di;

    x_orb := r * cos(u);
    y_orb := r * sin(u);

    omega_asc := omega0 + (omegadot - C_OMEGA_E) * tk - C_OMEGA_E * toe_s;

    x_ecef := x_orb * cos(omega_asc) - y_orb * cos(iang) * sin(omega_asc);
    y_ecef := x_orb * sin(omega_asc) + y_orb * cos(iang) * cos(omega_asc);
    z_ecef := y_orb * sin(iang);

    dtr := C_F_REL * e * sqrt_a * sin_e;
    clk_sec := af0 + af1 * tc + af2 * tc * tc + dtr - tgd;

    x_m_o := real_to_i32(x_ecef);
    y_m_o := real_to_i32(y_ecef);
    z_m_o := real_to_i32(z_ecef);
    clk_corr_m_o := real_to_i32(clk_sec * C_C);
    valid_o := true;
  end procedure;
begin
  eph_valid_prn_o  <= eph_valid_prn_r;
  nav_word_count_o <= nav_word_count_r;
  tow_seconds_o    <= tow_seconds_r;
  sat_x_ecef_o     <= sat_x_ecef_r;
  sat_y_ecef_o     <= sat_y_ecef_r;
  sat_z_ecef_o     <= sat_z_ecef_r;
  sat_clk_corr_m_o <= sat_clk_corr_m_r;

  process (clk)
    variable raw_word_v        : std_logic_vector(29 downto 0);
    variable data24_v          : std_logic_vector(23 downto 0);
    variable data24_cand_v     : std_logic_vector(23 downto 0);
    variable cand_found_v      : boolean;
    variable cand_d29_v        : std_logic;
    variable cand_d30_v        : std_logic;
    variable d29_sl_v          : std_logic;
    variable d30_sl_v          : std_logic;

    variable prn_idx_v         : integer;
    variable sf_id_v           : integer;
    variable tow_z_v           : integer;
    variable tow_sec_v         : integer;
    variable tow_update_v      : boolean;

    variable sf_mask_v         : std_logic_vector(2 downto 0);
    variable iode2_i           : integer;
    variable iode3_i           : integer;
    variable eph_now_valid_v   : std_logic;

    variable sat_ok_v          : boolean;
    variable sat_x_v           : integer;
    variable sat_y_v           : integer;
    variable sat_z_v           : integer;
    variable sat_clk_v         : integer;

    variable w1               : std_logic_vector(23 downto 0);
    variable w2               : std_logic_vector(23 downto 0);
    variable w3               : std_logic_vector(23 downto 0);
    variable w4               : std_logic_vector(23 downto 0);
    variable w5               : std_logic_vector(23 downto 0);
    variable w6               : std_logic_vector(23 downto 0);
    variable w7               : std_logic_vector(23 downto 0);
    variable w8               : std_logic_vector(23 downto 0);
    variable w9               : std_logic_vector(23 downto 0);
    variable w10              : std_logic_vector(23 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        eph_valid_prn_r  <= (others => '0');
        nav_word_count_r <= (others => '0');
        tow_seconds_r    <= (others => '0');

        for p in 0 to 31 loop
          -- Hold-last-valid fallback seed until true ephemeris decode is complete.
          sat_x_ecef_r(p) <= to_signed((-13000000) + p * 800000, 32);
          sat_y_ecef_r(p) <= to_signed(10000000 - p * 600000, 32);
          sat_z_ecef_r(p) <= to_signed((-9000000) + p * 550000, 32);
          sat_clk_corr_m_r(p) <= (others => '0');

          sf_mask_r(p) <= (others => '0');
          iode_sf2_r(p) <= (others => '0');
          iode_sf3_r(p) <= (others => '0');
          iodc_r(p) <= (others => '0');
          week_r(p) <= (others => '0');
          toc_s_r(p) <= (others => '0');
          toe_s_r(p) <= (others => '0');

          af0_r(p) <= (others => '0');
          af1_r(p) <= (others => '0');
          af2_r(p) <= (others => '0');
          tgd_r(p) <= (others => '0');

          crs_r(p) <= (others => '0');
          dn_r(p) <= (others => '0');
          m0_r(p) <= (others => '0');
          cuc_r(p) <= (others => '0');
          e_u_r(p) <= (others => '0');
          cus_r(p) <= (others => '0');
          sqrt_a_u_r(p) <= (others => '0');

          cic_r(p) <= (others => '0');
          omega0_r(p) <= (others => '0');
          cis_r(p) <= (others => '0');
          i0_r(p) <= (others => '0');
          crc_r(p) <= (others => '0');
          omega_r(p) <= (others => '0');
          omegadot_r(p) <= (others => '0');
          idot_r(p) <= (others => '0');
        end loop;

        for i in 0 to G_NUM_CHANNELS - 1 loop
          word_shift_r(i) <= (others => '0');
          bit_count_r(i) <= 0;
          word_sync_r(i) <= '0';
          prev_d29_r(i) <= '0';
          prev_d30_r(i) <= '0';
          word_idx_r(i) <= 0;
          parity_fail_r(i) <= 0;
          chan_prn_prev_r(i) <= (others => '0');
          for w in 0 to 9 loop
            subframe_word_r(i, w) <= (others => '0');
          end loop;
        end loop;
      elsif nav_en_i = '1' then
        tow_update_v := false;
        tow_sec_v := to_integer(tow_seconds_r);

        for i in 0 to G_NUM_CHANNELS - 1 loop
          if chan_prn_i(i) /= chan_prn_prev_r(i) then
            chan_prn_prev_r(i) <= chan_prn_i(i);
            word_shift_r(i) <= (others => '0');
            bit_count_r(i) <= 0;
            word_sync_r(i) <= '0';
            prev_d29_r(i) <= '0';
            prev_d30_r(i) <= '0';
            word_idx_r(i) <= 0;
            parity_fail_r(i) <= 0;
          end if;

          if chan_nav_valid_i(i) = '1' then
            raw_word_v := word_shift_r(i)(28 downto 0) & chan_nav_bit_i(i);
            word_shift_r(i) <= raw_word_v;
            prn_idx_v := to_integer(chan_prn_i(i)) - 1;

            if word_sync_r(i) = '0' then
              cand_found_v := false;
              cand_d29_v := '0';
              cand_d30_v := '0';
              data24_cand_v := (others => '0');

              for d29 in 0 to 1 loop
                for d30 in 0 to 1 loop
                  if not cand_found_v then
                    if d29 = 0 then
                      d29_sl_v := '0';
                    else
                      d29_sl_v := '1';
                    end if;
                    if d30 = 0 then
                      d30_sl_v := '0';
                    else
                      d30_sl_v := '1';
                    end if;
                    if nav_parity_ok(
                      raw_word_v,
                      d29_sl_v,
                      d30_sl_v
                    ) then
                      data24_v := nav_data24(raw_word_v, d30_sl_v);
                      if data24_v(23 downto 16) = "10001011" then
                        cand_found_v := true;
                        cand_d29_v := d29_sl_v;
                        cand_d30_v := d30_sl_v;
                        data24_cand_v := data24_v;
                      end if;
                    end if;
                  end if;
                end loop;
              end loop;

              if cand_found_v and prn_idx_v >= 0 and prn_idx_v <= 31 then
                word_sync_r(i) <= '1';
                bit_count_r(i) <= 0;
                word_idx_r(i) <= 1;
                parity_fail_r(i) <= 0;
                prev_d29_r(i) <= raw_word_v(1);
                prev_d30_r(i) <= raw_word_v(0);
                subframe_word_r(i, 0) <= data24_cand_v;
                nav_word_count_r <= nav_word_count_r + 1;
              end if;
            else
              if bit_count_r(i) = 29 then
                bit_count_r(i) <= 0;

                if nav_parity_ok(raw_word_v, prev_d29_r(i), prev_d30_r(i)) then
                  data24_v := nav_data24(raw_word_v, prev_d30_r(i));
                  nav_word_count_r <= nav_word_count_r + 1;
                  prev_d29_r(i) <= raw_word_v(1);
                  prev_d30_r(i) <= raw_word_v(0);
                  parity_fail_r(i) <= 0;

                  if word_idx_r(i) = 0 and data24_v(23 downto 16) /= "10001011" then
                    word_sync_r(i) <= '0';
                    word_idx_r(i) <= 0;
                  else
                    subframe_word_r(i, word_idx_r(i)) <= data24_v;

                    if word_idx_r(i) = 9 and prn_idx_v >= 0 and prn_idx_v <= 31 then
                      w1 := subframe_word_r(i, 0);
                      w2 := subframe_word_r(i, 1);
                      w3 := subframe_word_r(i, 2);
                      w4 := subframe_word_r(i, 3);
                      w5 := subframe_word_r(i, 4);
                      w6 := subframe_word_r(i, 5);
                      w7 := subframe_word_r(i, 6);
                      w8 := subframe_word_r(i, 7);
                      w9 := subframe_word_r(i, 8);
                      w10 := data24_v;

                      sf_id_v := to_integer(unsigned(w2(4 downto 2)));
                      tow_z_v := to_integer(unsigned(w2(23 downto 7)));
                      tow_sec_v := (tow_z_v * 6) mod 604800;
                      tow_update_v := true;
                      -- pragma translate_off
                      log_msg("NAV",
                              "New GPS NAV message received in channel " & integer'image(i) &
                              ": subframe " & integer'image(sf_id_v) &
                              " from satellite GPS PRN " & integer'image(prn_idx_v + 1) &
                              " (TOW=" & integer'image(tow_sec_v) & "s)");
                      log_msg("NAV",
                              "NAV decoded: PRN=" & integer'image(prn_idx_v + 1) &
                              " SF=" & integer'image(sf_id_v) &
                              " TOW=" & integer'image(tow_sec_v) & "s " &
                              "W1=0x" & slv24_to_hex(w1) & " " &
                              "W2=0x" & slv24_to_hex(w2) & " " &
                              "W3=0x" & slv24_to_hex(w3) & " " &
                              "W4=0x" & slv24_to_hex(w4) & " " &
                              "W5=0x" & slv24_to_hex(w5) & " " &
                              "W6=0x" & slv24_to_hex(w6) & " " &
                              "W7=0x" & slv24_to_hex(w7) & " " &
                              "W8=0x" & slv24_to_hex(w8) & " " &
                              "W9=0x" & slv24_to_hex(w9) & " " &
                              "W10=0x" & slv24_to_hex(w10));
                      -- pragma translate_on

                      sf_mask_v := sf_mask_r(prn_idx_v);

                      case sf_id_v is
                        when 1 =>
                          sf_mask_v(0) := '1';
                          sf_mask_r(prn_idx_v) <= sf_mask_v;

                          week_r(prn_idx_v) <= resize(unsigned(w3(23 downto 14)), 16);
                          iodc_r(prn_idx_v) <= unsigned(std_logic_vector'(w3(1 downto 0) & w8(23 downto 16)));
                          toc_s_r(prn_idx_v) <= unsigned(w8(15 downto 0));

                          af2_r(prn_idx_v) <= resize(signed(w9(23 downto 16)), 32);
                          af1_r(prn_idx_v) <= resize(signed(w9(15 downto 0)), 32);
                          af0_r(prn_idx_v) <= resize(signed(w10(23 downto 2)), 32);
                          tgd_r(prn_idx_v) <= resize(signed(w7(7 downto 0)), 32);

                        when 2 =>
                          sf_mask_v(1) := '1';
                          sf_mask_r(prn_idx_v) <= sf_mask_v;

                          iode_sf2_r(prn_idx_v) <= unsigned(w3(23 downto 16));
                          crs_r(prn_idx_v) <= resize(signed(w3(15 downto 0)), 32);
                          dn_r(prn_idx_v) <= resize(signed(w4(23 downto 8)), 32);
                          m0_r(prn_idx_v) <= signed(std_logic_vector'(w4(7 downto 0) & w5));
                          cuc_r(prn_idx_v) <= resize(signed(w6(23 downto 8)), 32);
                          e_u_r(prn_idx_v) <= unsigned(std_logic_vector'(w6(7 downto 0) & w7));
                          cus_r(prn_idx_v) <= resize(signed(w8(23 downto 8)), 32);
                          sqrt_a_u_r(prn_idx_v) <= unsigned(std_logic_vector'(w8(7 downto 0) & w9));
                          toe_s_r(prn_idx_v) <= unsigned(w10(23 downto 8));

                        when 3 =>
                          sf_mask_v(2) := '1';
                          sf_mask_r(prn_idx_v) <= sf_mask_v;

                          cic_r(prn_idx_v) <= resize(signed(w3(23 downto 8)), 32);
                          omega0_r(prn_idx_v) <= signed(std_logic_vector'(w3(7 downto 0) & w4));
                          cis_r(prn_idx_v) <= resize(signed(w5(23 downto 8)), 32);
                          i0_r(prn_idx_v) <= signed(std_logic_vector'(w5(7 downto 0) & w6));
                          crc_r(prn_idx_v) <= resize(signed(w7(23 downto 8)), 32);
                          omega_r(prn_idx_v) <= signed(std_logic_vector'(w7(7 downto 0) & w8));
                          omegadot_r(prn_idx_v) <= resize(signed(w9), 32);
                          iode_sf3_r(prn_idx_v) <= unsigned(w10(23 downto 16));
                          idot_r(prn_idx_v) <= resize(signed(w10(15 downto 2)), 32);

                        when others =>
                          null;
                      end case;

                      iode2_i := to_integer(iode_sf2_r(prn_idx_v));
                      iode3_i := to_integer(iode_sf3_r(prn_idx_v));
                      if sf_id_v = 2 then
                        iode2_i := to_integer(unsigned(w3(23 downto 16)));
                      elsif sf_id_v = 3 then
                        iode3_i := to_integer(unsigned(w10(23 downto 16)));
                      end if;

                      if sf_mask_v = "111" and iode2_i = iode3_i and iode2_i /= 0 then
                        eph_now_valid_v := '1';
                      else
                        eph_now_valid_v := eph_valid_prn_r(prn_idx_v);
                      end if;
                      eph_valid_prn_r(prn_idx_v) <= eph_now_valid_v;
                    end if;

                    if word_idx_r(i) = 9 then
                      word_idx_r(i) <= 0;
                    else
                      word_idx_r(i) <= word_idx_r(i) + 1;
                    end if;
                  end if;
                else
                  if parity_fail_r(i) >= 2 then
                    word_sync_r(i) <= '0';
                    word_idx_r(i) <= 0;
                    bit_count_r(i) <= 0;
                    parity_fail_r(i) <= 0;
                  else
                    parity_fail_r(i) <= parity_fail_r(i) + 1;
                  end if;
                end if;
              else
                bit_count_r(i) <= bit_count_r(i) + 1;
              end if;
            end if;
          end if;
        end loop;

        if tow_update_v then
          tow_seconds_r <= to_unsigned(tow_sec_v, tow_seconds_r'length);
          for p in 0 to 31 loop
            if eph_valid_prn_r(p) = '1' then
              propagate_sat_state(
                tow_sec_v,
                toc_s_r(p),
                toe_s_r(p),
                af0_r(p),
                af1_r(p),
                af2_r(p),
                tgd_r(p),
                crs_r(p),
                dn_r(p),
                m0_r(p),
                cuc_r(p),
                e_u_r(p),
                cus_r(p),
                sqrt_a_u_r(p),
                cic_r(p),
                omega0_r(p),
                cis_r(p),
                i0_r(p),
                crc_r(p),
                omega_r(p),
                omegadot_r(p),
                idot_r(p),
                sat_ok_v,
                sat_x_v,
                sat_y_v,
                sat_z_v,
                sat_clk_v
              );

              if sat_ok_v then
                sat_x_ecef_r(p) <= to_signed(sat_x_v, 32);
                sat_y_ecef_r(p) <= to_signed(sat_y_v, 32);
                sat_z_ecef_r(p) <= to_signed(sat_z_v, 32);
                sat_clk_corr_m_r(p) <= to_signed(sat_clk_v, 32);
              end if;
            end if;
          end loop;
        end if;
      end if;
    end if;
  end process;
end architecture;
