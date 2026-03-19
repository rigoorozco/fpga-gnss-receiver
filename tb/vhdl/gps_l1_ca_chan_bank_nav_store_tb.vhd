library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_nco_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_chan_bank_nav_store_tb is
  generic (
    G_RUN_MS              : integer := 40;
    G_USE_FILE_INPUT      : boolean := true;
    G_INPUT_FILE          : string  := "2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN_2msps.dat";
    G_FILE_SAMPLE_RATE_SPS: integer := 2000000;
    G_DUT_SAMPLE_RATE_SPS : integer := 2000000
  );
end entity;

architecture tb of gps_l1_ca_chan_bank_nav_store_tb is
  constant C_CLK_PERIOD      : time := 2 ns;
  constant C_NUM_CH          : integer := 4;
  constant C_SIGNAL_AMP_EACH : integer := 7000;
  constant C_HOLD_MS         : integer := 5;

  type int_arr_t is array (natural range <>) of integer;

  constant C_PRNS  : int_arr_t(0 to C_NUM_CH - 1) := (1, 32, 17, 11);
  constant C_DOPPS : int_arr_t(0 to C_NUM_CH - 1) := (-1000, -1500, -1500, 750);
  constant C_CODES : int_arr_t(0 to C_NUM_CH - 1) := (848, 1008, 992, 928);

  signal clk                : std_logic := '0';
  signal rst_n              : std_logic := '0';
  signal core_en            : std_logic := '0';
  signal tracking_en        : std_logic := '0';
  signal chan_enable_mask   : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '1');
  signal s_valid            : std_logic := '0';
  signal s_i                : signed(15 downto 0) := (others => '0');
  signal s_q                : signed(15 downto 0) := (others => '0');
  signal min_cn0_dbhz_i     : unsigned(7 downto 0) := to_unsigned(0, 8);
  signal carrier_lock_th_i  : signed(15 downto 0) := to_signed(0, 16);
  signal max_lock_fail_i    : unsigned(7 downto 0) := to_unsigned(4, 8);
  signal dopp_step_pullin_i : unsigned(15 downto 0) := to_unsigned(80, 16);
  signal dopp_step_lock_i   : unsigned(15 downto 0) := to_unsigned(20, 16);
  signal pll_bw_hz_i        : unsigned(15 downto 0) := to_unsigned(8960, 16);
  signal dll_bw_hz_i        : unsigned(15 downto 0) := to_unsigned(512, 16);
  signal pll_bw_narrow_hz_i : unsigned(15 downto 0) := to_unsigned(1280, 16);
  signal dll_bw_narrow_hz_i : unsigned(15 downto 0) := to_unsigned(128, 16);
  signal fll_bw_hz_i        : unsigned(15 downto 0) := to_unsigned(2560, 16);
  signal assign_valid_i     : std_logic := '0';
  signal assign_ch_idx_i    : unsigned(7 downto 0) := (others => '0');
  signal assign_prn_i       : unsigned(5 downto 0) := (others => '0');
  signal assign_dopp_i      : signed(15 downto 0) := (others => '0');
  signal assign_code_i      : unsigned(10 downto 0) := (others => '0');

  signal chan_alloc_o        : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_state_o        : track_state_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_lock_o    : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_carrier_lock_o : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_report_valid_o : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_prn_o          : u6_arr_t(0 to C_NUM_CH - 1);
  signal chan_dopp_o         : s16_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_o         : u11_arr_t(0 to C_NUM_CH - 1);
  signal chan_cn0_dbhz_o     : u8_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_i_o     : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_q_o     : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_nav_valid_o    : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_nav_bit_o      : std_logic_vector(C_NUM_CH - 1 downto 0);

  signal eph_valid_prn_o     : std_logic_vector(31 downto 0);
  signal nav_word_count_o    : unsigned(31 downto 0);
  signal tow_seconds_o       : unsigned(31 downto 0);
  signal sat_x_ecef_o        : s32_arr_t(0 to 31);
  signal sat_y_ecef_o        : s32_arr_t(0 to 31);
  signal sat_z_ecef_o        : s32_arr_t(0 to 31);
  signal sat_clk_corr_m_o    : s32_arr_t(0 to 31);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_chan_bank
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk                 => clk,
      rst_n               => rst_n,
      core_en             => core_en,
      tracking_en         => tracking_en,
      chan_enable_mask    => chan_enable_mask,
      s_valid             => s_valid,
      s_i                 => s_i,
      s_q                 => s_q,
      min_cn0_dbhz_i      => min_cn0_dbhz_i,
      carrier_lock_th_i   => carrier_lock_th_i,
      max_lock_fail_i     => max_lock_fail_i,
      dopp_step_pullin_i  => dopp_step_pullin_i,
      dopp_step_lock_i    => dopp_step_lock_i,
      pll_bw_hz_i         => pll_bw_hz_i,
      dll_bw_hz_i         => dll_bw_hz_i,
      pll_bw_narrow_hz_i  => pll_bw_narrow_hz_i,
      dll_bw_narrow_hz_i  => dll_bw_narrow_hz_i,
      fll_bw_hz_i         => fll_bw_hz_i,
      assign_valid_i      => assign_valid_i,
      assign_ch_idx_i     => assign_ch_idx_i,
      assign_prn_i        => assign_prn_i,
      assign_dopp_i       => assign_dopp_i,
      assign_code_i       => assign_code_i,
      chan_alloc_o        => chan_alloc_o,
      chan_state_o        => chan_state_o,
      chan_code_lock_o    => chan_code_lock_o,
      chan_carrier_lock_o => chan_carrier_lock_o,
      chan_report_valid_o => chan_report_valid_o,
      chan_prn_o          => chan_prn_o,
      chan_dopp_o         => chan_dopp_o,
      chan_code_o         => chan_code_o,
      chan_cn0_dbhz_o     => chan_cn0_dbhz_o,
      chan_prompt_i_o     => chan_prompt_i_o,
      chan_prompt_q_o     => chan_prompt_q_o,
      chan_nav_valid_o    => chan_nav_valid_o,
      chan_nav_bit_o      => chan_nav_bit_o
    );

  nav_store_u : entity work.gps_l1_ca_nav_store
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk              => clk,
      rst_n            => rst_n,
      nav_en_i         => tracking_en,
      chan_nav_valid_i => chan_nav_valid_o,
      chan_nav_bit_i   => chan_nav_bit_o,
      chan_prn_i       => chan_prn_o,
      eph_valid_prn_o  => eph_valid_prn_o,
      nav_word_count_o => nav_word_count_o,
      tow_seconds_o    => tow_seconds_o,
      sat_x_ecef_o     => sat_x_ecef_o,
      sat_y_ecef_o     => sat_y_ecef_o,
      sat_z_ecef_o     => sat_z_ecef_o,
      sat_clk_corr_m_o => sat_clk_corr_m_o
    );

  stim_proc : process
    type prn_seq_t is array (0 to 1022) of std_logic;
    type prn_seq_arr_t is array (0 to C_NUM_CH - 1) of prn_seq_t;

    function carr_fcw_from_hz(dopp_hz : integer) return signed is
    begin
      return to_signed(dopp_hz * 2147, 32);
    end function;

    type iq_file_t is file of character;
    file iq_file : iq_file_t;

    function s16_from_le(lo_b : character; hi_b : character) return signed is
      variable v_u16 : integer;
      variable v_s16 : integer;
    begin
      v_u16 := character'pos(lo_b) + 256 * character'pos(hi_b);
      if v_u16 >= 32768 then
        v_s16 := v_u16 - 65536;
      else
        v_s16 := v_u16;
      end if;
      return to_signed(v_s16, 16);
    end function;

    procedure drive_file_sample(i_v : in signed(15 downto 0); q_v : in signed(15 downto 0)) is
    begin
      s_valid <= '1';
      s_i <= i_v;
      s_q <= q_v;
      wait until rising_edge(clk);
      s_valid <= '0';
    end procedure;

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

    function g2_tap_a(prn_i : integer) return integer is
    begin
      case prn_i is
        when 1  => return 2;
        when 2  => return 3;
        when 3  => return 4;
        when 4  => return 5;
        when 5  => return 1;
        when 6  => return 2;
        when 7  => return 1;
        when 8  => return 2;
        when 9  => return 3;
        when 10 => return 2;
        when 11 => return 3;
        when 12 => return 5;
        when 13 => return 6;
        when 14 => return 7;
        when 15 => return 8;
        when 16 => return 9;
        when 17 => return 1;
        when 18 => return 2;
        when 19 => return 3;
        when 20 => return 4;
        when 21 => return 5;
        when 22 => return 6;
        when 23 => return 1;
        when 24 => return 4;
        when 25 => return 5;
        when 26 => return 6;
        when 27 => return 7;
        when 28 => return 8;
        when 29 => return 1;
        when 30 => return 2;
        when 31 => return 3;
        when others => return 4;
      end case;
    end function;

    function g2_tap_b(prn_i : integer) return integer is
    begin
      case prn_i is
        when 1  => return 6;
        when 2  => return 7;
        when 3  => return 8;
        when 4  => return 9;
        when 5  => return 9;
        when 6  => return 10;
        when 7  => return 8;
        when 8  => return 9;
        when 9  => return 10;
        when 10 => return 3;
        when 11 => return 4;
        when 12 => return 6;
        when 13 => return 7;
        when 14 => return 8;
        when 15 => return 9;
        when 16 => return 10;
        when 17 => return 4;
        when 18 => return 5;
        when 19 => return 6;
        when 20 => return 7;
        when 21 => return 8;
        when 22 => return 9;
        when 23 => return 3;
        when 24 => return 6;
        when 25 => return 7;
        when 26 => return 8;
        when 27 => return 9;
        when 28 => return 10;
        when 29 => return 6;
        when 30 => return 7;
        when 31 => return 8;
        when others => return 9;
      end case;
    end function;

    function build_prn_sequence(prn_i : integer) return prn_seq_t is
      variable g1 : std_logic_vector(9 downto 0);
      variable g2 : std_logic_vector(9 downto 0);
      variable g1_out : std_logic;
      variable g2_out : std_logic;
      variable fb1 : std_logic;
      variable fb2 : std_logic;
      variable ta  : integer;
      variable tb  : integer;
      variable seq_v : prn_seq_t;
    begin
      g1 := (others => '1');
      g2 := (others => '1');
      ta := g2_tap_a(prn_i);
      tb := g2_tap_b(prn_i);

      for chip in 0 to 1022 loop
        g1_out := g1(9);
        g2_out := g2(10 - ta) xor g2(10 - tb);
        seq_v(chip) := g1_out xor g2_out;

        fb1 := g1(2) xor g1(9);
        fb2 := g2(1) xor g2(2) xor g2(5) xor g2(7) xor g2(8) xor g2(9);

        g1 := g1(8 downto 0) & fb1;
        g2 := g2(8 downto 0) & fb2;
      end loop;
      return seq_v;
    end function;

    procedure assign_channel(
      ch_idx_v : in integer;
      prn_v    : in integer;
      dopp_v   : in integer;
      code_v   : in integer
    ) is
    begin
      assign_ch_idx_i <= to_unsigned(ch_idx_v, assign_ch_idx_i'length);
      assign_prn_i <= to_unsigned(prn_v, assign_prn_i'length);
      assign_dopp_i <= to_signed(dopp_v, assign_dopp_i'length);
      assign_code_i <= to_unsigned(code_v mod 1023, assign_code_i'length);
      assign_valid_i <= '1';
      wait until rising_edge(clk);
      assign_valid_i <= '0';
      wait until rising_edge(clk);
    end procedure;

    procedure log_nav_store_summary is
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

      function slv32_to_hex(x : std_logic_vector(31 downto 0)) return string is
        variable s : string(1 to 8);
        variable n : integer;
      begin
        for i in 0 to 7 loop
          n := to_integer(unsigned(x(31 - i * 4 downto 28 - i * 4)));
          s(i + 1) := hex_nibble(n);
        end loop;
        return s;
      end function;

      variable prn_idx_v : integer;
    begin
      log_msg(
        "nav_store summary: nav_word_count=" & integer'image(to_integer(nav_word_count_o)) &
        ", tow_seconds=" & integer'image(to_integer(tow_seconds_o)) &
        ", eph_valid_prn=0x" & slv32_to_hex(eph_valid_prn_o)
      );

      for ch in 0 to C_NUM_CH - 1 loop
        prn_idx_v := C_PRNS(ch) - 1;
        log_msg(
          "nav_store PRN" & integer'image(C_PRNS(ch)) &
          ": sat_x=" & integer'image(to_integer(sat_x_ecef_o(prn_idx_v))) &
          ", sat_y=" & integer'image(to_integer(sat_y_ecef_o(prn_idx_v))) &
          ", sat_z=" & integer'image(to_integer(sat_z_ecef_o(prn_idx_v))) &
          ", sat_clk_corr_m=" & integer'image(to_integer(sat_clk_corr_m_o(prn_idx_v)))
        );
      end loop;
    end procedure;

    variable prn_seq_v       : prn_seq_arr_t;
    variable carr_phase_v    : s32_arr_t(0 to C_NUM_CH - 1);
    variable code_nco_v      : u32_arr_t(0 to C_NUM_CH - 1);
    variable chip_idx_v      : int_arr_t(0 to C_NUM_CH - 1);
    variable sig_i_acc_v     : integer;
    variable sig_q_acc_v     : integer;
    variable sig_sign_i      : integer;
    variable phase_idx_i     : integer;
    variable lo_i_i          : integer;
    variable lo_q_i          : integer;
    variable next_code_v     : unsigned(31 downto 0);
    variable carr_fcw_v      : signed(31 downto 0);
    variable all_locked_v    : boolean;
    variable lock_hold_samp_v: integer := 0;
    variable lock_seen_v     : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
    variable read_status_v   : file_open_status;
    variable b0_v            : character;
    variable b1_v            : character;
    variable b2_v            : character;
    variable b3_v            : character;
    variable in_file_cnt_v   : integer := 0;
    variable out_samp_cnt_v  : integer := 0;
    variable run_samples_v   : integer := 0;
  begin
    for ch in 0 to C_NUM_CH - 1 loop
      prn_seq_v(ch) := build_prn_sequence(C_PRNS(ch));
      carr_phase_v(ch) := (others => '0');
      chip_idx_v(ch) := C_CODES(ch) mod 1023;
      code_nco_v(ch) := shift_left(to_unsigned(chip_idx_v(ch), 32), 21);
    end loop;

    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    core_en <= '1';
    tracking_en <= '1';

    for i in 0 to 4 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "0000" report "Expected all channels deallocated after reset." severity failure;

    for ch in 0 to C_NUM_CH - 1 loop
      assign_channel(ch, C_PRNS(ch), C_DOPPS(ch), C_CODES(ch));
    end loop;

    assert chan_alloc_o = "1111" report "Expected all channels allocated after assignment." severity failure;
    run_samples_v := G_RUN_MS * C_SAMPLES_PER_MS;
    assert run_samples_v > 0 report "G_RUN_MS must be positive." severity failure;

    if G_USE_FILE_INPUT then
      assert G_FILE_SAMPLE_RATE_SPS = G_DUT_SAMPLE_RATE_SPS
        report "File sample rate must equal DUT sample rate. Pre-decimate input file before replay."
        severity failure;
      file_open(read_status_v, iq_file, G_INPUT_FILE, read_mode);
      assert read_status_v = open_ok
        report "Unable to open input file: " & G_INPUT_FILE
        severity failure;

      while not endfile(iq_file) loop
        exit when out_samp_cnt_v >= run_samples_v;

        if endfile(iq_file) then exit; end if;
        read(iq_file, b0_v);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b1_v);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b2_v);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b3_v);

        drive_file_sample(s16_from_le(b0_v, b1_v), s16_from_le(b2_v, b3_v));
        out_samp_cnt_v := out_samp_cnt_v + 1;

        all_locked_v := true;
        for ch in 0 to C_NUM_CH - 1 loop
          if chan_code_lock_o(ch) = '1' and chan_carrier_lock_o(ch) = '1' and chan_state_o(ch) = TRACK_LOCKED then
            lock_seen_v(ch) := '1';
          end if;
          if not (chan_code_lock_o(ch) = '1' and chan_carrier_lock_o(ch) = '1' and chan_state_o(ch) = TRACK_LOCKED) then
            all_locked_v := false;
          end if;
        end loop;

        if all_locked_v then
          lock_hold_samp_v := lock_hold_samp_v + 1;
        else
          lock_hold_samp_v := 0;
        end if;
        in_file_cnt_v := in_file_cnt_v + 1;
      end loop;
      file_close(iq_file);
      log_msg(
        "file replay done. input_samples=" & integer'image(in_file_cnt_v) &
        ", injected_samples=" & integer'image(out_samp_cnt_v)
      );
    else
      for sample_idx in 0 to run_samples_v - 1 loop
        sig_i_acc_v := 0;
        sig_q_acc_v := 0;

        for ch in 0 to C_NUM_CH - 1 loop
          if prn_seq_v(ch)(chip_idx_v(ch)) = '1' then
            sig_sign_i := -C_SIGNAL_AMP_EACH;
          else
            sig_sign_i := C_SIGNAL_AMP_EACH;
          end if;

          phase_idx_i := to_integer(unsigned(carr_phase_v(ch)(31 downto 22)));
          lo_i_i := to_integer(lo_cos_q15(to_unsigned(phase_idx_i, 10)));
          lo_q_i := to_integer(lo_sin_q15(to_unsigned(phase_idx_i, 10)));

          sig_i_acc_v := sig_i_acc_v + ((sig_sign_i * lo_i_i) / 32768);
          sig_q_acc_v := sig_q_acc_v + ((sig_sign_i * lo_q_i) / 32768);
        end loop;

        s_valid <= '1';
        s_i <= clamp_s16(sig_i_acc_v);
        s_q <= clamp_s16(sig_q_acc_v);
        wait until rising_edge(clk);
        s_valid <= '0';

        for ch in 0 to C_NUM_CH - 1 loop
          carr_fcw_v := carr_fcw_from_hz(C_DOPPS(ch));
          carr_phase_v(ch) := carr_phase_v(ch) + carr_fcw_v;
          next_code_v := code_nco_v(ch) + C_CODE_NCO_FCW;
          if next_code_v < code_nco_v(ch) then
            if chip_idx_v(ch) = 1022 then
              chip_idx_v(ch) := 0;
            else
              chip_idx_v(ch) := chip_idx_v(ch) + 1;
            end if;
          end if;
          code_nco_v(ch) := next_code_v;
        end loop;

        all_locked_v := true;
        for ch in 0 to C_NUM_CH - 1 loop
          if chan_code_lock_o(ch) = '1' and chan_carrier_lock_o(ch) = '1' and chan_state_o(ch) = TRACK_LOCKED then
            lock_seen_v(ch) := '1';
          end if;
          if not (chan_code_lock_o(ch) = '1' and chan_carrier_lock_o(ch) = '1' and chan_state_o(ch) = TRACK_LOCKED) then
            all_locked_v := false;
          end if;
        end loop;

        if all_locked_v then
          lock_hold_samp_v := lock_hold_samp_v + 1;
        else
          lock_hold_samp_v := 0;
        end if;
      end loop;
    end if;

    s_valid <= '0';

    for ch in 0 to C_NUM_CH - 1 loop
      assert to_integer(chan_prn_o(ch)) = C_PRNS(ch)
        report "Unexpected PRN on channel " & integer'image(ch)
        severity failure;
      assert lock_seen_v(ch) = '1'
        report "Expected lock to be seen on channel " & integer'image(ch)
        severity failure;
    end loop;

    if lock_hold_samp_v >= (C_HOLD_MS * C_SAMPLES_PER_MS) then
      log_msg("all four channels held lock together for " & integer'image(C_HOLD_MS) & " ms");
    else
      log_msg("all four channels did not hold lock together for " & integer'image(C_HOLD_MS) & " ms");
    end if;

    log_nav_store_summary;
    log_msg("gps_l1_ca_chan_bank_nav_store_tb completed");
    stop;
    wait;
  end process;

  chan_report_proc : process (clk)
    variable lock_logged_v : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');

    function state_name(x : track_state_t) return string is
    begin
      case x is
        when TRACK_IDLE   => return "IDLE";
        when TRACK_PULLIN => return "PULLIN";
        when TRACK_LOCKED => return "LOCKED";
      end case;
    end function;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        lock_logged_v := (others => '0');
      else
        for ch in 0 to C_NUM_CH - 1 loop
          if chan_report_valid_o(ch) = '1' and
             chan_code_lock_o(ch) = '1' and
             chan_carrier_lock_o(ch) = '1' and
             chan_state_o(ch) = TRACK_LOCKED and
             lock_logged_v(ch) = '0' then
            log_msg(
              "channel " & integer'image(ch) &
              " locked: prn=" & integer'image(to_integer(chan_prn_o(ch))) &
              ", dopp=" & integer'image(to_integer(chan_dopp_o(ch))) &
              ", code=" & integer'image(to_integer(chan_code_o(ch))) &
              ", cn0_dbhz=" & integer'image(to_integer(chan_cn0_dbhz_o(ch))) &
              ", state=" & state_name(chan_state_o(ch))
            );
            lock_logged_v(ch) := '1';
          end if;
        end loop;
      end if;
    end if;
  end process;

  nav_store_report_proc : process (clk)
    variable prev_nav_word_count_v : unsigned(31 downto 0) := (others => '0');
    variable prev_tow_seconds_v    : unsigned(31 downto 0) := (others => '0');
    variable prev_eph_valid_v      : std_logic_vector(31 downto 0) := (others => '0');
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        prev_nav_word_count_v := (others => '0');
        prev_tow_seconds_v := (others => '0');
        prev_eph_valid_v := (others => '0');
      elsif nav_word_count_o /= prev_nav_word_count_v or
            tow_seconds_o /= prev_tow_seconds_v or
            eph_valid_prn_o /= prev_eph_valid_v then
        log_msg(
          "nav_store update: nav_word_count=" & integer'image(to_integer(nav_word_count_o)) &
          ", tow_seconds=" & integer'image(to_integer(tow_seconds_o))
        );
        prev_nav_word_count_v := nav_word_count_o;
        prev_tow_seconds_v := tow_seconds_o;
        prev_eph_valid_v := eph_valid_prn_o;
      end if;
    end if;
  end process;
end architecture;
