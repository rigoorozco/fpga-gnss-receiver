library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_nco_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_chan_bank_tb is
  generic (
    G_USE_FILE_INPUT      : boolean := false;
    G_INPUT_FILE          : string  := "2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat";
    G_FILE_SAMPLE_RATE_SPS: integer := 4000000;
    G_DUT_SAMPLE_RATE_SPS : integer := 2000000;
    G_MAX_FILE_SAMPLES    : integer := 3000000
  );
end entity;

architecture tb of gps_l1_ca_chan_bank_tb is
  constant C_CLK_PERIOD : time := 2 ns;
  constant C_NUM_CH     : integer := 2;
  constant C_SIGNAL_AMP : integer := 12000;

  signal clk                : std_logic := '0';
  signal rst_n              : std_logic := '0';
  signal core_en            : std_logic := '0';
  signal tracking_en        : std_logic := '0';
  signal chan_enable_mask   : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '1');
  signal s_valid            : std_logic := '0';
  signal s_i                : signed(15 downto 0) := (others => '0');
  signal s_q                : signed(15 downto 0) := (others => '0');
  signal min_cn0_dbhz_i     : unsigned(7 downto 0) := to_unsigned(20, 8);
  signal carrier_lock_th_i  : signed(15 downto 0) := to_signed(16384, 16);
  signal max_lock_fail_i    : unsigned(7 downto 0) := to_unsigned(20, 8);
  signal dopp_step_pullin_i : unsigned(15 downto 0) := to_unsigned(80, 16);
  signal dopp_step_lock_i   : unsigned(15 downto 0) := to_unsigned(20, 16);
  signal assign_valid_i     : std_logic := '0';
  signal assign_ch_idx_i    : unsigned(7 downto 0) := (others => '0');
  signal assign_prn_i       : unsigned(5 downto 0) := (others => '0');
  signal assign_dopp_i      : signed(15 downto 0) := (others => '0');
  signal assign_code_i      : unsigned(10 downto 0) := (others => '0');

  signal chan_alloc_o       : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_state_o       : track_state_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_lock_o   : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_carrier_lock_o: std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_report_valid_o: std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_prn_o         : u6_arr_t(0 to C_NUM_CH - 1);
  signal chan_dopp_o        : s16_arr_t(0 to C_NUM_CH - 1);
  signal chan_code_o        : u11_arr_t(0 to C_NUM_CH - 1);
  signal chan_cn0_dbhz_o    : u8_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_i_o    : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_prompt_q_o    : s24_arr_t(0 to C_NUM_CH - 1);
  signal chan_nav_valid_o   : std_logic_vector(C_NUM_CH - 1 downto 0);
  signal chan_nav_bit_o     : std_logic_vector(C_NUM_CH - 1 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_chan_bank
    generic map (
      G_NUM_CHANNELS => C_NUM_CH
    )
    port map (
      clk                => clk,
      rst_n              => rst_n,
      core_en            => core_en,
      tracking_en        => tracking_en,
      chan_enable_mask   => chan_enable_mask,
      s_valid            => s_valid,
      s_i                => s_i,
      s_q                => s_q,
      min_cn0_dbhz_i     => min_cn0_dbhz_i,
      carrier_lock_th_i  => carrier_lock_th_i,
      max_lock_fail_i    => max_lock_fail_i,
      dopp_step_pullin_i => dopp_step_pullin_i,
      dopp_step_lock_i   => dopp_step_lock_i,
      assign_valid_i     => assign_valid_i,
      assign_ch_idx_i    => assign_ch_idx_i,
      assign_prn_i       => assign_prn_i,
      assign_dopp_i      => assign_dopp_i,
      assign_code_i      => assign_code_i,
      chan_alloc_o       => chan_alloc_o,
      chan_state_o       => chan_state_o,
      chan_code_lock_o   => chan_code_lock_o,
      chan_carrier_lock_o=> chan_carrier_lock_o,
      chan_report_valid_o=> chan_report_valid_o,
      chan_prn_o         => chan_prn_o,
      chan_dopp_o        => chan_dopp_o,
      chan_code_o        => chan_code_o,
      chan_cn0_dbhz_o    => chan_cn0_dbhz_o,
      chan_prompt_i_o    => chan_prompt_i_o,
      chan_prompt_q_o    => chan_prompt_q_o,
      chan_nav_valid_o   => chan_nav_valid_o,
      chan_nav_bit_o     => chan_nav_bit_o
    );

  stim_proc : process
    type prn_seq_t is array (0 to 1022) of std_logic;

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

    procedure drive_tracking_ms(
      prn_seq_v     : in prn_seq_t;
      dopp_hz_v     : in integer;
      carr_phase_v  : inout signed(31 downto 0);
      code_nco_v    : inout unsigned(31 downto 0);
      chip_idx_v    : inout integer
    ) is
      variable sig_sign_i  : integer;
      variable phase_idx_i : integer;
      variable lo_i_i      : integer;
      variable lo_sin_i    : integer;
      variable sig_i_i     : integer;
      variable sig_q_i     : integer;
      variable next_code_v : unsigned(31 downto 0);
      variable carr_fcw_v  : signed(31 downto 0);
    begin
      carr_fcw_v := carr_fcw_from_hz(dopp_hz_v);
      for s in 0 to C_SAMPLES_PER_MS - 1 loop
        if prn_seq_v(chip_idx_v) = '1' then
          sig_sign_i := -C_SIGNAL_AMP;
        else
          sig_sign_i := C_SIGNAL_AMP;
        end if;

        phase_idx_i := to_integer(unsigned(carr_phase_v(31 downto 22)));
        lo_i_i := to_integer(lo_cos_q15(to_unsigned(phase_idx_i, 10)));
        lo_sin_i := to_integer(lo_sin_q15(to_unsigned(phase_idx_i, 10)));

        sig_i_i := (sig_sign_i * lo_i_i) / 32768;
        sig_q_i := (sig_sign_i * lo_sin_i) / 32768;

        s_valid <= '1';
        s_i <= to_signed(sig_i_i, s_i'length);
        s_q <= to_signed(sig_q_i, s_q'length);
        wait until rising_edge(clk);

        carr_phase_v := carr_phase_v + carr_fcw_v;
        next_code_v := code_nco_v + C_CODE_NCO_FCW;
        if next_code_v < code_nco_v then
          if chip_idx_v = 1022 then
            chip_idx_v := 0;
          else
            chip_idx_v := chip_idx_v + 1;
          end if;
        end if;
        code_nco_v := next_code_v;
      end loop;
    end procedure;

    procedure drive_tracking_sample(
      prn_seq_v     : in prn_seq_t;
      dopp_hz_v     : in integer;
      carr_phase_v  : inout signed(31 downto 0);
      code_nco_v    : inout unsigned(31 downto 0);
      chip_idx_v    : inout integer
    ) is
      variable sig_sign_i  : integer;
      variable phase_idx_i : integer;
      variable lo_i_i      : integer;
      variable lo_sin_i    : integer;
      variable sig_i_i     : integer;
      variable sig_q_i     : integer;
      variable next_code_v : unsigned(31 downto 0);
      variable carr_fcw_v  : signed(31 downto 0);
    begin
      carr_fcw_v := carr_fcw_from_hz(dopp_hz_v);
      if prn_seq_v(chip_idx_v) = '1' then
        sig_sign_i := -C_SIGNAL_AMP;
      else
        sig_sign_i := C_SIGNAL_AMP;
      end if;

      phase_idx_i := to_integer(unsigned(carr_phase_v(31 downto 22)));
      lo_i_i := to_integer(lo_cos_q15(to_unsigned(phase_idx_i, 10)));
      lo_sin_i := to_integer(lo_sin_q15(to_unsigned(phase_idx_i, 10)));

      sig_i_i := (sig_sign_i * lo_i_i) / 32768;
      sig_q_i := (sig_sign_i * lo_sin_i) / 32768;

      s_valid <= '1';
      s_i <= to_signed(sig_i_i, s_i'length);
      s_q <= to_signed(sig_q_i, s_q'length);
      wait until rising_edge(clk);
      s_valid <= '0';

      carr_phase_v := carr_phase_v + carr_fcw_v;
      next_code_v := code_nco_v + C_CODE_NCO_FCW;
      if next_code_v < code_nco_v then
        if chip_idx_v = 1022 then
          chip_idx_v := 0;
        else
          chip_idx_v := chip_idx_v + 1;
        end if;
      end if;
      code_nco_v := next_code_v;
    end procedure;

    procedure assign_and_wait_lock(
      ch_idx_v     : in integer;
      prn_v        : in integer;
      dopp_v       : in integer;
      code_v       : in integer;
      max_ms_v     : in integer;
      case_name    : in string
    ) is
      variable prn_seq_v      : prn_seq_t;
      variable carr_phase_v   : signed(31 downto 0) := (others => '0');
      variable code_nco_v     : unsigned(31 downto 0) := (others => '0');
      variable chip_idx_v     : integer := 0;
      variable lock_seen_v    : boolean := false;
      variable mask_v         : std_logic_vector(C_NUM_CH - 1 downto 0) := (others => '0');
      variable code_init_v    : integer;
      variable read_status_v  : file_open_status;
      variable b0_v           : character;
      variable b1_v           : character;
      variable b2_v           : character;
      variable b3_v           : character;
      variable in_file_cnt_v  : integer := 0;
      variable out_samp_cnt_v : integer := 0;
      variable decim_v        : integer := 1;
      variable lock_hold_samp_v : integer := 0;
      variable lock_hold_done_v : boolean := false;
      variable synth_max_samples_v : integer := 0;
    begin
      code_init_v := code_v mod 1023;
      if code_init_v < 0 then
        code_init_v := code_init_v + 1023;
      end if;
      chip_idx_v := code_init_v;
      code_nco_v := shift_left(to_unsigned(code_init_v, code_nco_v'length), 21);
      prn_seq_v := build_prn_sequence(prn_v);

      mask_v(ch_idx_v) := '1';
      chan_enable_mask <= mask_v;

      assign_ch_idx_i <= to_unsigned(ch_idx_v, assign_ch_idx_i'length);
      assign_prn_i <= to_unsigned(prn_v, assign_prn_i'length);
      assign_dopp_i <= to_signed(dopp_v, assign_dopp_i'length);
      assign_code_i <= to_unsigned(code_init_v, assign_code_i'length);
      assign_valid_i <= '1';
      wait until rising_edge(clk);
      assign_valid_i <= '0';

      for i in 0 to 1 loop
        wait until rising_edge(clk);
      end loop;

      assert chan_alloc_o(ch_idx_v) = '1'
        report case_name & ": expected channel allocated."
        severity failure;
      assert to_integer(chan_prn_o(ch_idx_v)) = prn_v
        report case_name & ": expected PRN retune."
        severity failure;

      if G_USE_FILE_INPUT then
        decim_v := G_FILE_SAMPLE_RATE_SPS / G_DUT_SAMPLE_RATE_SPS;
        in_file_cnt_v := 0;
        out_samp_cnt_v := 0;
        file_open(read_status_v, iq_file, G_INPUT_FILE, read_mode);
        assert read_status_v = open_ok
          report case_name & ": unable to open input file: " & G_INPUT_FILE
          severity failure;

        while not endfile(iq_file) loop
          exit when lock_hold_done_v;
          if G_MAX_FILE_SAMPLES > 0 and out_samp_cnt_v >= G_MAX_FILE_SAMPLES then
            exit;
          end if;

          if endfile(iq_file) then exit; end if;
          read(iq_file, b0_v);
          if endfile(iq_file) then exit; end if;
          read(iq_file, b1_v);
          if endfile(iq_file) then exit; end if;
          read(iq_file, b2_v);
          if endfile(iq_file) then exit; end if;
          read(iq_file, b3_v);

          if (in_file_cnt_v mod decim_v) = 0 then
            drive_file_sample(s16_from_le(b0_v, b1_v), s16_from_le(b2_v, b3_v));
            out_samp_cnt_v := out_samp_cnt_v + 1;
            if chan_code_lock_o(ch_idx_v) = '1' and
               chan_carrier_lock_o(ch_idx_v) = '1' and
               chan_state_o(ch_idx_v) = TRACK_LOCKED then
              lock_seen_v := true;
              lock_hold_samp_v := lock_hold_samp_v + 1;
              if lock_hold_samp_v >= C_SAMPLES_PER_MS then
                lock_hold_done_v := true;
              end if;
            else
              lock_hold_samp_v := 0;
            end if;
          end if;
          in_file_cnt_v := in_file_cnt_v + 1;
        end loop;
        file_close(iq_file);
        log_msg(case_name & " file replay done. input_samples=" &
                integer'image(in_file_cnt_v) &
                ", injected_samples=" & integer'image(out_samp_cnt_v));

        if not lock_hold_done_v then
          for i in 0 to 5000 loop
            drive_file_sample(to_signed(0, 16), to_signed(0, 16));
            if chan_code_lock_o(ch_idx_v) = '1' and
               chan_carrier_lock_o(ch_idx_v) = '1' and
               chan_state_o(ch_idx_v) = TRACK_LOCKED then
              lock_seen_v := true;
              lock_hold_samp_v := lock_hold_samp_v + 1;
              if lock_hold_samp_v >= C_SAMPLES_PER_MS then
                lock_hold_done_v := true;
                exit;
              end if;
            else
              lock_hold_samp_v := 0;
            end if;
          end loop;
        end if;
      else
        synth_max_samples_v := max_ms_v * C_SAMPLES_PER_MS;
        for s in 0 to synth_max_samples_v - 1 loop
          drive_tracking_sample(prn_seq_v, dopp_v, carr_phase_v, code_nco_v, chip_idx_v);
          if chan_code_lock_o(ch_idx_v) = '1' and
             chan_carrier_lock_o(ch_idx_v) = '1' and
             chan_state_o(ch_idx_v) = TRACK_LOCKED then
            lock_seen_v := true;
            lock_hold_samp_v := lock_hold_samp_v + 1;
            if lock_hold_samp_v >= C_SAMPLES_PER_MS then
              lock_hold_done_v := true;
              exit;
            end if;
          else
            lock_hold_samp_v := 0;
          end if;
        end loop;
      end if;

      if lock_seen_v then
        log_msg(case_name & ": lock seen");
      else
        log_msg(case_name & ": lock not seen");
      end if;
      if lock_hold_done_v then
        log_msg(case_name & ": lock held for 1 ms");
      elsif lock_seen_v then
        log_msg(case_name & ": lock dropped before 1 ms hold");
      end if;

      assert lock_seen_v
        report case_name & ": did not reach TRACK_LOCKED with code+carrier lock."
        severity failure;
      assert lock_hold_done_v
        report case_name & ": did not keep lock for 1 ms."
        severity failure;
      assert to_integer(chan_cn0_dbhz_o(ch_idx_v)) >= to_integer(min_cn0_dbhz_i)
        report case_name & ": expected C/N0 estimate above configured minimum."
        severity failure;
    end procedure;
  begin
    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    core_en <= '1';
    tracking_en <= '1';
    if G_USE_FILE_INPUT then
      assert G_FILE_SAMPLE_RATE_SPS mod G_DUT_SAMPLE_RATE_SPS = 0
        report "File sample rate must be integer multiple of DUT sample rate."
        severity failure;
    end if;
    min_cn0_dbhz_i <= to_unsigned(0, min_cn0_dbhz_i'length);
    carrier_lock_th_i <= to_signed(0, carrier_lock_th_i'length);
    max_lock_fail_i <= to_unsigned(4, max_lock_fail_i'length);

    for i in 0 to 4 loop
      wait until rising_edge(clk);
    end loop;

    assert chan_alloc_o = "00" report "Expected all channels deallocated after reset." severity failure;

    -- Verify tracking/lock using acquisition-derived {PRN, code, Doppler}.
    assign_and_wait_lock(0, 1,  -1000, 848,  8, "Run4 result -> ch0");
    assign_and_wait_lock(0, 20, -1000, 320,  8, "Run5 result -> ch0");
    assign_and_wait_lock(0, 32, -1500, 1008, 8, "Run6 result -> ch0");
    assign_and_wait_lock(0, 17, -1500, 992,  8, "Run7 result -> ch0");
    assign_and_wait_lock(0, 11, 750,   928,  8, "Run8 result -> ch0");

    log_msg("gps_l1_ca_chan_bank_tb completed");
    s_valid <= '0';
    stop;
    wait;
  end process;
end architecture;
