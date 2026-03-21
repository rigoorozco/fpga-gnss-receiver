library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gps_l1_ca_pkg.all;
use work.gps_l1_ca_log_pkg.all;

entity gps_l1_ca_acq_tb is
  generic (
    G_USE_FILE_INPUT      : boolean := false;
    G_INPUT_FILE          : string  := "2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN_2msps.dat";
    G_FILE_SAMPLE_RATE_SPS: integer := 2000000;
    G_DUT_SAMPLE_RATE_SPS : integer := 2000000;
    G_MAX_FILE_SAMPLES    : integer := 0;
    G_DUT_ACQ_IMPL_FFT    : boolean := false
  );
end entity;

architecture tb of gps_l1_ca_acq_tb is
  constant C_CLK_PERIOD : time := 10 ns;

  function trim_int_img(v : integer) return string is
    constant raw : string := integer'image(v);
  begin
    if raw(raw'low) = ' ' then
      return raw(raw'low + 1 to raw'high);
    end if;
    return raw;
  end function;

  function lpad(s_in : string; width : natural) return string is
    variable out_s : string(1 to width);
    variable start : natural;
  begin
    if s_in'length >= width then
      return s_in(s_in'length - width + 1 to s_in'length);
    end if;

    out_s := (others => ' ');
    start := width - s_in'length + 1;
    out_s(start to width) := s_in;
    return out_s;
  end function;

  function pow10(n : natural) return integer is
    variable p : integer := 1;
  begin
    for i in 1 to n loop
      p := p * 10;
    end loop;
    return p;
  end function;

  function fmt_fixed(value_scaled : integer; frac_digits : natural) return string is
    variable abs_v    : integer;
    variable int_part : integer;
    variable frac_part: integer;
    variable scale_v  : integer;
    variable frac_s   : string(1 to frac_digits);
    variable tmp      : integer;
    constant digit0_i : integer := character'pos('0');
  begin
    scale_v := pow10(frac_digits);
    if value_scaled < 0 then
      abs_v := -value_scaled;
    else
      abs_v := value_scaled;
    end if;

    int_part := abs_v / scale_v;
    frac_part := abs_v mod scale_v;

    tmp := frac_part;
    for idx in frac_digits downto 1 loop
      frac_s(idx) := character'val(digit0_i + (tmp mod 10));
      tmp := tmp / 10;
    end loop;

    if value_scaled < 0 then
      return "-" & trim_int_img(int_part) & "." & frac_s;
    end if;
    return trim_int_img(int_part) & "." & frac_s;
  end function;

  signal clk            : std_logic := '0';
  signal rst_n          : std_logic := '0';
  signal core_en        : std_logic := '0';
  signal start_pulse    : std_logic := '0';
  signal prn_start      : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal prn_stop       : unsigned(5 downto 0) := to_unsigned(1, 6);
  signal doppler_min    : signed(15 downto 0) := to_signed(-10000, 16);
  signal doppler_max    : signed(15 downto 0) := to_signed(10000, 16);
  signal doppler_step   : signed(15 downto 0) := to_signed(250, 16);
  signal detect_thresh  : unsigned(31 downto 0) := (others => '0');
  signal coh_ms_i       : unsigned(7 downto 0) := to_unsigned(1, 8);
  signal noncoh_dwells_i: unsigned(7 downto 0) := to_unsigned(1, 8);
  signal doppler_bins_i : unsigned(7 downto 0) := to_unsigned(1, 8);
  signal code_bins_i    : unsigned(10 downto 0) := to_unsigned(1, 11);
  signal code_step_i    : unsigned(10 downto 0) := to_unsigned(1, 11);
  signal s_valid        : std_logic := '0';
  signal s_i            : signed(15 downto 0) := (others => '0');
  signal s_q            : signed(15 downto 0) := (others => '0');

  signal acq_done       : std_logic;
  signal acq_success    : std_logic;
  signal result_valid   : std_logic;
  signal result_prn     : unsigned(5 downto 0);
  signal result_dopp    : signed(15 downto 0);
  signal result_code    : unsigned(10 downto 0);
  signal result_metric  : unsigned(31 downto 0);
begin
  clk <= not clk after C_CLK_PERIOD / 2;

  dut : entity work.gps_l1_ca_acq
    generic map (
      G_DWELL_MS     => 1,
      G_ACQ_IMPL_FFT => G_DUT_ACQ_IMPL_FFT
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      core_en       => core_en,
      start_pulse   => start_pulse,
      prn_start     => prn_start,
      prn_stop      => prn_stop,
      doppler_min   => doppler_min,
      doppler_max   => doppler_max,
      doppler_step  => doppler_step,
      detect_thresh => detect_thresh,
      coh_ms_i      => coh_ms_i,
      noncoh_dwells_i => noncoh_dwells_i,
      doppler_bin_count_i => doppler_bins_i,
      code_bin_count_i => code_bins_i,
      code_bin_step_i => code_step_i,
      s_valid       => s_valid,
      s_i           => s_i,
      s_q           => s_q,
      acq_done      => acq_done,
      acq_success   => acq_success,
      result_valid  => result_valid,
      result_prn    => result_prn,
      result_dopp   => result_dopp,
      result_code   => result_code,
      result_metric => result_metric
    );

  stim_proc : process
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

    procedure pulse_start is
    begin
      start_pulse <= '1';
      wait until rising_edge(clk);
      start_pulse <= '0';
    end procedure;

    procedure drive_file_sample(i_v : in signed(15 downto 0); q_v : in signed(15 downto 0)) is
    begin
      s_valid <= '1';
      s_i <= i_v;
      s_q <= q_v;
      wait until rising_edge(clk);
      s_valid <= '0';
    end procedure;

    procedure log_acq_tuple(case_tag : in string) is
    begin
      if not G_USE_FILE_INPUT then
        log_msg("ACQ_TUPLE tag=" & case_tag &
                " success=" & std_logic'image(acq_success) &
                " valid=" & std_logic'image(result_valid) &
                " prn=" & integer'image(to_integer(result_prn)) &
                " code=" & integer'image(to_integer(result_code)) &
                " dopp=" & integer'image(to_integer(result_dopp)) &
                " metric=" & integer'image(to_integer(result_metric)));
      end if;
    end procedure;

    procedure run_prn_case(
      prn_v            : in integer;
      thresh_v         : in unsigned(31 downto 0);
      case_name        : in string;
      case_tag         : in string
    ) is
      variable seen_done_v   : boolean := false;
      variable read_status_v : file_open_status;
      variable b0_v          : character;
      variable b1_v          : character;
      variable b2_v          : character;
      variable b3_v          : character;
      variable in_file_cnt_v : integer := 0;
      variable out_samp_cnt_v: integer := 0;
    begin
      detect_thresh <= thresh_v;
      prn_start <= to_unsigned(prn_v, prn_start'length);
      prn_stop <= to_unsigned(prn_v, prn_stop'length);
      pulse_start;

      if G_USE_FILE_INPUT then
        file_open(read_status_v, iq_file, G_INPUT_FILE, read_mode);
        assert read_status_v = open_ok
          report "Unable to open input file: " & G_INPUT_FILE
          severity failure;

        while not endfile(iq_file) loop
          exit when seen_done_v;
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

          drive_file_sample(s16_from_le(b0_v, b1_v), s16_from_le(b2_v, b3_v));
          out_samp_cnt_v := out_samp_cnt_v + 1;
          if acq_done = '1' then
            seen_done_v := true;
          end if;
          in_file_cnt_v := in_file_cnt_v + 1;
        end loop;

        file_close(iq_file);
        if not seen_done_v then
          for i in 0 to 5000 loop
            drive_file_sample(to_signed(0, 16), to_signed(0, 16));
            if acq_done = '1' then
              seen_done_v := true;
              exit;
            end if;
          end loop;
        end if;
      else
        for i in 0 to 5000 loop
          wait until rising_edge(clk);
          if acq_done = '1' then
            seen_done_v := true;
            exit;
          end if;
        end loop;
      end if;

      assert seen_done_v
        report case_name & ": acquisition did not finish."
        severity failure;
      assert acq_success = '1'
        report case_name & ": expected acquisition success."
        severity failure;
      assert result_valid = '1'
        report case_name & ": expected result_valid."
        severity failure;
      assert to_integer(result_prn) = prn_v
        report case_name & ": expected detected PRN to match configured PRN."
        severity failure;
      log_acq_tuple(case_tag);
    end procedure;

    procedure run_fullspace_prn_case(
      prn_v     : in integer;
      thresh_v  : in unsigned(31 downto 0);
      case_name : in string;
      case_tag  : in string
    ) is
    begin
      run_prn_case(prn_v, thresh_v, case_name, case_tag);
      assert result_code /= to_unsigned(0, result_code'length) or
             result_dopp /= to_signed(0, result_dopp'length)
        report case_name & ": expected non-zero code or Doppler estimate."
        severity failure;
    end procedure;

    procedure log_prn_sweep_finding(prn_v : in integer) is
      variable metric_x100_v : integer;
      variable doppler_x10_v : integer;
      variable code_x10_v    : integer;
    begin
      metric_x100_v := (to_integer(result_metric) + 5) / 10; -- metric ~= raw/1000, 2 decimals
      doppler_x10_v := to_integer(result_dopp) * 10;
      code_x10_v := to_integer(result_code) * 10;

      log_msg(
        "prn " & lpad(trim_int_img(prn_v), 3) &
        " doppler " & lpad(fmt_fixed(doppler_x10_v, 1), 8) &
        " metric " & lpad(fmt_fixed(metric_x100_v, 2), 5) &
        " code_offset " & lpad(fmt_fixed(code_x10_v, 1), 7)
      );
    end procedure;

    variable seen_done : boolean;
    variable read_status : file_open_status;
    variable b0          : character;
    variable b1          : character;
    variable b2          : character;
    variable b3          : character;
    variable in_file_cnt : integer := 0;
    variable out_samp_cnt: integer := 0;
    variable run1_metric_v      : unsigned(31 downto 0) := (others => '0');
    variable realistic_thresh_v : unsigned(31 downto 0) := (others => '0');
    variable fullspace_thresh_v : unsigned(31 downto 0) := (others => '0');
    variable step_hz_v                 : integer := 1;
    variable full_dopp_bins_v          : integer := 1;
    variable full_code_bins_v          : integer := 1;
    variable full_space_samples_req_v  : integer := 0;
    variable no_sig_prn_v              : unsigned(5 downto 0) := (others => '0');
    variable no_sig_code_v             : unsigned(10 downto 0) := (others => '0');
    variable no_sig_dopp_v             : signed(15 downto 0) := (others => '0');
    variable no_sig_metric_v           : unsigned(31 downto 0) := (others => '0');
  begin
    rst_n <= '0';
    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;
    rst_n <= '1';
    core_en <= '1';
    if G_USE_FILE_INPUT then
      assert G_FILE_SAMPLE_RATE_SPS = G_DUT_SAMPLE_RATE_SPS
        report "File sample rate must equal DUT sample rate. Pre-decimate input file before replay."
        severity failure;
      s_valid <= '0';
    else
      s_valid <= '1';
      s_i <= to_signed(300, 16);
      s_q <= to_signed(50, 16);
    end if;

    -- Full code-Doppler search space for PRNs found in stimulus file.
    if G_USE_FILE_INPUT then
      step_hz_v := abs(to_integer(doppler_step));
      if step_hz_v < 1 then
        step_hz_v := 1;
      end if;
      full_dopp_bins_v := (abs(to_integer(doppler_max) - to_integer(doppler_min)) / step_hz_v) + 1;
      if full_dopp_bins_v > 81 then
        full_dopp_bins_v := 81;
      elsif full_dopp_bins_v < 1 then
        full_dopp_bins_v := 1;
      end if;
      full_code_bins_v := 1023;
      full_space_samples_req_v := C_SAMPLES_PER_MS * (1 + (full_code_bins_v * full_dopp_bins_v));

      assert G_MAX_FILE_SAMPLES = 0 or G_MAX_FILE_SAMPLES >= full_space_samples_req_v
        report "Run4 PRN 1 full code-doppler requires G_MAX_FILE_SAMPLES >= " &
               integer'image(full_space_samples_req_v) & "."
        severity failure;

      prn_start <= to_unsigned(1, prn_start'length);
      prn_stop <= to_unsigned(1, prn_stop'length);
      code_bins_i <= to_unsigned(full_code_bins_v, code_bins_i'length);
      code_step_i <= to_unsigned(1, code_step_i'length);
      doppler_bins_i <= to_unsigned(full_dopp_bins_v, doppler_bins_i'length);
      run_fullspace_prn_case(1, realistic_thresh_v, "Run4 PRN 1 full code-doppler", "run4_prn1_fullspace");

      -- Derive a robust non-zero threshold from full-space PRN 1 before testing remaining PRNs.
      fullspace_thresh_v := shift_right(result_metric, 4);
      if fullspace_thresh_v = to_unsigned(0, fullspace_thresh_v'length) then
        fullspace_thresh_v := to_unsigned(1, fullspace_thresh_v'length);
      end if;

      log_msg("PRN sweep report (1..32):");
      log_prn_sweep_finding(1);
      for sweep_prn_v in 2 to 32 loop
        run_fullspace_prn_case(
          sweep_prn_v,
          fullspace_thresh_v,
          "Run PRN " & trim_int_img(sweep_prn_v) & " full code-doppler",
          "run_prn" & trim_int_img(sweep_prn_v) & "_fullspace"
        );
        log_prn_sweep_finding(sweep_prn_v);
      end loop;
    else
      log_msg("Skipping full code-doppler PRN 1 run when not using file input.");

      -- Corner run: doppler range inversion must still produce a bounded estimate.
      prn_start <= to_unsigned(1, prn_start'length);
      prn_stop <= to_unsigned(1, prn_stop'length);
      doppler_min <= to_signed(2000, doppler_min'length);
      doppler_max <= to_signed(-2000, doppler_max'length);
      doppler_step <= to_signed(500, doppler_step'length);
      code_bins_i <= to_unsigned(1, code_bins_i'length);
      code_step_i <= to_unsigned(1, code_step_i'length);
      doppler_bins_i <= to_unsigned(1, doppler_bins_i'length);
      s_i <= to_signed(300, 16);
      s_q <= to_signed(50, 16);
      run_prn_case(1, to_unsigned(0, detect_thresh'length),
                   "Run4 doppler inversion corner", "run4_dopp_inversion");
      assert result_dopp >= to_signed(-2000, result_dopp'length) and
             result_dopp <= to_signed(2000, result_dopp'length)
        report "Run4 doppler inversion: estimated Doppler should remain bounded."
        severity failure;

      -- Corner run: no-signal tie behavior should be reproducible across repeated starts.
      doppler_min <= to_signed(-1000, doppler_min'length);
      doppler_max <= to_signed(1000, doppler_max'length);
      doppler_step <= to_signed(500, doppler_step'length);
      code_bins_i <= to_unsigned(1, code_bins_i'length);
      code_step_i <= to_unsigned(1, code_step_i'length);
      doppler_bins_i <= to_unsigned(1, doppler_bins_i'length);
      s_i <= to_signed(0, 16);
      s_q <= to_signed(0, 16);
      run_prn_case(1, to_unsigned(0, detect_thresh'length),
                   "Run5 no-signal reproducibility A", "run5_no_signal_a");
      no_sig_prn_v := result_prn;
      no_sig_code_v := result_code;
      no_sig_dopp_v := result_dopp;
      no_sig_metric_v := result_metric;
      run_prn_case(1, to_unsigned(0, detect_thresh'length),
                   "Run6 no-signal reproducibility B", "run6_no_signal_b");
      assert result_prn = no_sig_prn_v and
             result_code = no_sig_code_v and
             result_dopp = no_sig_dopp_v and
             result_metric = no_sig_metric_v
        report "Run6 no-signal reproducibility: tuple changed across repeated runs."
        severity failure;

      -- Corner run: oversized bin config should complete (FFT mode exercises clipping path quickly).
      if G_DUT_ACQ_IMPL_FFT then
        doppler_min <= to_signed(0, doppler_min'length);
        doppler_max <= to_signed(0, doppler_max'length);
        doppler_step <= to_signed(250, doppler_step'length);
        doppler_bins_i <= to_unsigned(255, doppler_bins_i'length);
        code_bins_i <= to_unsigned(255, code_bins_i'length);
        code_step_i <= to_unsigned(2047, code_step_i'length);
        s_i <= to_signed(300, 16);
        s_q <= to_signed(50, 16);
        run_prn_case(1, to_unsigned(0, detect_thresh'length),
                     "Run7 bin clipping corner", "run7_bin_clipping");
      else
        log_msg("Skipping Run7 bin clipping corner in TD mode due runtime cost.");
      end if;
    end if;

    log_msg("gps_l1_ca_acq_tb completed");
    wait;
  end process;
end architecture;
