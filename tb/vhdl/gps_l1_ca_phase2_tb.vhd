library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_l1_ca_phase2_tb is
  generic (
    G_USE_FILE_INPUT      : boolean := true;
    G_INPUT_FILE          : string  := "2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat";
    G_FILE_SAMPLE_RATE_SPS: integer := 4000000;
    G_DUT_SAMPLE_RATE_SPS : integer := 2000000;
    G_MAX_FILE_SAMPLES    : integer := 200000;
    G_REQUIRE_PVT         : boolean := false
  );
end entity;

architecture tb of gps_l1_ca_phase2_tb is
  constant C_CLK_PER : time := 20 ns;

  signal clk          : std_logic := '0';
  signal rst_n        : std_logic := '0';
  signal sample_valid : std_logic := '0';
  signal sample_i     : signed(15 downto 0) := (others => '0');
  signal sample_q     : signed(15 downto 0) := (others => '0');
  signal sample_ready : std_logic;

  signal ctrl_wreq    : std_logic := '0';
  signal ctrl_waddr   : unsigned(7 downto 0) := (others => '0');
  signal ctrl_wdata   : std_logic_vector(31 downto 0) := (others => '0');
  signal ctrl_wack    : std_logic;
  signal ctrl_rreq    : std_logic := '0';
  signal ctrl_raddr   : unsigned(7 downto 0) := (others => '0');
  signal ctrl_rdata   : std_logic_vector(31 downto 0);
  signal ctrl_rack    : std_logic;

  signal uart_txd     : std_logic;
begin
  clk <= not clk after C_CLK_PER / 2;

  dut : entity work.gps_l1_ca_phase2_top
    generic map (
      G_NUM_CHANNELS      => 5
    )
    port map (
      clk          => clk,
      rst_n        => rst_n,
      sample_valid => sample_valid,
      sample_i     => sample_i,
      sample_q     => sample_q,
      sample_ready => sample_ready,
      ctrl_wreq    => ctrl_wreq,
      ctrl_waddr   => ctrl_waddr,
      ctrl_wdata   => ctrl_wdata,
      ctrl_wack    => ctrl_wack,
      ctrl_rreq    => ctrl_rreq,
      ctrl_raddr   => ctrl_raddr,
      ctrl_rdata   => ctrl_rdata,
      ctrl_rack    => ctrl_rack,
      uart_txd     => uart_txd
    );

  stimulus : process
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

    procedure ctrl_write(addr : in natural; data : in std_logic_vector(31 downto 0)) is
    begin
      wait until rising_edge(clk);
      ctrl_waddr <= to_unsigned(addr, ctrl_waddr'length);
      ctrl_wdata <= data;
      ctrl_wreq  <= '1';
      wait until rising_edge(clk);
      ctrl_wreq  <= '0';
      wait until rising_edge(clk);
    end procedure;

    procedure ctrl_read(addr : in natural; data : out std_logic_vector(31 downto 0)) is
    begin
      wait until rising_edge(clk);
      ctrl_raddr <= to_unsigned(addr, ctrl_raddr'length);
      ctrl_rreq  <= '1';
      wait until rising_edge(clk);
      ctrl_rreq  <= '0';
      wait until rising_edge(clk);
      data := ctrl_rdata;
    end procedure;

    procedure drive_sample(i_v : in signed(15 downto 0); q_v : in signed(15 downto 0)) is
    begin
      wait until rising_edge(clk);
      while sample_ready /= '1' loop
        sample_valid <= '0';
        wait until rising_edge(clk);
      end loop;

      sample_valid <= '1';
      sample_i <= i_v;
      sample_q <= q_v;

      wait until rising_edge(clk);
      sample_valid <= '0';

      for i in 0 to 23 loop
        wait until rising_edge(clk);
      end loop;
    end procedure;

    variable read_status    : file_open_status;
    variable b0             : character;
    variable b1             : character;
    variable b2             : character;
    variable b3             : character;
    variable in_file_cnt    : integer := 0;
    variable out_samp_cnt   : integer := 0;
    variable decim          : integer := 1;
    variable sample_cnt     : integer := 0;
    variable status_reg     : std_logic_vector(31 downto 0) := (others => '0');
    variable alloc_reg      : std_logic_vector(31 downto 0) := (others => '0');
    variable pvt_seen       : boolean := false;
    variable alloc_channels : integer;
  begin
    rst_n <= '0';
    wait for 200 ns;
    rst_n <= '1';
    wait for 100 ns;

    -- Phase 2 configuration.
    ctrl_write(16#04#, x"0000051F"); -- ch mask 0x1F, preferred count 5
    ctrl_write(16#08#, x"00000A01"); -- PRN range 1..10
    ctrl_write(16#0C#, x"00000020"); -- low acq threshold for TB
    ctrl_write(16#10#, x"0000EC78"); -- doppler min -5000
    ctrl_write(16#14#, x"00001388"); -- doppler max +5000
    ctrl_write(16#18#, x"000001F4"); -- doppler step 500

    -- core_en + rescan + tracking + uart + nav + obs + pvt.
    ctrl_write(16#00#, x"000000FD");
    -- Keep all run enables, clear rescan pulse.
    ctrl_write(16#00#, x"000000F9");

    if G_USE_FILE_INPUT then
      assert G_FILE_SAMPLE_RATE_SPS mod G_DUT_SAMPLE_RATE_SPS = 0
        report "File sample rate must be integer multiple of DUT sample rate."
        severity failure;
      decim := G_FILE_SAMPLE_RATE_SPS / G_DUT_SAMPLE_RATE_SPS;

      file_open(read_status, iq_file, G_INPUT_FILE, read_mode);
      assert read_status = open_ok
        report "Unable to open input file: " & G_INPUT_FILE
        severity failure;

      report "Phase 2 replay input: " & G_INPUT_FILE;
      report "Input Fs=" & integer'image(G_FILE_SAMPLE_RATE_SPS) &
             " -> DUT Fs=" & integer'image(G_DUT_SAMPLE_RATE_SPS) &
             ", decimation=" & integer'image(decim);

      while not endfile(iq_file) loop
        if G_MAX_FILE_SAMPLES > 0 and out_samp_cnt >= G_MAX_FILE_SAMPLES then
          exit;
        end if;

        if endfile(iq_file) then exit; end if;
        read(iq_file, b0);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b1);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b2);
        if endfile(iq_file) then exit; end if;
        read(iq_file, b3);

        if (in_file_cnt mod decim) = 0 then
          drive_sample(s16_from_le(b0, b1), s16_from_le(b2, b3));
          out_samp_cnt := out_samp_cnt + 1;
        end if;
        in_file_cnt := in_file_cnt + 1;
      end loop;
      file_close(iq_file);
      report "Phase 2 file replay done. input_samples=" & integer'image(in_file_cnt) &
             ", injected_samples=" & integer'image(out_samp_cnt);
    else
      while sample_cnt < 150000 loop
        drive_sample(
          to_signed((sample_cnt mod 511) - 255, 16),
          to_signed(255 - (sample_cnt mod 511), 16)
        );
        sample_cnt := sample_cnt + 1;
      end loop;
    end if;

    for i in 0 to 299 loop
      wait for 1 ms;
      ctrl_read(16#40#, status_reg);
      ctrl_read(16#4C#, alloc_reg);
      alloc_channels := 0;
      for b in 0 to 4 loop
        if alloc_reg(b) = '1' then
          alloc_channels := alloc_channels + 1;
        end if;
      end loop;

      if status_reg(2) = '1' then
        pvt_seen := true;
        report "Phase 2 PVT valid observed. allocated_channels=" & integer'image(alloc_channels);
        exit;
      end if;
    end loop;

    if G_REQUIRE_PVT then
      assert pvt_seen
        report "PVT valid not observed before timeout."
        severity failure;
    else
      if not pvt_seen then
        report "PVT valid not observed before timeout." severity warning;
      end if;
    end if;

    wait for 1 ms;
    assert false report "gps_l1_ca_phase2_tb completed" severity note;
    wait;
  end process;
end architecture;
