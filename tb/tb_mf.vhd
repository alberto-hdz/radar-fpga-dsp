-- Self-checking testbench for mf_fir.
-- Sends the golden-model input twice back to back (checks the clear on tlast)
-- with an idle cycle every few samples (checks the valid gating), and compares
-- every output against vectors/mf_out_expected.txt. Run from the repo root.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.mf_coef_pkg.all;

entity tb_mf is
  generic (
    IN_FILE  : string  := "vectors/mf_in.txt";
    EXP_FILE : string  := "vectors/mf_out_expected.txt";
    SIM_FILE : string  := "vectors/mf_out_sim.txt";
    N_WIN    : integer := 1024;
    N_PULSES : integer := 2;
    GAP_EVERY: integer := 5      -- idle cycle after every 5 samples (0 = no gaps)
  );
end entity tb_mf;

architecture sim of tb_mf is
  constant T_CLK : time := 5 ns;  -- 200 MHz

  signal clk  : std_logic := '0';
  signal rst  : std_logic := '1';
  signal done : boolean   := false;

  signal s_tvalid, s_tready, s_tlast : std_logic := '0';
  signal s_i, s_q : signed(IQ_W-1 downto 0) := (others => '0');
  signal m_tvalid, m_tlast : std_logic;
  signal m_i, m_q : signed(OUT_W-1 downto 0);
begin

  -- clock stops when the checker is done, so the simulation ends by itself
  clk <= not clk after T_CLK/2 when not done else clk;

  dut : entity work.mf_fir
    port map (
      clk => clk, rst => rst,
      s_tvalid => s_tvalid, s_tready => s_tready, s_tlast => s_tlast,
      s_i => s_i, s_q => s_q,
      m_tvalid => m_tvalid, m_tlast => m_tlast,
      m_i => m_i, m_q => m_q
    );

  -- Stimulus: read I/Q pairs and stream them in
  stim : process
    file f    : text;
    variable l : line;
    variable vi, vq, n : integer;
  begin
    rst <= '1';
    for c in 1 to 5 loop
      wait until rising_edge(clk);
    end loop;
    rst <= '0';

    for p in 1 to N_PULSES loop
      file_open(f, IN_FILE, read_mode);
      n := 0;
      while not endfile(f) loop
        readline(f, l);
        read(l, vi);
        read(l, vq);
        n := n + 1;
        wait until rising_edge(clk);
        s_i      <= to_signed(vi, IQ_W);
        s_q      <= to_signed(vq, IQ_W);
        s_tvalid <= '1';
        if endfile(f) then
          s_tlast <= '1';
        else
          s_tlast <= '0';
        end if;
        if GAP_EVERY > 0 and (n mod GAP_EVERY) = 0 then
          wait until rising_edge(clk);
          s_tvalid <= '0';
          s_tlast  <= '0';
        end if;
      end loop;
      file_close(f);
    end loop;

    wait until rising_edge(clk);
    s_tvalid <= '0';
    s_tlast  <= '0';
    wait;
  end process;

  -- Checker: compare each valid output with the expected file
  check : process
    file fe    : text;
    file fs    : text;
    variable le, ls : line;
    variable ei, eq, idx, errors : integer := 0;
  begin
    file_open(fs, SIM_FILE, write_mode);
    errors := 0;
    for p in 1 to N_PULSES loop
      file_open(fe, EXP_FILE, read_mode);
      idx := 0;
      while idx < N_WIN loop
        wait until rising_edge(clk);
        if m_tvalid = '1' then
          readline(fe, le);
          read(le, ei);
          read(le, eq);

          write(ls, to_integer(m_i));
          write(ls, string'(" "));
          write(ls, to_integer(m_q));
          writeline(fs, ls);

          if to_integer(m_i) /= ei or to_integer(m_q) /= eq then
            errors := errors + 1;
            if errors <= 10 then
              report "Mismatch pulse " & integer'image(p) & " sample " & integer'image(idx) &
                     ": got (" & integer'image(to_integer(m_i)) & ", " & integer'image(to_integer(m_q)) &
                     ") expected (" & integer'image(ei) & ", " & integer'image(eq) & ")"
                severity error;
            end if;
          end if;

          if (idx = N_WIN-1) /= (m_tlast = '1') then
            errors := errors + 1;
            report "tlast wrong at pulse " & integer'image(p) & " sample " & integer'image(idx)
              severity error;
          end if;
          idx := idx + 1;
        end if;
      end loop;
      file_close(fe);
    end loop;
    file_close(fs);

    assert errors = 0
      report "FAIL: " & integer'image(errors) & " mismatches" severity failure;
    report "PASS: " & integer'image(N_PULSES * N_WIN) & " samples matched";
    done <= true;
    wait;
  end process;

  -- Watchdog: fail if the DUT never finishes
  watchdog : process
  begin
    wait until done for 200 us;
    assert done report "FAIL: timeout" severity failure;
    wait;
  end process;

end architecture sim;
