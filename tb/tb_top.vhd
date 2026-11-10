-- End-to-end testbench for radar_top.
-- Raw 12-bit I/Q in (vectors/mf_in.txt), detection flags out, compared with
-- vectors/cfar_expected.txt along with the range bin and tlast on every
-- result. Two pulses back to back, with an idle cycle every few samples.
-- Run from the repo root.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.mf_coef_pkg.all;

entity tb_top is
  generic (
    IN_FILE  : string  := "vectors/mf_in.txt";
    EXP_FILE : string  := "vectors/cfar_expected.txt";
    N_WIN    : integer := 1024;
    N_PULSES : integer := 2;
    GAP_EVERY: integer := 5
  );
end entity tb_top;

architecture sim of tb_top is
  constant T_CLK : time := 5 ns;

  signal clk  : std_logic := '0';
  signal rst  : std_logic := '1';
  signal done : boolean   := false;

  signal s_tvalid, s_tready, s_tlast : std_logic := '0';
  signal s_i, s_q : signed(IQ_W-1 downto 0) := (others => '0');
  signal m_tvalid, m_tlast, m_det : std_logic;
  signal m_bin : unsigned(15 downto 0);
begin

  clk <= not clk after T_CLK/2 when not done else clk;

  dut : entity work.radar_top
    port map (
      clk => clk, rst => rst,
      s_tvalid => s_tvalid, s_tready => s_tready, s_tlast => s_tlast,
      s_i => s_i, s_q => s_q,
      m_tvalid => m_tvalid, m_tlast => m_tlast, m_det => m_det, m_bin => m_bin
    );

  stim : process
    file f     : text;
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

  check : process
    file fe     : text;
    variable le : line;
    variable ev, idx, errors, hits : integer := 0;
  begin
    errors := 0;
    hits   := 0;
    for p in 1 to N_PULSES loop
      file_open(fe, EXP_FILE, read_mode);
      idx := 0;
      while idx < N_WIN loop
        wait until rising_edge(clk);
        if m_tvalid = '1' then
          readline(fe, le);
          read(le, ev);

          if m_det = '1' then
            hits := hits + 1;
            report "Detection pulse " & integer'image(p) & " bin " & integer'image(to_integer(m_bin));
          end if;

          if (m_det = '1') /= (ev = 1) then
            errors := errors + 1;
            if errors <= 10 then
              report "Mismatch pulse " & integer'image(p) & " bin " & integer'image(idx) &
                     ": got det=" & std_logic'image(m_det) & " expected " & integer'image(ev)
                severity error;
            end if;
          end if;
          if to_integer(m_bin) /= idx then
            errors := errors + 1;
            if errors <= 10 then
              report "Bin index wrong: got " & integer'image(to_integer(m_bin)) &
                     " expected " & integer'image(idx) severity error;
            end if;
          end if;
          if (idx = N_WIN-1) /= (m_tlast = '1') then
            errors := errors + 1;
            report "tlast wrong at pulse " & integer'image(p) & " bin " & integer'image(idx)
              severity error;
          end if;
          idx := idx + 1;
        end if;
      end loop;
      file_close(fe);
    end loop;

    assert errors = 0
      report "FAIL: " & integer'image(errors) & " mismatches" severity failure;
    report "PASS: " & integer'image(N_PULSES * N_WIN) & " range bins matched end to end, " &
           integer'image(hits) & " detections";
    done <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until done for 200 us;
    assert done report "FAIL: timeout" severity failure;
    wait;
  end process;

end architecture sim;
