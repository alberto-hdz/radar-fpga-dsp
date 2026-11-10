-- Cell-averaging CFAR detector (no divider).
--
-- Window per cell under test (CUT), with N_TRAIN = 16 and N_GUARD = 2:
--   [16 lag cells][2 guard][CUT][2 guard][16 lead cells]  = 37 cells
--
-- Detection: CUT * 2^16 > K_Q * (sum of the 32 training cells)
--   K_Q = round((Pfa^(-1/32) - 1) * 2^16) = 35385 for Pfa = 1e-6
-- so the average never has to be computed and no divider is needed.
--
-- Shift register sr(0) is the newest sample. When sample j arrives the CUT
-- is sample j-18 = sr(18). The training sums are running sums: each new
-- sample does one add and one subtract per side instead of re-adding 32 cells.
--   lead = sr(0)..sr(15)   : + new sample, - sr(15) (moves into the guard)
--   lag  = sr(21)..sr(36)  : + sr(20) (leaves the guard), - sr(36) (falls off)
-- The sums always equal the sum of what is in the shift register, so they
-- never need clearing between pulses.
--
-- m_sum carries the training sum to the testbench so it can check the
-- running sums directly. The detection flag alone can hide a wrong sum when
-- no decision happens to flip.
--
-- Output: one result per input cell, in order, with the cell index in m_bin.
-- The first and last 18 cells of a pulse don't have a full window, so they
-- are reported as no detection (same as the golden model). The last 18 cells
-- come out right after tlast from a flush counter, which runs while the next
-- pulse is still filling its window, so the two never collide.
--
-- Pipeline after a sample is accepted:
--   a : window shift + running sums (and output bookkeeping)
--   b : total training sum, capture CUT
--   c1: threshold multiply, split in two 18/19 x 16 halves (DSP48 M register)
--   c2: product registers (DSP48 P register)
--   c3: recombine: thr = lo + (hi << 18)
--   d : compare -> detection
--
-- Timing note: the first 200 MHz build did K_Q * sum as one 37 x 16 multiply.
-- Because K_Q is a constant, synthesis built it from LUTs and carry chains
-- (17 logic levels, -2.116 ns). Splitting the sum into two halves that each
-- fit a DSP48 multiplier, forcing them into DSPs and pipelining them fixes
-- that. The result is still exact: K*sum = K*lo + (K*hi << 18).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cfar_ca is
  generic (
    PWR_W   : integer := 32;
    N_TRAIN : integer := 16;     -- training cells per side
    N_GUARD : integer := 2;      -- guard cells per side
    K_Q     : integer := 35385;  -- threshold factor, Q0.16 (from radar_model.py)
    BIN_W   : integer := 16      -- range bin counter width
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    s_tvalid : in  std_logic;
    s_tready : out std_logic;
    s_tlast  : in  std_logic;
    s_pwr    : in  unsigned(PWR_W-1 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;
    m_det    : out std_logic;
    m_bin    : out unsigned(BIN_W-1 downto 0);
    m_sum    : out unsigned(PWR_W+4 downto 0)  -- training sum, for verification only
  );
end entity cfar_ca;

architecture rtl of cfar_ca is

  constant HALF  : integer := N_TRAIN + N_GUARD;   -- 18
  constant WIN   : integer := 2*HALF + 1;          -- 37
  constant SIDE_W: integer := PWR_W + 4;           -- 16 cells -> +4 bits
  constant SUM_W : integer := PWR_W + 5;           -- 32 cells -> +5 bits
  constant K_W   : integer := 16;
  constant THR_W : integer := SUM_W + K_W;         -- 53 bits
  constant LO_W  : integer := 18;                  -- low half of the sum
  constant HI_W  : integer := SUM_W - LO_W;        -- high half (19 bits)

  type sr_t is array (0 to WIN-1) of unsigned(PWR_W-1 downto 0);
  signal sr : sr_t := (others => (others => '0'));

  signal lead, lag : unsigned(SIDE_W-1 downto 0) := (others => '0');
  signal n_in      : unsigned(BIN_W-1 downto 0) := (others => '0');

  -- flush of the last HALF cells after tlast
  signal flush_cnt : integer range 0 to HALF := 0;
  signal flush_bin : unsigned(BIN_W-1 downto 0) := (others => '0');

  -- stage a outputs (bookkeeping for the cell whose window is now in sr)
  signal va, la, fa : std_logic := '0';   -- valid, last, force-zero
  signal bin_a      : unsigned(BIN_W-1 downto 0) := (others => '0');

  -- stage b
  signal vb, lb, fb : std_logic := '0';
  signal bin_b      : unsigned(BIN_W-1 downto 0) := (others => '0');
  signal sum_b      : unsigned(SUM_W-1 downto 0) := (others => '0');
  signal cut_b      : unsigned(PWR_W-1 downto 0) := (others => '0');

  -- stage c1: partial products
  signal vc1, lc1, fc1 : std_logic := '0';
  signal bin_c1        : unsigned(BIN_W-1 downto 0) := (others => '0');
  signal cut_c1        : unsigned(PWR_W-1 downto 0) := (others => '0');
  signal sum_c1        : unsigned(SUM_W-1 downto 0) := (others => '0');
  signal plo_c1        : unsigned(LO_W+K_W-1 downto 0) := (others => '0');
  signal phi_c1        : unsigned(HI_W+K_W-1 downto 0) := (others => '0');

  -- stage c2: product registers
  signal vc2, lc2, fc2 : std_logic := '0';
  signal bin_c2        : unsigned(BIN_W-1 downto 0) := (others => '0');
  signal cut_c2        : unsigned(PWR_W-1 downto 0) := (others => '0');
  signal sum_c2        : unsigned(SUM_W-1 downto 0) := (others => '0');
  signal plo_c2        : unsigned(LO_W+K_W-1 downto 0) := (others => '0');
  signal phi_c2        : unsigned(HI_W+K_W-1 downto 0) := (others => '0');

  -- stage c3: threshold
  signal vc, lc, fc : std_logic := '0';
  signal bin_c      : unsigned(BIN_W-1 downto 0) := (others => '0');
  signal thr_c      : unsigned(THR_W-1 downto 0) := (others => '0');
  signal cut_c      : unsigned(PWR_W-1 downto 0) := (others => '0');
  signal sum_c      : unsigned(SUM_W-1 downto 0) := (others => '0');

  -- K_Q is a constant, so without this synthesis builds the multiplies from LUTs
  attribute use_dsp : string;
  attribute use_dsp of plo_c1 : signal is "yes";
  attribute use_dsp of phi_c1 : signal is "yes";

  -- stage d (outputs)
  signal vd, ld, dd : std_logic := '0';
  signal bin_d      : unsigned(BIN_W-1 downto 0) := (others => '0');
  signal sum_d      : unsigned(SUM_W-1 downto 0) := (others => '0');

begin

  s_tready <= '1';

  process (clk)
    variable cut_x : unsigned(THR_W-1 downto 0);
  begin
    if rising_edge(clk) then
      -- default: no new result from stage a this cycle
      va <= '0';
      la <= '0';
      fa <= '0';

      ---------------------------------------------------------------------
      -- stage a: accept a sample, shift the window, update running sums
      ---------------------------------------------------------------------
      if s_tvalid = '1' then
        sr(0) <= s_pwr;
        for m in 1 to WIN-1 loop
          sr(m) <= sr(m-1);
        end loop;
        lead <= lead + resize(s_pwr, SIDE_W) - resize(sr(N_TRAIN-1), SIDE_W);
        lag  <= lag + resize(sr(HALF+N_GUARD), SIDE_W) - resize(sr(WIN-1), SIDE_W);

        -- once HALF samples are in, every new sample completes the window
        -- of cell n_in - HALF
        if n_in >= HALF then
          va    <= '1';
          bin_a <= n_in - HALF;
          if n_in < 2*HALF then
            fa <= '1';  -- first HALF cells: window not full
          end if;
        end if;

        if s_tlast = '1' then
          n_in      <= (others => '0');
          flush_cnt <= HALF;
          flush_bin <= n_in - HALF + 1;
        else
          n_in <= n_in + 1;
        end if;
      end if;

      -- flush the last HALF cells of the previous pulse (always no detection)
      if flush_cnt > 0 then
        va        <= '1';
        fa        <= '1';
        bin_a     <= flush_bin;
        flush_bin <= flush_bin + 1;
        flush_cnt <= flush_cnt - 1;
        if flush_cnt = 1 then
          la <= '1';
        end if;
      end if;

      ---------------------------------------------------------------------
      -- stage b: total training sum and CUT for the cell from stage a
      ---------------------------------------------------------------------
      vb    <= va;
      lb    <= la;
      fb    <= fa;
      bin_b <= bin_a;
      sum_b <= resize(lead, SUM_W) + resize(lag, SUM_W);
      cut_b <= sr(HALF);

      ---------------------------------------------------------------------
      -- stage c1: two partial products, K_Q * low half and K_Q * high half
      ---------------------------------------------------------------------
      vc1    <= vb;
      lc1    <= lb;
      fc1    <= fb;
      bin_c1 <= bin_b;
      cut_c1 <= cut_b;
      sum_c1 <= sum_b;
      plo_c1 <= sum_b(LO_W-1 downto 0) * to_unsigned(K_Q, K_W);
      phi_c1 <= sum_b(SUM_W-1 downto LO_W) * to_unsigned(K_Q, K_W);

      ---------------------------------------------------------------------
      -- stage c2: product registers
      ---------------------------------------------------------------------
      vc2    <= vc1;
      lc2    <= lc1;
      fc2    <= fc1;
      bin_c2 <= bin_c1;
      cut_c2 <= cut_c1;
      sum_c2 <= sum_c1;
      plo_c2 <= plo_c1;
      phi_c2 <= phi_c1;

      ---------------------------------------------------------------------
      -- stage c3: threshold = K_Q*lo + (K_Q*hi << 18) = K_Q * sum (exact)
      ---------------------------------------------------------------------
      vc    <= vc2;
      lc    <= lc2;
      fc    <= fc2;
      bin_c <= bin_c2;
      cut_c <= cut_c2;
      sum_c <= sum_c2;
      thr_c <= resize(plo_c2, THR_W) + shift_left(resize(phi_c2, THR_W), LO_W);

      ---------------------------------------------------------------------
      -- stage d: compare CUT * 2^16 > threshold
      ---------------------------------------------------------------------
      cut_x := shift_left(resize(cut_c, THR_W), K_W);
      vd    <= vc;
      ld    <= lc;
      bin_d <= bin_c;
      sum_d <= sum_c;
      if fc = '0' and cut_x > thr_c then
        dd <= '1';
      else
        dd <= '0';
      end if;

      if rst = '1' then
        sr   <= (others => (others => '0'));
        lead <= (others => '0');
        lag  <= (others => '0');
        n_in <= (others => '0');
        flush_cnt <= 0;
        va <= '0'; vb <= '0'; vc1 <= '0'; vc2 <= '0'; vc <= '0'; vd <= '0';
        la <= '0'; lb <= '0'; lc1 <= '0'; lc2 <= '0'; lc <= '0'; ld <= '0';
      end if;
    end if;
  end process;

  -- synthesis translate_off
  -- A new pulse can't finish a window during the flush (it needs HALF+1
  -- samples first), so a collision means the input broke that assumption.
  process (clk)
  begin
    if rising_edge(clk) then
      assert not (flush_cnt > 0 and s_tvalid = '1' and n_in >= HALF)
        report "cfar_ca: flush collided with a new result" severity error;
    end if;
  end process;
  -- synthesis translate_on

  m_tvalid <= vd;
  m_tlast  <= ld;
  m_det    <= dd;
  m_bin    <= bin_d;
  m_sum    <= sum_d;  -- leave open at the top level and synthesis removes it

end architecture rtl;
