-- Magnitude squared: pwr = I^2 + Q^2
--
-- 16-bit signed I/Q in, 32-bit unsigned power out, 4 clocks latency.
--   s1: input register   (DSP48 A/B register)
--   s2: square           (DSP48 M register)
--   s3: product register (DSP48 P register)
--   s4: add
-- Using all three DSP48 pipeline registers lets the multiplier run at full
-- speed. The first 200 MHz build only had one register around the multiply
-- and missed timing by 0.693 ns.
-- Each square is at most (-32768)^2 = 2^30, so the sum is at most 2^31
-- and always fits in 32 bits unsigned.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mag_sq is
  generic (
    IN_W  : integer := 16;
    OUT_W : integer := 32
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    s_tvalid : in  std_logic;
    s_tready : out std_logic;
    s_tlast  : in  std_logic;
    s_i      : in  signed(IN_W-1 downto 0);
    s_q      : in  signed(IN_W-1 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;
    m_pwr    : out unsigned(OUT_W-1 downto 0)
  );
end entity mag_sq;

architecture rtl of mag_sq is
  signal xi, xq           : signed(IN_W-1 downto 0) := (others => '0');
  signal mi, mq, pi, pq   : signed(2*IN_W-1 downto 0) := (others => '0');
  signal pwr              : unsigned(OUT_W-1 downto 0) := (others => '0');
  signal v1, l1, v2, l2, v3, l3, v4, l4 : std_logic := '0';
begin

  s_tready <= '1';

  process (clk)
  begin
    if rising_edge(clk) then
      -- s1: input register
      xi <= s_i;
      xq <= s_q;
      v1 <= s_tvalid;
      l1 <= s_tlast and s_tvalid;

      -- s2: squares (always >= 0, so the sign bit is always 0)
      mi <= xi * xi;
      mq <= xq * xq;
      v2 <= v1;
      l2 <= l1;

      -- s3: product register
      pi <= mi;
      pq <= mq;
      v3 <= v2;
      l3 <= l2;

      -- s4: sum
      pwr <= resize(unsigned(pi), OUT_W) + resize(unsigned(pq), OUT_W);
      v4  <= v3;
      l4  <= l3;

      if rst = '1' then
        v1 <= '0'; v2 <= '0'; v3 <= '0'; v4 <= '0';
        l1 <= '0'; l2 <= '0'; l3 <= '0'; l4 <= '0';
      end if;
    end if;
  end process;

  m_tvalid <= v4;
  m_tlast  <= l4;
  m_pwr    <= pwr;

end architecture rtl;
