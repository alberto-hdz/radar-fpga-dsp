-- Magnitude squared: pwr = I^2 + Q^2
--
-- 16-bit signed I/Q in, 32-bit unsigned power out, 2 clocks latency.
--   s1: square I and Q (maps to DSP48 multipliers)
--   s2: add
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
  signal sq_i, sq_q : signed(2*IN_W-1 downto 0) := (others => '0');
  signal pwr        : unsigned(OUT_W-1 downto 0) := (others => '0');
  signal v1, l1, v2, l2 : std_logic := '0';
begin

  s_tready <= '1';

  process (clk)
  begin
    if rising_edge(clk) then
      -- s1: squares (always >= 0, so the sign bit is always 0)
      sq_i <= s_i * s_i;
      sq_q <= s_q * s_q;
      v1   <= s_tvalid;
      l1   <= s_tlast and s_tvalid;

      -- s2: sum
      pwr <= resize(unsigned(sq_i), OUT_W) + resize(unsigned(sq_q), OUT_W);
      v2  <= v1;
      l2  <= l1;

      if rst = '1' then
        v1 <= '0'; v2 <= '0';
        l1 <= '0'; l2 <= '0';
      end if;
    end if;
  end process;

  m_tvalid <= v2;
  m_tlast  <= l2;
  m_pwr    <= pwr;

end architecture rtl;
