-- 64-tap complex matched filter, transposed form.
--
-- y[n] = sum_k h[k] * x[n-k], complex:
--   yI = xI*hI - xQ*hQ
--   yQ = xI*hQ + xQ*hI
--
-- Transposed form: every input sample is multiplied by all taps at once and
-- the partial sums move down a register chain. The critical path is one
-- multiply or one add no matter how many taps there are.
--   y    = p(0)  + z(1)
--   z(k) = p(k)  + z(k+1)   for k = 1..62
--   z(63)= p(63)
--
-- Pipeline (4 clocks latency):
--   s1: register input
--   s2: 4 real products per tap (maps to DSP48 multipliers)
--   s3: complex combine
--   s4: adder chain + output accumulator
--
-- The front stages run every clock and carry a valid bit. The adder chain only
-- moves when a valid sample reaches it, so gaps in s_tvalid don't break the math.
-- When the last sample of a pulse (tlast) reaches the chain, the chain is
-- cleared so the next pulse starts from zero, like the golden model.
--
-- No backpressure: s_tready is always '1' because ADC data can't be paused.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mf_coef_pkg.all;

entity mf_fir is
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;  -- synchronous, active high
    -- input stream
    s_tvalid : in  std_logic;
    s_tready : out std_logic;
    s_tlast  : in  std_logic;
    s_i      : in  signed(IQ_W-1 downto 0);
    s_q      : in  signed(IQ_W-1 downto 0);
    -- output stream
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;
    m_i      : out signed(OUT_W-1 downto 0);
    m_q      : out signed(OUT_W-1 downto 0)
  );
end entity mf_fir;

architecture rtl of mf_fir is

  type prod_arr_t is array (0 to N_TAPS-1) of signed(PROD_W-1 downto 0);
  type cplx_arr_t is array (0 to N_TAPS-1) of signed(PROD_W downto 0);
  type acc_arr_t  is array (1 to N_TAPS-1) of signed(ACC_W-1 downto 0);

  -- s1: input register
  signal xi1, xq1 : signed(IQ_W-1 downto 0) := (others => '0');
  signal v1, l1   : std_logic := '0';

  -- s2: real products
  signal m_ii, m_qq, m_iq, m_qi : prod_arr_t := (others => (others => '0'));
  signal v2, l2 : std_logic := '0';

  -- s3: complex products
  signal p_i, p_q : cplx_arr_t := (others => (others => '0'));
  signal v3, l3   : std_logic := '0';

  -- s4: transposed adder chain and output accumulator
  signal z_i, z_q     : acc_arr_t := (others => (others => '0'));
  signal acc_i, acc_q : signed(ACC_W-1 downto 0) := (others => '0');
  signal v4, l4       : std_logic := '0';

begin

  s_tready <= '1';

  process (clk)
  begin
    if rising_edge(clk) then
      -- s1: input register
      xi1 <= s_i;
      xq1 <= s_q;
      v1  <= s_tvalid;
      l1  <= s_tlast and s_tvalid;

      -- s2: 4 real multiplies per tap (coefficients are constants)
      for k in 0 to N_TAPS-1 loop
        m_ii(k) <= xi1 * to_signed(H_I(k), COEF_W);
        m_qq(k) <= xq1 * to_signed(H_Q(k), COEF_W);
        m_iq(k) <= xi1 * to_signed(H_Q(k), COEF_W);
        m_qi(k) <= xq1 * to_signed(H_I(k), COEF_W);
      end loop;
      v2 <= v1;
      l2 <= l1;

      -- s3: complex combine (24-bit products -> 25-bit sums)
      for k in 0 to N_TAPS-1 loop
        p_i(k) <= resize(m_ii(k), PROD_W+1) - resize(m_qq(k), PROD_W+1);
        p_q(k) <= resize(m_iq(k), PROD_W+1) + resize(m_qi(k), PROD_W+1);
      end loop;
      v3 <= v2;
      l3 <= l2;

      -- s4: adder chain, only moves on valid samples
      if v3 = '1' then
        acc_i <= resize(p_i(0), ACC_W) + z_i(1);
        acc_q <= resize(p_q(0), ACC_W) + z_q(1);
        if l3 = '1' then
          -- end of pulse: clear the chain so the next pulse starts clean
          z_i <= (others => (others => '0'));
          z_q <= (others => (others => '0'));
        else
          for k in 1 to N_TAPS-2 loop
            z_i(k) <= resize(p_i(k), ACC_W) + z_i(k+1);
            z_q(k) <= resize(p_q(k), ACC_W) + z_q(k+1);
          end loop;
          z_i(N_TAPS-1) <= resize(p_i(N_TAPS-1), ACC_W);
          z_q(N_TAPS-1) <= resize(p_q(N_TAPS-1), ACC_W);
        end if;
      end if;
      v4 <= v3;
      l4 <= l3;

      if rst = '1' then
        v1 <= '0'; v2 <= '0'; v3 <= '0'; v4 <= '0';
        l1 <= '0'; l2 <= '0'; l3 <= '0'; l4 <= '0';
        z_i <= (others => (others => '0'));
        z_q <= (others => (others => '0'));
      end if;
    end if;
  end process;

  -- Output = acc >> OUT_SHIFT. Taking the top bits of a two's complement
  -- number is an arithmetic shift (floors negatives), same as Python's >>.
  m_tvalid <= v4;
  m_tlast  <= l4;
  m_i      <= acc_i(ACC_W-1 downto OUT_SHIFT);
  m_q      <= acc_q(ACC_W-1 downto OUT_SHIFT);

end architecture rtl;
