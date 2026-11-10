-- Radar signal processor top level.
--
--   12-bit I/Q (ADC) -> mf_fir -> mag_sq -> cfar_ca -> detection + range bin
--
-- One sample per clock at 200 MHz = 200 MSPS real-time throughput.
-- Everything runs on one clock. In a real system the ADC would be on its own
-- clock and would need a clock domain crossing (async FIFO) in front of this.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mf_coef_pkg.all;

entity radar_top is
  generic (
    K_Q   : integer := 35385;  -- CFAR threshold factor, Q0.16
    BIN_W : integer := 16
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    -- ADC sample stream
    s_tvalid : in  std_logic;
    s_tready : out std_logic;
    s_tlast  : in  std_logic;
    s_i      : in  signed(IQ_W-1 downto 0);
    s_q      : in  signed(IQ_W-1 downto 0);
    -- detection stream (one result per range bin)
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;
    m_det    : out std_logic;
    m_bin    : out unsigned(BIN_W-1 downto 0)
  );
end entity radar_top;

architecture rtl of radar_top is
  -- matched filter -> magnitude squared
  signal mf_tvalid, mf_tlast : std_logic;
  signal mf_i, mf_q          : signed(OUT_W-1 downto 0);
  -- magnitude squared -> CFAR
  signal pw_tvalid, pw_tlast : std_logic;
  signal pw_pwr              : unsigned(31 downto 0);
  -- unused handshakes (no backpressure anywhere in the chain)
  signal ms_tready, cf_tready : std_logic;
begin

  u_mf : entity work.mf_fir
    port map (
      clk => clk, rst => rst,
      s_tvalid => s_tvalid, s_tready => s_tready, s_tlast => s_tlast,
      s_i => s_i, s_q => s_q,
      m_tvalid => mf_tvalid, m_tlast => mf_tlast,
      m_i => mf_i, m_q => mf_q
    );

  u_mag : entity work.mag_sq
    generic map (IN_W => OUT_W, OUT_W => 32)
    port map (
      clk => clk, rst => rst,
      s_tvalid => mf_tvalid, s_tready => ms_tready, s_tlast => mf_tlast,
      s_i => mf_i, s_q => mf_q,
      m_tvalid => pw_tvalid, m_tlast => pw_tlast, m_pwr => pw_pwr
    );

  u_cfar : entity work.cfar_ca
    generic map (PWR_W => 32, N_TRAIN => 16, N_GUARD => 2, K_Q => K_Q, BIN_W => BIN_W)
    port map (
      clk => clk, rst => rst,
      s_tvalid => pw_tvalid, s_tready => cf_tready, s_tlast => pw_tlast,
      s_pwr => pw_pwr,
      m_tvalid => m_tvalid, m_tlast => m_tlast, m_det => m_det, m_bin => m_bin,
      m_sum => open  -- verification-only port, removed by synthesis
    );

end architecture rtl;
