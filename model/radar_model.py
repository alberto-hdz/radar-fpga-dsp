"""
Bit-true golden model for the FPGA radar signal processor.

Chain: LFM chirp -> 64-tap complex matched filter -> |I|^2 + |Q|^2 -> CA-CFAR

Every integer this script writes to vectors/ is exactly what the VHDL
has to produce, so the testbenches can check the RTL bit for bit.
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # no display needed, so this also runs in CI
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
VEC = ROOT / "vectors"
DOCS = ROOT / "docs"

# ---------------------------------------------------------------------------
# System parameters
# ---------------------------------------------------------------------------
C = 3e8              # speed of light (m/s)
FS = 200e6           # complex sample rate = 1 sample per clock at 200 MHz
B = 50e6             # chirp bandwidth -> range resolution c/(2B) = 3 m
N_PULSE = 64         # pulse length in samples (0.32 us) -> 64 filter taps
N_WIN = 1024         # receive window (range bins per pulse)

IQ_MAX = 2047        # 12-bit signed, clipped symmetric to +/-2047
COEF_MAX = 2047      # 12-bit signed coefficients, same clipping

# Filter output scaling.
# Worst case |yI| = 64 taps * 2 (complex add) * 2047 * 2047 = 536,346,752 < 2^30,
# so the accumulator fits in 31 signed bits. Shifting right by 15 leaves a
# 16-bit result that can never overflow, so I don't need saturation logic.
OUT_SHIFT = 15

# CA-CFAR settings
N_TRAIN = 16         # training cells per side (32 total, a power of 2)
N_GUARD = 2          # guard cells per side (keeps target energy out of the noise estimate)
PFA = 1e-6           # design probability of false alarm
HALF = N_TRAIN + N_GUARD  # 18 cells on each side of the cell under test

# Targets: (start sample, amplitude). Targets 2 and 3 are 10 bins apart on
# purpose: the strong one sits inside the weak one's training window, which
# raises the noise estimate and can hide the weak one (CFAR masking).
TARGETS = [(200, 0.8), (600, 1.0), (610, 0.4)]
NOISE_STD = 0.5      # complex noise std; per-sample SNR is near 0 dB before compression
SEED = 1             # fixed seed so the vectors are identical every run


def make_chirp():
    """Baseband LFM chirp sweeping -B/2 to +B/2 over the pulse."""
    t = np.arange(N_PULSE) / FS
    T = N_PULSE / FS
    k = B / T  # chirp rate (Hz/s)
    return np.exp(1j * np.pi * k * (t - T / 2) ** 2)


def quantize(x, full_scale):
    """Scale so the largest I or Q value hits full_scale, then round to int."""
    peak = max(np.max(np.abs(x.real)), np.max(np.abs(x.imag)))
    s = full_scale / peak
    i = np.clip(np.round(x.real * s), -full_scale, full_scale).astype(np.int64)
    q = np.clip(np.round(x.imag * s), -full_scale, full_scale).astype(np.int64)
    return i, q


def make_rx(chirp, rng):
    """Received window: delayed, scaled, random-phase chirp echoes plus noise."""
    rx = np.zeros(N_WIN, dtype=complex)
    for start, amp in TARGETS:
        phase = np.exp(1j * rng.uniform(0, 2 * np.pi))
        rx[start:start + N_PULSE] += amp * phase * chirp
    noise = rng.normal(0, NOISE_STD / np.sqrt(2), (2, N_WIN))
    return rx + noise[0] + 1j * noise[1]


def matched_filter(rI, rQ, hI, hQ):
    """
    Bit-true complex FIR, done with integer math the same way the hardware does it:
      yI = rI*hI - rQ*hQ
      yQ = rI*hQ + rQ*hI
    I keep the first N_WIN outputs because the hardware makes one output per
    input sample, with the delay line cleared to zero at the start of each pulse.
    """
    yI = np.convolve(rI, hI) - np.convolve(rQ, hQ)
    yQ = np.convolve(rI, hQ) + np.convolve(rQ, hI)
    yI, yQ = yI[:N_WIN], yQ[:N_WIN]
    # Arithmetic shift right floors negative numbers, same as VHDL shift_right(signed)
    return yI >> OUT_SHIFT, yQ >> OUT_SHIFT


def ca_cfar(p, k_q):
    """
    Cell-averaging CFAR with no divider.
    Threshold T = alpha * mean = (Pfa^(-1/N) - 1) * sum, so I only need one
    constant K = Pfa^(-1/N) - 1, stored as Q0.16 (k_q).
    Detection: p[i] * 2^16 > k_q * sum(training cells)
    Cells within HALF of either edge have an incomplete window, so I force them to 0.
    """
    det = np.zeros(N_WIN, dtype=np.int64)
    thr = np.zeros(N_WIN)
    for i in range(HALF, N_WIN - HALF):
        lag = p[i - HALF:i - N_GUARD].sum()            # 16 cells before the guards
        lead = p[i + N_GUARD + 1:i + HALF + 1].sum()   # 16 cells after the guards
        s = int(lag + lead)
        det[i] = 1 if (int(p[i]) << 16) > k_q * s else 0
        thr[i] = k_q * s / 2**16
    return det, thr


def write_vectors(rI, rQ, hI, hQ, yI, yQ, p, det):
    VEC.mkdir(exist_ok=True)
    np.savetxt(VEC / "mf_in.txt", np.c_[rI, rQ], fmt="%d")
    np.savetxt(VEC / "mf_coef.txt", np.c_[hI, hQ], fmt="%d")   # h[0]..h[63]
    np.savetxt(VEC / "mf_out_expected.txt", np.c_[yI, yQ], fmt="%d")
    np.savetxt(VEC / "pwr_expected.txt", p, fmt="%d")
    np.savetxt(VEC / "cfar_expected.txt", det, fmt="%d")


def plot(p, thr, det):
    DOCS.mkdir(exist_ok=True)
    # The filter peak shows up N_PULSE-1 samples after the echo starts,
    # so I subtract that delay to plot true range.
    rng_m = (np.arange(N_WIN) - (N_PULSE - 1)) * C / (2 * FS)
    p_db = 10 * np.log10(p + 1)
    valid = np.arange(HALF, N_WIN - HALF)
    hits = np.flatnonzero(det)

    plt.figure(figsize=(10, 4.5))
    plt.plot(rng_m, p_db, lw=0.8, label="Matched filter output |y|^2")
    plt.plot(rng_m[valid], 10 * np.log10(thr[valid] + 1), lw=1.2, label="CA-CFAR threshold")
    plt.plot(rng_m[hits], p_db[hits], "rv", ms=7, label="Detections")
    for start, _ in TARGETS:
        plt.axvline(start * C / (2 * FS), color="gray", ls=":", lw=0.8)
    plt.xlabel("Range (m)")
    plt.ylabel("Power (dB, integer units)")
    plt.title("Pulse compression + CA-CFAR (bit-true model)")
    plt.legend(loc="upper right")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(DOCS / "range_profile.png", dpi=150)


def main():
    rng = np.random.default_rng(SEED)

    chirp = make_chirp()
    rx = make_rx(chirp, rng)
    rI, rQ = quantize(rx, IQ_MAX)

    # Matched filter = time-reversed conjugate of the transmitted pulse
    hI, hQ = quantize(np.conj(chirp[::-1]), COEF_MAX)

    yI, yQ = matched_filter(rI, rQ, hI, hQ)
    p = yI ** 2 + yQ ** 2   # fits in 32 bits unsigned (actually < 2^31)

    k = PFA ** (-1 / (2 * N_TRAIN)) - 1
    k_q = int(round(k * 2**16))
    det, thr = ca_cfar(p, k_q)

    write_vectors(rI, rQ, hI, hQ, yI, yQ, p, det)
    plot(p, thr, det)

    hits = np.flatnonzero(det)
    print(f"K = {k:.5f}  ->  K_Q (Q0.16) = {k_q}")
    print(f"Max |yI|,|yQ| = {max(np.abs(yI).max(), np.abs(yQ).max())} (16-bit limit 32767)")
    print(f"Max power     = {p.max()} (32-bit limit {2**32 - 1})")
    print(f"Expected peaks at samples {[s + N_PULSE - 1 for s, _ in TARGETS]}")
    print(f"CFAR detections at samples {hits.tolist()}")
    print(f"Wrote vectors to {VEC} and plot to {DOCS / 'range_profile.png'}")


if __name__ == "__main__":
    main()
