# DCLK vs. GCLK Frequency Requirements & Caveats

*Based on MBI5153 / MBI505x LED Driver Specifications*

---

## 0. Observations

* GCLK of 8.9Mhz
* DCLK of 8Mhz

Is a good combination. The datasheet says GCLK needs to be just 20% or greater of DCLK but this seems wrong. GCLK > DCLK from what I see.

## 1. Maximum Frequency Limits

### At $V_{DD} = 5.0\text{V}$
* **Max DCLK Frequency:** $30\text{ MHz}$
* **Max GCLK Frequency:** $33\text{ MHz}$ *(drops to $16.6\text{ MHz}$ when the GCLK multiplier is enabled)*

### At $V_{DD} = 3.3\text{V}$
* **Max DCLK Frequency:** $25\text{ MHz}$
* **Max GCLK Frequency:** $20\text{ MHz}$ *(drops to $10\text{ MHz}$ when the GCLK multiplier is enabled)*

---

## 2. Core Frequency Requirement (The 20% Rule)

* **Minimum Ratio:** $F_{\text{GCLK}} \ge 0.20 \times F_{\text{DCLK}}$
* **Reason:** GCLK drives the internal transfers from the shift registers into embedded SRAM. If GCLK runs too slowly relative to DCLK, data corruption will occur during latch operations.

---

## 3. Key Operational Caveats & Special Conditions

### Shared Clock Mode ($F_{\text{GCLK}} = F_{\text{DCLK}}$)
* DCLK cannot stop immediately after frame data finishes transmitting.
* DCLK **must** be stopped after executing the VSYNC command.
* DCLK must explicitly stop before the internal GCLK counter reaches `1023` to properly process dead-time and blanking intervals.

### GCLK Multiplier Mode
* **Duty Cycle:** Requires a strict **50% duty cycle** due to dual-edge triggering.
* **Frequency Cap:** Maximum input frequency is halved ($16.6\text{ MHz}$ at $5.0\text{V}$, $10\text{ MHz}$ at $3.3\text{V}$).

### VSYNC Timing & Handshaking
* **Pause Before VSYNC:** GCLK must be brought low and stopped before issuing a VSYNC command to satisfy $t_{\text{SU2}}$ and $t_{\text{H2}}$ setup/hold requirements.
* **Post-Latch Delay:** After sending the final data latch for a frame, at least **50 GCLK cycles** must elapse before the VSYNC signal is pulsed so that data can be loaded from SRAM into the internal display buffer.