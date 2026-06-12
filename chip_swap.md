# Chip Selection & Swap Decisions

Running record of component choices for this controller and the firmware/hardware
implications of each. Board context: **bgkatz `3phase_integrated`** controller (STM32F446RE)
running this firmware, paired with a **SteadyWin GIM6010-8** motor.

_Last updated: 2026-06-11._

## Summary

| Component | From | To / Choice | Status | Firmware impact |
|---|---|---|---|---|
| Motor | — | SteadyWin **GIM6010-8** | ✅ configured | params in flash/defaults |
| Gate driver | DRV8323**RS** (SPI) | DRV8323**RH** (hardware strap) | 🟡 firmware ready (`DRV_USE_SPI`), board respin needed | `DRV_USE_SPI` switch |
| Encoder | MA710 (MPS, SPI) | MagnTek **MT6816** / MT6835 | 🟡 firmware ready (`ENC_TYPE`), needs chip + bench | `ENC_TYPE` switch |

Legend: ✅ done · 🟡 partially done / pending hardware · 🔵 under evaluation

---

## 1. Motor — SteadyWin GIM6010-8

**Decision:** use the GIM6010-8 integrated joint motor (motor only; driven by the bgkatz
controller, **not** SteadyWin's own ODrive-based driver).

### Key specs (from datasheet §1.2/§1.3)
| Spec | Value | → Firmware param |
|---|---|---|
| Pole pairs | **14** | `PPAIRS` (also auto-set by calibration) |
| Reduction ratio | **8:1** | `GR` (set via `g8` menu / boot default) |
| Torque constant | **0.47 N·m/A** | `KT` |
| Phase resistance | **0.48 Ω** | `R_PHASE` (observer only) |
| Phase inductance | **368 µH** | `L_D`/`L_Q` in `hw_config.h` |
| Rated / stall current | **10.5 A / 25 A** | `I_MAX_CONT` / `I_MAX` |
| Rated / stall torque | 5 / 11 N·m | — |
| Max speed | 420 rpm (motor) → ~5.5 rad/s output | `V_MAX` (optional) |
| Encoder | 16-bit single-turn absolute | (on the controller board, not the motor) |

### Applied to firmware
- `hw_config.h`: `L_D = L_Q = 0.000368` (inert in current control path, set for correctness).
- `main.c` boot defaults: `GR=8`, `KT=0.47`, `I_MAX_CONT=10.5`, `PPAIRS=14`.
- Per-motor config lives in **flash** (set via serial `s` menu); pole-pairs is **auto-measured**
  by encoder calibration (`c`). Verify it prints `Pole Pairs: 14`.

### Caveats
- Datasheet numbers describe SteadyWin's integrated driver; only the *motor-intrinsic* values
  transfer. Encoder CPR / ADC scaling / `V_BUS_MAX` belong to the **bgkatz controller**.
- 8:1 gearbox friction/backlash is the dominant sim2real gap; anti-cogging gives little here.

### Links
- Datasheet (MakerBotics mirror): <https://github.com/MakerBotics/GIM6010-8-Joint-Robot-Motor-/blob/master/SteadyWin%20GIM6010-8.pdf>
- Vendor: SteadyWin (Jiangxi Sitaiwei Automation Equipment Co., Ltd.)

---

## 2. Gate driver — DRV8323RS → DRV8323RH

**Decision:** firmware supports both variants via a compile switch; moving to the **RH**
(hardware-interface) variant is feasible but needs a board respin (4 pins).

The **`R`** (DRV8323**R** = integrated buck regulator) is identical on both — irrelevant to
firmware. Only **`S` (SPI) vs `H` (hardware strap)** matters.

### Part numbers
| Variant | Interface | LCSC | Package |
|---|---|---|---|
| DRV8323**RS** (current) | SPI | — | 48-pin VQFN (RGZ) |
| DRV8323**RH**RGZR (target) | hardware strap | **C543035** | 48-pin VQFN (RGZ) |

### Pin difference — only 4 pins (same RGZ footprint)
| Pin | RS (SPI) | RH (hardware) | RH type |
|----|----|----|----|
| 32 | `nSCS` | **`GAIN`** | 4-level resistor |
| 31 | `SCLK` | **`VDS`** | 7-level resistor (OCP trip) |
| 30 | `SDI` | **`IDRIVE`** | 7-level resistor (gate current) |
| 29 | `SDO` | **`MODE`** | 4-level resistor (PWM mode) |

Everything else identical: 3× CSA (`SOA/B/C`,`SPA/B/C`,`SNA/B/C`), `VREF`, gate drives,
`ENABLE`, `nFAULT`, `CAL`, buck pins. `nFAULT` (GPIO) and `CAL` (auto-offset-cal GPIO) exist
on **both**.

### RH strap resistors (to reproduce the firmware's old SPI defaults)
Reference all straps to **DVDD (3.3 V internal reg pin)**, not VM.

| Pin | Want | **Strap** | Datasheet level |
|---|---|---|---|
| MODE | 3× PWM | **47 kΩ → AGND** | 47k-AGND = 3× PWM |
| GAIN | 40 V/V (keeps `I_SCALE`) | **tie DVDD** | AGND=5, 47k-AGND=10, Hi-Z=20, **DVDD=40** |
| VDS | ~0.45 V OCP (no exact step) | **Hi-Z=0.6 V**, or 75k-AGND=0.26 V | ladder below |
| IDRIVE | moderate | **Hi-Z** (120 mA src / 240 mA sink) | tune to FET gate charge |

- **VDS ladder (V):** AGND=0.06 · 18k-AGND=0.13 · 75k-AGND=0.26 · Hi-Z=0.6 · 75k-DVDD=1.13 ·
  18k-DVDD=1.88 · DVDD=**disabled** (trip current = `V_VDS / Rds(on)`).
- **4-level pins (GAIN/MODE):** tie-AGND / 47 kΩ-AGND / Hi-Z(float) / tie-DVDD.
- **7-level pins (IDRIVE/VDS):** tie-AGND / 18 kΩ-AGND / 75 kΩ-AGND / Hi-Z / 75 kΩ-DVDD /
  18 kΩ-DVDD / tie-DVDD. Use ±5% resistors.
- `nFAULT` needs an **external pull-up** to DVDD; `CAL` tie low (or to a GPIO for auto-cal).

### Firmware support — `DRV_USE_SPI` switch (implemented)
`hw_config.h`:
```c
#define DRV_USE_SPI     1   // 1 = DRV8323RS (SPI),  0 = DRV8323RH (hardware strap)
#define DRV_HW_CSA_GAIN 40  // (RH only) must match the GAIN strap: 5/10/20/40
//#define DRV_NFAULT  GPIOB, GPIO_PIN_X   // (RH only) if nFAULT wired to MCU
```
What it gates: SPI config block in `main.c`, `i_scale` derivation in `foc.c`
(`I_SCALE × 40/DRV_HW_CSA_GAIN` on RH), and `drv_enable_gd`/`drv_disable_gd`/`drv_print_faults`
in `drv8323.c` (RH → ENABLE-pin arm/disarm + `nFAULT` GPIO). Both configs build clean.

### Trade-offs (RH) — acceptable for a fixed-motor design
- Lose runtime gain/OCP/IDRIVE config and detailed SPI fault decode (keep `nFAULT` line).
- Deadtime / OCP-mode / retry frozen at H/W defaults.
- `DRV_HW_CSA_GAIN` **must** match the GAIN strap or current scaling is wrong (GAIN=40 keeps
  `I_SCALE` unchanged).
- **Disarm = sleep** (no SPI COAST + `INLx` strapped high) → ~1 ms charge-pump wakeup on
  re-arm (first ~1 ms of torque weak). Needs bench validation.
- **Not pin/footprint drop-in on an RS board:** pins 29–32 route to MCU SPI on RS; RH needs the
  4 strap resistors instead → board respin.

### Links
- TI datasheet (DRV8320/8320R/8323/8323R, SLVSDJ3): <https://www.ti.com/lit/ds/symlink/drv8323.pdf>
- LCSC DRV8323RHRGZR (C543035): <https://www.lcsc.com/product-detail/C543035.html>

---

## 3. Position encoder — MA710 → MagnTek (Chinese, LCSC)

**Decision (evaluating):** replace the MPS **MA710** (14-bit SPI magnetic angle sensor) with a
**MagnTek (杭州麦歌恩)** equivalent stocked on LCSC.

### Candidates
| Part | Res. | Interface | LCSC | Notes |
|---|---|---|---|---|
| **MT6816** | 14-bit | SPI (mode 3) | `C879560` (ACD), `C2925077` (STD) | **Closest 1:1** to MA710; AMR |
| **MT6835** | **21-bit** | 4-wire SPI + ABZ/PWM/UVW | search "MT6835" on LCSC | **Upgrade**, FOC/servo-grade |
| **MT6701** | 14-bit | SSI / I²C / ABZ / PWM | `C2856764` (STD) | Cheap, popular (SimpleFOC); SSI read-only |

**Leading pick:** MT6816 (direct match) or MT6835 (higher resolution).

### Firmware support — `ENC_TYPE` switch (implemented for MT6816)
`hw_config.h`:
```c
#define ENC_TYPE_ORIG    0   // original board sensor: 1x 16-bit read, 65536 CPR, SPI mode 0
#define ENC_TYPE_MT6816  1   // MagnTek MT6816: 16384 CPR, SPI mode 3, two-register read
#define ENC_TYPE  ENC_TYPE_ORIG   // <-- set to ENC_TYPE_MT6816 when an MT6816 is fitted
```
Flipping it pulls in MT6816-correct values everywhere via derived macros:
`ENC_CPR=16384`, `ENC_LUT_SHIFT=7`, `ENC_SPI_CPOL/CPHA = mode 3`, `ENC_SPI_PRESCALER=/4`
(11.25 MHz < 15.6 MHz max), `ENC_WARMUP_CMD=0x8300`. Both configs build clean.

What it gates:
- `spi.c` — `hspi3` CLKPolarity/CLKPhase/BaudRatePrescaler driven by `ENC_SPI_*`.
- `position_sensor.c` — `ps_sample` does the **two-register MT6816 read** (0x8300→reg 0x03,
  0x8400→reg 0x04) and assembles `raw = (d03<<6)|(d04>>2)`; LUT indexing uses `ENC_LUT_SHIFT`.
  `ps_warmup` uses `ENC_WARMUP_CMD`.

### Datasheet-confirmed protocol (MT6816, §7.6)
- **SPI mode 3** (CPOL=1, CPHA=1); read command frame = `R/W(1) | A6..A0 | 8 dummy` → read
  0x03 = `0x8300`, read 0x04 = `0x8400`; register byte returned in the **low byte**.
- 14-bit angle: reg `0x03 = Angle<13:6>`, reg `0x04[7:2] = Angle<5:0>`, `[1] = No-Mag warning`,
  `[0] = parity`; reg `0x05[3] = over-speed (>25 krpm)` — free encoder-fault signals.
- TSCK min 64 ns → **max ~15.6 MHz** (so the original /2 = 22.5 MHz was too fast).

### Still to do (RH-style hardware/bench items)
- Pick **MT6816** (14-bit, direct) vs **MT6835** (21-bit upgrade — would need its own
  `ENC_TYPE`, different frame/CPR).
- Bench: re-run encoder calibration (`c`) with the MT6816 — the 128-entry LUT regenerates for
  16384 CPR automatically (no calibration code change). Verify angle direction/zero.
- Confirm the `0x8300/0x8400` read bytes on a real part (standard MT6816 format, used here).

### Links
- MT6816 — LCSC: <https://www.lcsc.com/product-detail/C879560.html>
- MT6816 datasheet: <https://datasheet.lcsc.com/lcsc/2112021230_Magn-Tek-MT6816CT-STD_C2925077.pdf>
- MT6835 datasheet: <https://www.magntek.com.cn/upload/pdf/202407/MT6835_Rev.1.3.pdf>
- MT6701 datasheet: <https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2109011830_Magn-Tek-MT6701CT-STD_C2856764.pdf>
- MT6701 in SimpleFOC: <https://community.simplefoc.com/t/mt6701-magnetic-position-encoder-support/2618>

---

## Open items
- [ ] DRV8323RH: board respin (4 strap resistors + nFAULT pull-up) before populating RH.
- [ ] DRV8323RH: bench-validate ENABLE-pin arm/disarm wakeup behavior.
- [ ] Encoder: pick MT6816 vs MT6835; MT6816 path is implemented (`ENC_TYPE_MT6816`) — bench:
      recalibrate + verify angle direction/zero + confirm read-command bytes on a real part.
