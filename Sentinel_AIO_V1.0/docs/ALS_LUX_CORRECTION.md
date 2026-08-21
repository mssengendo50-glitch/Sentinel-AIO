# ALS lux correction, and the route to a 200 ms PIR-to-image budget

Status 2026-08-21. Two separate things, in the order they have to be done.

---

## Part 1 — the lux formula is wrong, and the model is built on it

`LTR329_CalculateLux()` third branch, in **both** `Sentinel_AIO_V1.0` and
`Test firmware/TEST`:

```c
} else if (ratio < 0.85f) {
    lux = (5.9260f * ch0 - 0.1185f * ch1);   // wrong: 10x, and sign flipped
}
```

Should be, per Lite-On Appendix A:

```c
    lux = (0.5926f * ch0 + 0.1185f * ch1);
```

### Three independent confirmations

**Vendor.** Appendix A for LTR-303ALS / LTR-329ALS:

```
ELSEIF (RATIO < 0.85 && RATIO >= 0.64)
  ALS_LUX = (0.5926 * CH0 + 0.1185 * CH1) / ALS_GAIN / ALS_INT
```

The ESPHome `ltr_als_ps` reference agrees verbatim. Note the LTR-329 datasheet
itself only says *"Refer to Appendix A for the lux formula"* and does not
contain it — which is how this got missed.

**Continuity.** A piecewise lux formula must be continuous at its own
boundaries. As a coefficient on `ch0`:

| boundary | branch below | branch above (correct) | this firmware |
|---|---|---|---|
| ratio 0.45 | 2.6791 | 2.6791 — continuous | 2.6791 |
| ratio 0.64 | 0.8033 | 0.8033 — continuous | **5.7153 — a 7.11× cliff** |

**Hardware.** Same scene, minutes apart:

```
ch0=233 ch1=407  ratio 0.6359  ->  4.193 lux
ch0=233 ch1=419  ratio 0.6426  -> 27.731 lux
```

Identical `ch0`, twelve counts of `ch1`, 6.6× apart. Indoor tungsten sits at
ratio ≈ 0.64 on this part, so this is exactly where the device lives: one ADC
count of noise swings the AE seed by 7×.

Corrected, all four logged readings collapse to **2.1 – 4.2 lux** — one dim
room, which is what it was.

### Why it cannot simply be corrected

`Test firmware/TEST` carries the same expression, so every training label was
computed with it. The model is *self-consistent with the bug* — fixing the
formula alone makes the seed worse.

Worse than that: the 7.11× fold maps a genuinely ~7× darker population onto the
same lux values as the branch below, so the training set holds **contradictory
labels at the same input**. Relabelling cannot repair it.

And a second, independent defect: TEST ran the ALS at a fixed **1X gain /
100 ms**, no auto-ranging (`main.c:171`). Production auto-ranges to 96X at
50 ms. Converting:

```
scenes now reading 127-233 counts at 96X/50ms
       =           2.6-4.9 counts at 1X/100ms
datasheet dark ADC count (Lux = 0): 0 to 6 counts
```

Anything below ~5 lux was inside the dark band during collection; ~80 lux was
needed for 100 counts. **The low-light half of the training set is noise.** That
is what explains the one run whose lux landed on the *correct* branch and was
still 1.9 stops out.

### The fix, in order

1. Correct the coefficient and sign.
2. **Recollect.** Auto-ranging on, as production runs it. Log raw `ch0`, `ch1`,
   `als_gain`, `int_time` **alongside** lux — never lux alone. Had TEST done
   this, the dataset would be salvageable by recomputing offline instead of
   needing a rebuild.
3. Retrain `als_model_separate.c`.
4. Only then shorten `MIN_FRAMES_SEEDED`.

The collection rig already exists: the FSBL drift line prints
`(lux, converged exposure, converged gain)` per boot, and the seed payload
carries `als_ch0` / `als_ch1` / `als_gain`.

---

## Part 2 — closing the gap to 200 ms

Measured, three consistent runs: **569 / 569 / 579 ms** PIR → image captured.

```
PIR edge  -> rail up        ~29 ms   MSPM0
rail up   -> HAL_Init       ~63 ms   BootROM + startup
HAL_Init  -> pipes started   155 ms  FSBL init
frames (10 x 32.9)           329 ms
                            ------
                            ~576 ms
```

The MSPM0 owns 29 ms of 576. **The time is on the STM32.**

### MSPM0 — small, but worth banking

| # | change | effect | risk |
|---|---|---|---|
| M1 | *(done)* trigger check first in IDLE | removes a 150 ms+ intermittent spike when a PIR lands on a minute boundary | none |
| M2 | *(done)* arm SPI before logging IO2 | removes ~47 ms from inside FSBL's 60 ms seed timeout | none |
| M3 | move `UART_0` off LFCLK (currently 9600 baud, blocking, 1.04 ms/char) | removes 30–115 ms worst-case stalls anywhere in the path | changes what survives clock-gating and standby — a real trade |
| M4 | ALS integration 50 → 200 ms | accuracy, not speed: 4× the counts. `settle=0 ms` today, so ~190 ms of slack exists | costs 2 stops of bright-end headroom |

Nominal gain: near zero. Worst-case gain: large. M3 and M4 are about making the
number *reliable*, not smaller.

### STM32 — where the budget actually is

| # | change | saves | risk |
|---|---|---|---|
| S1 | assert camera `NRST`/`PWR_EN` at `MX_FSBL_GPIO_INIT` (t=0) and wait out the remainder later, instead of `HAL_Delay(50)+HAL_Delay(5)` blocking inside `HAL_DCMIPP_MspInit` | **~50 ms** | check sensor power-on spec |
| S2 | I2C1 100 kHz → 400 kHz (`Timing = 0x30C0EDFF` ≈ 101 kHz; ~92 register writes in IMX335 init) | **~28 ms** | low — IMX335 is rated 400 kHz |
| S3 | `MIN_FRAMES_SEEDED` 10 → 3 | **~230 ms** | **blocked on Part 1** |
| S4 | 60 fps sensor mode (`IMX335_SetFramerate` is commented out) — or a lower capture resolution; the model input is 256×256 | ~50 ms at 3 frames | check CSI ceiling at 2592×1944 10-bit 2-lane |
| S5 | shrink the FSBL image — BootROM copy time scales with size | part of the 63 ms | measure first |
| S6 | stop power-cycling the STM32 (retention instead of cold boot) | **~63 ms**, removes BootROM entirely | architectural; changes the power budget |

### Stack-up

```
today                                        576 ms
 S1 camera delay overlapped        -50   ->  526
 S2 I2C at 400 kHz                 -28   ->  498
 S3 frames 10 -> 3                -230   ->  268   <- needs Part 1 first
 S4 60 fps (3 frames @ 16.5 ms)    -50   ->  218
 S5/S6 BootROM                     -20+  ->  ~198
```

**S1 + S2 + S3 lands at ~268 ms and needs no architectural change.** Getting
under 200 requires S4 or S6 — a faster sensor mode, or not cold-booting the
STM32. There is no tuning path to 200 ms without one of them.

### Two questions to settle with the customer first

**What is "image captured"?** This measures to *frames captured in FSBL*. If it
means the JPEG on the SD card, the Appli adds `Media open took ~1600 ms` plus
inference and compression — a different conversation entirely, and none of the
above moves it.

**Is the PIR edge t = 0?** The ZDP323B does its own filtering before asserting
(`FILTER_STEP_3` / `FILTER_TYPE_C`). If the budget is 200 ms from *motion*, the
sensor's own detection latency is inside it and nothing here measures it.

---

## Also open

- **RTC has lost its time.** Logs show `2000-06-01 02:03:31` — exactly the
  syscfg calendar default. Images are named `Img-2000-06-01-*`. Unrelated to
  latency; chase separately.
- **Spurious IO2 edge after the seed.** FSBL toggles once and leaves IO2 high;
  the Appli's `MX_GPIO_Init` then manufactures a second edge, which the MSPM0
  services against an unarmed slave (`FF FF FF FF`). Harmless today because the
  edge counter tolerates it. FSBL driving IO2 low before the jump removes it.
- **`PIR -> IMAGE CAPTURED` can print a stale total.** After a software reset
  (`NVIC_SystemReset`) the rail never drops, so the MSPM0's stopwatch never
  restarts — one run reported 95072 ms. Needs a plausibility guard.
- **The end-to-end print mislabels the split.** `(MSPM0 245 + STM32 334)` reads
  as if the MSPM0 owns 245 ms; only ~29 ms is the MSPM0, ~63 ms is BootROM and
  ~143 ms is FSBL before it asks for the seed.
