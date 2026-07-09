# Correlator-Level Synthetic Multipath Injection (GPS L1 C/A)

This fork adds a config-gated feature to the GPS L1 C/A DLL/PLL tracking block that
injects one or more synthetic multipath rays **at the correlator level**. Re-processing a
clean recording (e.g. a TEXBAT `.bin`) then produces feature outputs (C/N0, code /
pseudorange bias, SQM E/P/L metrics) that carry a physically faithful, per-satellite
multipath signature — including DLL/PLL loop feedback, because the loops react to the
corrupted correlators before they are dumped.

When `multipath_enable=false` (the default), tracking is **byte-identical to the
baseline** — every new code path is guarded, and no extra correlator taps are allocated.

## ⚠️ Verification status

**The code is implemented and reviewed but NOT yet run against real data.** The
environment it was authored in has no C++ toolchain, no build tree, and no TEXBAT `.bin`,
so **validation gate #2 (the multipath error-envelope plot) has only been *documented as a
procedure*, not *executed*.** Gate #2 is the single test that simultaneously proves the
sign convention, the magnitude, and that the whole tap → combine → discriminator chain is
wired correctly — until it is run on a real `cleanStatic` pass and the
`code_error_chips`-vs-delay curve matches the classic S-curve, **treat the implementation
as unverified.** Producing that curve is the first thing to do after building. The
sign-flip and units notes below tell you what to change if the curve is wrong or inverted.

## How it works

For an echo `a·y[n-Δ]` (Δ = excess delay in chips), the correlation against the local
code at base shift `s` obeys the exact autocorrelation-shift identity

```
C'(s) = Σ y'[n]·c[n-s] = C(s) + a·C(s-Δ)
```

So the multipath-corrupted correlator value at each base tap equals the direct
correlation plus `a` times the correlation evaluated at that tap's shift minus Δ. Because
`C(s-Δ)` is computed by the *real* multicorrelator against the *real* samples, the code
autocorrelation shape and front-end bandlimiting are exact — no closed-form R(·) needed.

Implementation (in [dll_pll_veml_tracking.cc](src/algorithms/tracking/gnuradio_blocks/dll_pll_veml_tracking.cc)):

1. When injection is enabled, the correlator tap array is grown from `n_base` (3 for
   E/P/L, 5 for VE/E/P/L/VL) to `n_base · (1 + M)`. Base taps stay at indices
   `[0 .. n_base-1]`, so all existing discriminator / CN0 / lock-detector / dump code is
   untouched. Ray `j`'s echo taps are appended at `[n_base + j·n_base ..]`.
2. `update_multipath_shifts()` places each echo tap at `base_shift[i] - τ_j` and is
   refreshed every integration (the base spacing narrows when the loop goes to extended
   integration). The single existing `Carrier_wipeoff_multicorrelator_resampler()` call
   fills all taps in one pass, reusing the same carrier wipe-off.

   **Units:** the shift array is in *local-code-replica-sample* units — chips scaled by
   `d_code_samples_per_chip` (the replica's samples-per-chip: **1** for GPS L1 C/A, 2/12
   for Gal. E1 CBOC). This is a code-domain quantity, **not** the RF sample rate
   (~24.44 samples/chip @ 25 Msps); the resampler maps it to RF samples via
   `code_phase_step_chips`. The base taps use the identical convention
   (`-early_late_space_chips · d_code_samples_per_chip`), so the echo delay is expressed the
   same way: `τ_code_units = delay_chips · d_code_samples_per_chip`. **No ÷24.44 conversion
   is involved anywhere.**
3. `apply_multipath_injection()` computes each ray's complex amplitude for the current
   integration,

   ```
   a_j = amp_j · exp( i·( phase_j + prn_phase + 2π · diff_doppler_hz_j · t_abs ) )
   ```

   and combines the echo taps into the base taps in place **before** the discriminators run:

   ```
   corr_out[i] += Σ_j a_j · corr_out[n_base + j·n_base + i]
   ```

   - `t_abs = nitems_read(0) / fs` is **absolute recording time** (samples from file start),
     so the onset boundary `t_abs ≥ mp_onset_s` is the **same for every PRN**. A per-channel
     time-since-acquisition clock is deliberately *not* used for the onset, because PRNs
     acquire at different times and that would stagger the clean → dirty boundary; the
     downstream scaler fitting assumes one consistent boundary in recording time.
   - `prn_phase` is a deterministic per-PRN initial-phase offset (golden-ratio
     low-discrepancy in PRN). It decorrelates the fading across satellites even in
     "global params" mode — otherwise every channel shares `phase` + `diff_doppler` and
     fades in unison, an unphysically coherent common-mode signature.
   - Injection is additionally gated on `!pull_in_transitory` (this channel is locked), so
     the pre-onset segment is clean per channel.

The differential-Doppler term makes `a_j` evolve across integrations, producing fading at
the feature rate. The ray's carrier offset is carried entirely by `a_j`; no extra carrier
rotation is applied to the echo taps (that would double-count the wipe-off).

## Config keys

All keys are prefixed with the tracking role (`Tracking_1C.` for GPS L1 C/A) and read in
[dll_pll_conf.cc](src/algorithms/tracking/libs/dll_pll_conf.cc).

```ini
Tracking_1C.multipath_enable=false      ; master switch; false => byte-identical to baseline
Tracking_1C.mp_num_rays=1               ; number of synthetic rays M
Tracking_1C.mp_onset_s=120              ; inject only after this tracking time [s] (post pull-in)

; per-ray keys, index 0 .. M-1:
Tracking_1C.mp_delay_chips_0=0.5        ; excess delay [chips]  (sensitive L1 band ~0.1..1.5)
Tracking_1C.mp_amp_0=0.5                ; signed relative amplitude / MDR (-1..1; sign flips phase 0<->pi)
Tracking_1C.mp_phase_rad_0=0.0          ; initial carrier phase [rad]
Tracking_1C.mp_diff_doppler_hz_0=2.0    ; fading rate [Hz] (a few Hz .. tens of Hz)

; optional, reserved for the realistic-channel extension (see below):
Tracking_1C.mp_cir_file=                ; path to an ITU-R P.681 CIR time series
```

Defaults when a per-ray key is omitted: `delay=0.5`, `amp=0.5`, `phase=0.0`,
`diff_doppler=2.0`. A second ray uses the `_1` suffix, etc. **The resolved per-ray
parameters are logged (and printed) once per channel at `start_tracking`**, so a mistyped
key that silently falls back to a default does not pass unnoticed — check that line against
your intended config.

### Minimal example

Append to the `Tracking_1C` block of, e.g.,
[gnss-sdr_GPS_L1_ishort.conf](conf/File_input/GPS/gnss-sdr_GPS_L1_ishort.conf) (the
`short` + `Ishort_To_Complex` layout matches the TEXBAT `.bin` format):

```ini
Tracking_1C.multipath_enable=true
Tracking_1C.mp_num_rays=1
Tracking_1C.mp_onset_s=120
Tracking_1C.mp_delay_chips_0=0.5
Tracking_1C.mp_amp_0=0.5
Tracking_1C.mp_phase_rad_0=0.0
Tracking_1C.mp_diff_doppler_hz_0=2.0
```

## Validation

Run these in order; each is a gate.

1. **Disabled == baseline.** With `multipath_enable=false`, dumps are byte-identical to a
   baseline run on the same `.bin`.
2. **Multipath error envelope.** One static ray (`mp_amp=0.5`, `mp_diff_doppler=0`,
   `mp_phase=0`), sweep `mp_delay_chips_0` over `0 .. 1.5`. Plot steady-state
   `code_error_chips` (or pseudorange bias) vs delay: it must trace the classic S-shaped
   multipath error envelope — zero at 0, peak near ~0.2–0.5 chips, decaying toward
   ~1.5 chips; the sign inverts when `mp_amp` sign or `mp_phase` (0 vs π) flips.
   **If the curve is inverted in delay, flip the shift sign** in
   `update_multipath_shifts()` (change `- tau_code_units` to `+ tau_code_units`); the units
   are unaffected by the flip. If instead the curve looks *flat* or the echo lands at ~24×
   the intended delay, that is a units error — re-read the "Units" note above (the shift
   array is code-domain, not RF samples).
3. **Loop stays locked.** `carrier_lock_test` / CN0 stay in a sane range at moderate MDR.
4. **Fading visible.** With `mp_diff_doppler_hz≈2`, CN0 and E/P/L show a slow ripple.
5. **Feature deltas.** Full pipeline with vs. without injection: `Delta_c`, `Ratio_c`,
   `pseudorange_variance`, `cn0` move in the injected segment and are unchanged pre-onset.

### Delay arithmetic (L1 C/A @ 25 Msps)

1 chip = 293.05 m; 1 sample = 40 ns = 0.04093 chips ≈ 12.29 m; ≈ 24.44 **RF** samples/chip
(distinct from the local-code replica's `d_code_samples_per_chip` = 1, see the Units note).
The 0.1–1.5 chip band ≈ 30–440 m ≈ 2.4–36.6 RF samples.

## Physical-fidelity caveats (deliberate trade-offs)

These are known, accepted approximations — documented so they are not surprises:

- **The echo duplicates thermal noise.** The identity echoes `a·y[n−Δ]` where `y` is
  signal **plus** front-end noise, so a scaled copy of the noise is injected too, whereas a
  real echo only reflects the signal `x(θ)`. At MDR 0.5 that is ≈ `α²` = 25 % extra noise
  on the echo path — a minor, slightly *pessimistic* C/N0 effect. It is the unavoidable
  price of reusing the real correlator for an exact code/bandlimiting shape.
- **`a_j` is held constant across each integration dwell.** Fine at 1 ms / low fading rate.
  At 20 ms extended integration with `fd` approaching tens of Hz, the intra-dwell phase
  drift (~1 rad) is no longer negligible; the model slightly under-represents fading-induced
  correlation loss within a dwell. Low priority for the first (single-ray, ~2 Hz) study.
- **Global params still echo each channel's own PRN.** "Global (shared-across-PRN)" means
  the *scalar parameters* (delay/amp/phase/fd) are shared; each channel still correlates and
  echoes **its own PRN's** signal (codes are quasi-orthogonal, so only that PRN's echo
  correlates). This is **not** a single Route-A global echo added to the raw input. Per-PRN
  initial phase is additionally decorrelated (see `prn_phase` above).

## Realistic-channel extension (reserved, not yet implemented)

For an ITU-R P.681 (DLR Lehner–Steingass land-mobile) CIR-driven variant, the injection
core is unchanged: only the source of `{τ_j, a_j}` changes. A thin driver keyed off
`mp_cir_file` would, each integration, look up the current CIR taps for this channel's PRN
at `t_abs`, map each tap to `(τ_j, a_j)`, and feed the same combine step. The tap-array
augmentation, `update_multipath_shifts()`, and `apply_multipath_injection()` are designed
to be reused as-is.

## Build

GNSS-SDR must be recompiled and the resulting `gnss-sdr` binary shipped to the deployment
(which runs a prebuilt binary). From a configured build tree:

```sh
cmake --build build -j$(nproc) --target gnss-sdr
```

## Files changed

- [src/algorithms/tracking/libs/dll_pll_conf.h](src/algorithms/tracking/libs/dll_pll_conf.h) /
  [.cc](src/algorithms/tracking/libs/dll_pll_conf.cc) — new config fields + parser.
- [src/algorithms/tracking/gnuradio_blocks/dll_pll_veml_tracking.h](src/algorithms/tracking/gnuradio_blocks/dll_pll_veml_tracking.h) /
  [.cc](src/algorithms/tracking/gnuradio_blocks/dll_pll_veml_tracking.cc) — tap
  augmentation, `update_multipath_shifts()`, `apply_multipath_injection()`, gating.

The adapter chain (`GpsL1CaDllPllTracking` → `BaseDllPllTracking` →
`Dll_Pll_Conf::SetFromConfiguration`) needs no change; the new keys are read generically
for any tracking role.

## Open decisions to confirm before the first study

These do not block the build — the current default behaviour is single-ray, global
(shared-across-PRN) parameters — but they shape the experiment and should be confirmed
with the requester.

1. **Number of rays M for the first study.** Single-ray (`mp_num_rays=1`) is enough for
   the specificity test and matches the Li et al. 2023 single-ray training reference.
   Multi-ray is only needed for realistic ITU-R P.681 CIRs. *Current default: single
   ray.*
2. **Per-PRN vs global parameters for the sweep.** Global is simpler and adequate for a
   first pass. The config keys are currently read per tracking role (delay/amp/phase/fd
   shared across PRNs — but each channel still echoes its own PRN, and per-PRN initial
   phase is decorrelated so the fading is not synchronous). The block architecture already
   allows fully per-PRN parameters (needed for realistic CIRs), which would be keyed by PRN
   with fall-back to the global value. *Current default: global scalars + per-PRN phase
   decorrelation.*
3. **Which scenarios get reprocessed.** At minimum `cleanStatic`; likely also
   `cleanDynamic`. *To be confirmed.*

The pre-onset segment must remain clean (guaranteed by the `mp_onset_s` gate) — the
downstream scaler fitting relies on this, mirroring the existing spoof-onset design.
