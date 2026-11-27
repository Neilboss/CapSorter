# Brag Plan: CapSorter

## What is this app?
CapSorter is a 100% offline, dual-controller vision-guided Cartesian pick-and-place robot repository (`github.com/Neilboss/CapSorter`) spanning bare-metal V1 (Teensy 4.1 + ESP32-CAM) and edge-Linux V2 (Raspberry Pi Zero 2 W + 160° IMX219 CSI + 3.5" SPI Touch LCD) over a shared Arduino Mega 2560 motion and fail-safe layer.

## The angle
Most tabletop robotic sorters depend on laptops, cloud APIs, or rigid grid pockets — and fail the moment a gripper nudges a neighboring cap. CapSorter lets you scatter caps anywhere on an open flat 300×300 mm plywood plate, re-images the board in parallel during every bin drop (`CMD_SCAN_WINDOW_OPEN`), and ships as a complete open-source engineering repo: hardened device firmware, Stages 0–8 physical bring-up tools, a 15-page CVPR-format paper, 28 automated SIL tests, and a live 2D web simulator.

## Hook (first 2–3 seconds)
Full-bleed dark studio push-in on the 500×500×500 mm Cartesian gantry lifting a red bottle cap (`portal_robot_system.png`) with crisp mono hardware telemetry: **"Scatter bottle caps anywhere."** → **"CapSorter sees, adapts, and sorts every last one."**

## Key moments (the middle)
- **V1 vs. V2 Dual-Controller Partitioning**: Side-by-side bare-metal Teensy 4.1 + ESP32-CAM (`359 KiB` static DMAMEM) vs. consolidated Pi Zero 2 W + 160° IMX219 + 3.5" SPI LCD HUD (`~45 ms` vision), united on one Arduino Mega 2560 + CRC-16 UART + dual-pole E-stop layer.
- **Working App Centerpiece (Closed-Loop Simulator)**: Live 2D CapSorter V2 simulator picking from a touching red cap pair, bumping an adjacent yellow cap by `+32 mm`, flashing `CMD_SCAN_WINDOW_OPEN (0xA5)` during the 800 ms bin drop to re-acquire the bumped cap's new `(X, Y) mm` coordinates, and routing straight from Bin 1 without a wasted home park (`100% completion vs. 67.5% open-loop`).
- **Complete Repository Tour**: 4-card breakdown of `firmware/`, `firmware/bringup/` (Stages 0–8 manual + 8 bench scripts), `research_paper/` (15-page PDF), and `website/ + tests/` (28/28 passing tests).

## Outro / punchline
Official `CapSorter` logo centered on dark grid with subtle audio-reactive glow, **"Offline Vision-Guided Cartesian Pick & Place"**, `github.com/Neilboss/CapSorter` • `capsorter.vercel.app`, and team credit (`By Neil Shettigar, Kahan & Shourya`).

## User flow worth showing
**Entry → Key Action → Result**:
1. Scattered & touching bottle caps sit on the continuous 300×300 mm open plywood workspace (`0x5B` single-cap target dispatched).
2. Gantry lifts Red Cap #1 and nudges neighboring Yellow Cap by `+32 mm`; upon reaching Bin 1 (`X = 355 mm`), Mega emits `CMD_SCAN_WINDOW_OPEN (0xA5)` while dropping the cap.
3. Pi Zero 2 W re-acquires the bumped cap's updated `(X, Y) mm` coordinates in ~45 ms, Bin 1 count increments, and the gantry dispatches straight to the updated spot.

## Tone
- Preset: `polished`
- Creative direction: cinematic industrial hardware & embedded robotics product film
- Interpretation: Confident dark-mode engineering aesthetic (`#0a0d14`), razor-sharp typography (`Outfit` + `JetBrains Mono`), purposeful smooth motion, generous reading holds, and restrained low-HF-risk SFX.

## Format: landscape — 1920x1080 (30fps)
## Duration: 23.5s

## Visual identity (from the project)
- Background: `#0a0d14` (cards `#111622` / `#131927`, subtle `40px` engineering grid)
- Accent: `#ff6b00` (V1 industrial orange), `#00d2ff` (V2 precision cyan), `#10b981` (emerald ready/pass)
- Text: `#f8fafc` primary, `#cbd5e1` secondary (WCAG AA/AAA compliant on `#0a0d14` and `#111622`)
- Display font: `Outfit`
- Body / Telemetry font: `Inter` / `JetBrains Mono`
- Strongest visual element: `portal_robot_system.png`, `gantry_3axis_cad.png`, `logo.png`, and the interactive V2 closed-loop 2D gantry simulator UI.

## Share copy (draft)
CapSorter is a 100% offline, dual-controller bottle-cap sorting robot built in two open-source architectures (Teensy 4.1 + ESP32-CAM and Raspberry Pi Zero 2 W + 160° IMX219 CSI + 3.5" SPI LCD) over a shared Arduino Mega 2560 motion & fail-safe layer. Pipelined bin-side re-imaging (`CMD_SCAN_WINDOW_OPEN`) adapts to bumped caps mid-cycle — backed by a 15-page paper, Stages 0–8 bench bring-up tools, and a live 2D simulator at capsorter.vercel.app.

## Audio direction
- Role: steady, clean electronic-corporate underscore with restrained hardware/UI SFX accents
- Music: `happy-beats-business-moves-vol-12-by-ende-dot-app.mp3` (109.96 BPM, steady and clean)
- Music treatment: start at `0.0s`, `data-volume="0.32"`, `data-fade-in="0.4"`, `data-fade-out="2.0"` across final logo hold
- Music cue guidance: Bundled preset read (`happy-beats-business-moves-vol-12-by-ende-dot-app.music-cues.md`, 109.96 BPM). Major strong cues locked: `8.74s` (Scene 3 simulator reveal), `13.11s` (bin-side scan payoff & Bin 1 increment), `17.47s` (final repo card lock). Beat-grid windows used for Scene 2 cards (`4.91s`, `6.00s`, `7.09s`) and Scene 4 cards (`15.84s`, `16.38s`, `16.93s`, `17.47s`) with full-set reading holds afterward.
- Audio-reactive treatment: subtle; modulate hero radial glow and active telemetry border brightness slightly on beat envelope (no equalizer bars or strobing)
- SFX posture: restrained, low/medium HF risk (Kenney `impactSoft_medium`, `interface/click_002..003`, `interface/bong_001`, `casino/card-slide-1`, `impactBell_heavy_000`)
- Restraint rule: never stack loud high-frequency hits or overwhelm typography

## Storyboard

### Scene 1 — Hardware Hook — 4.30s (`0.00s–4.30s`)
Full-bleed studio shot of `portal_robot_system.png` slowly pushing in (`scale 1.07 -> 1.0`) behind a dark gradient scrim and subtle `40px` engineering grid. Top mono telemetry pill (`500×500×500 MM • 100% OFFLINE • ZERO CLOUD`) enters at `0.20s`, headline **"Scatter bottle caps anywhere."** locks in at `0.56s`, sub-headline **"CapSorter sees, adapts, and sorts every last one."** enters at `1.64s` and holds fully settled for `2.2s` alongside the `CapSorter` logo badge.
Sequential/interaction: yes — 3-step text + badge cascade on beat grid (`0.20s`, `0.56s`, `1.64s`).
Audio intent: clean warm impact anchoring the opening hardware shot.
Audio-coupled idea: low-HF-risk reveal thud on hook headline at `0.52s`.
Music: steady clean bed fading in (`0.4s`).
Transition mood: clean stagger fade → Scene 2.

### Scene 2 — Dual-Controller Architecture (V1 vs. V2) — 4.44s (`4.30s–8.74s`)
Dark blueprint stage showcasing CapSorter's dual front-ends with `gantry_3axis_cad.png` thumbnail in the shared motion bar. Left V1 Orange card (`4.91s`), Right V2 Cyan card (`6.00s`), and bottom Shared Arduino Mega 2560 Safety Layer (`7.09s`) slide in cleanly and hold together until `8.60s`.
Sequential/interaction: yes — 3 architecture layers arrive on every-other-beat cadence (`4.91s`, `6.00s`, `7.09s`) and hold settled together.
Audio intent: crisp mechanical precision.
Audio-coupled idea: subtle card slide accents on V1/V2 arrivals.
Music: steady clean bed.
Transition mood: snappy hard beat cut at `8.74s` (strong cue) → Scene 3.

### Scene 3 — Live Closed-Loop Bin-Side Re-Imaging Simulator — 6.55s (`8.74s–15.29s`)
Authentic 2D recreation of CapSorter V2's interactive simulator: left `300×300 mm` continuous plywood board with `18..282 mm` safe reach border, 4 color bins (`BIN 1 RED`..`BIN 4 BLU`), live moving Y-bridge + X-carriage + parallel X-jaws; right V2 Telemetry & Opcode stream (`0x5B`, `0xA5`, `0xA6`, `100% vs. 67.5%`). Gantry picks a touching Red cap (`9.2s–10.6s`), physically nudges a neighboring Yellow cap `+32 mm` (annotated dashed vector), traverses to Bin 1 (`10.6s–11.9s`), triggers a glowing cyan **`CMD_SCAN_WINDOW_OPEN (0xA5)`** scan wave across the board (`12.0s–13.11s`) that snaps a fresh target reticle onto the Yellow cap's new `(X, Y) mm` coordinates while Bin 1 ticks `0 → 1`, then routes straight from Bin 1 toward the updated Yellow cap (`13.2s–15.1s`).
Sequential/interaction: yes — simulated gantry pick, physical neighbor bump, bin-side scan sweep, live coordinate update, and bin counter increment.
Audio intent: tactile electromechanical confirmation on clamp, camera scan ping, and bin deposit.
Audio-coupled idea: click on jaw clamp (`10.1s`), soft bell chime on `CMD_SCAN_WINDOW_OPEN` (`12.0s`), warm soft thud on Bin 1 deposit (`13.05s`, beat-locked to `13.11s` strong cue).
Music: steady clean bed driving motion.
Transition mood: clean slide → Scene 4.

### Scene 4 — Complete Open-Source GitHub Repo — 4.37s (`15.29s–19.66s`)
Header **"Everything in one open-source repo"** (`15.29s`) over a 2×2 grid of real repository modules arriving on consecutive beats (`15.84s`, `16.38s`, `16.93s`, `17.47s` strong cue) and holding fully settled together for `2.1s`:
1. `firmware/` — Mega 2560, Pi Zero 2 W, Teensy 4.1, ESP32-CAM & 3.5" LCD HUD
2. `firmware/bringup/` — Stages 0–8 Physical Testing Guide & 8 Staged Bench Scripts
3. `research_paper/` — 15-Page CVPR-Format Paper & SIL Ablation Benchmarks
4. `website/ + tests/` — Live 2D Simulator, 12-Page Dossiers & 28/28 Passing Tests
Sequential/interaction: yes — 4 repo module cards snap in on beat grid and hold together for easy reading.
Audio intent: crisp checklist momentum landing firmly on `17.47s`.
Audio-coupled idea: subtle click accents + warm thud on 4th card (`17.47s`).
Music: steady clean bed.
Transition mood: smooth crossfade → Scene 5.

### Scene 5 — Brand Outro & Repository Links — 3.84s (`19.66s–23.50s`)
Clean dark studio stage with subtle pulsing orange/cyan radial aura. Official `CapSorter` logo (`logo.png`) scales smoothly into center (`19.75s`), tagline **"Offline Vision-Guided Cartesian Pick & Place"** (`20.25s`), high-contrast pill **`github.com/Neilboss/CapSorter  •  capsorter.vercel.app`** (`20.75s`), and credit **`By Neil Shettigar, Kahan & Shourya`** (`21.25s`), holding cleanly to `23.50s` as music fades out (`21.5s–23.5s`).
Sequential/interaction: yes — logo + tagline + URLs settle cleanly with >2.2s final hold.
Audio intent: resonant, confident resolution.
Audio-coupled idea: warm resonant bell (`impactBell_heavy_000.ogg`) on logo arrival (`19.70s`) ringing smoothly as music fades out.
Music: fade-out `21.5s–23.5s`.

**Music mood for this video:** polished
**Audio summary:** Clean 110 BPM electronic underscore (`vol-12`) anchored by restrained low-HF-risk impacts and UI clicks on key robotics milestones, fading out smoothly beneath a final resonant logo bell.
