# Hyperframes Composition Brief: CapSorter

## Objective
Create a 23.5-second, 1920×1080 (30fps) polished launch video explaining the full CapSorter open-source GitHub repository (`github.com/Neilboss/CapSorter` / `capsorter.vercel.app`).

## Output
- Composition directory: `brag-output/composition/`
- Rendered video: `brag-output/brag.mp4`
- Format: landscape — 1920×1080 @ 30fps
- Duration: 23.5s (705 frames)

## Source Material
- Project root: `c:\Users\shett\Desktop\CapSorter`
- Primary files read: `README.md`, `website/index.html`, `website/css/style.css`, `firmware/README.md`, `firmware/HARDWARE_BRINGUP_GUIDE.md`, `research_paper/main.tex`
- Product name: CapSorter (Autonomous Bottle-Cap Sorting Machine)
- Tagline / strongest claim: 100% offline dual-controller Cartesian pick-and-place robot with pipelined bin-side re-imaging (`100% completion vs. 67.5% open-loop`)
- Key UI or visual moment to recreate: CapSorter V2 Interactive 2D Closed-Loop Simulator (continuous 300×300 mm open plywood workspace, touching red pair, `+32 mm` neighbor bump, `CMD_SCAN_WINDOW_OPEN (0xA5)` bin-side scan re-acquisition, live telemetry)
- Copy that must appear verbatim:
  - `500 × 500 × 500 MM  •  100% OFFLINE  •  ZERO CLOUD`
  - `Scatter bottle caps anywhere.`
  - `CapSorter sees, adapts, and sorts every last one.`
  - `CMD_SCAN_WINDOW_OPEN (0xA5)`
  - `100% Completion vs. 67.5% Open-Loop`
  - `github.com/Neilboss/CapSorter`
  - `capsorter.vercel.app`
  - `By Neil Shettigar, Kahan & Shourya`

## Creative Direction
- Tone preset: `polished`
- Creative direction: cinematic industrial hardware & embedded robotics product film
- Interpretation: Dark industrial engineering precision, high-contrast WCAG AA/AAA text, snappy entrances with long settled reading holds, real CAD visuals, and live working simulator choreography.
- Avoid: Generic SaaS language, abstract filler visuals, unverified claims (no ISR claims, no galvanic isolation claims, no "production-ready" claims).

## Visual Identity
- Background: `#0a0d14` (surfaces `#111622`, `#161e30`; engineering grid `rgba(255,255,255,0.03)`)
- Text: `#f8fafc` primary, `#cbd5e1` secondary, `#94a3b8` muted (all verified for WCAG contrast)
- Accents: `#ff6b00` (V1 orange), `#00d2ff` (V2 cyan), `#10b981` (emerald), `#f59e0b` (amber)
- Display font: `Outfit` (Google Fonts / sans-serif fallback)
- Body / Mono font: `Inter` / `JetBrains Mono`
- Visual references copied into `composition/assets/img/`: `portal_robot_system.png`, `gantry_3axis_cad.png`, `logo.png`

## Storyboard
Use `brag-output/brag-plan.md` as the creative contract:
1. Scene 1 — Hardware Hook (`0.00s–4.30s`)
2. Scene 2 — Dual-Controller Architecture V1 vs. V2 (`4.30s–8.74s`)
3. Scene 3 — Live Closed-Loop Bin-Side Re-Imaging Simulator (`8.74s–15.29s`)
4. Scene 4 — Complete Open-Source GitHub Repo (`15.29s–19.66s`)
5. Scene 5 — Brand Outro & Repository Links (`19.66s–23.50s`)

## Audio
- Audio role: clean steady underscore with restrained motion-matched SFX
- Music: `assets/music/happy-beats-business-moves-vol-12-by-ende-dot-app.mp3` (`data-volume="0.32"`, `data-fade-in="0.4"`, `data-fade-out="2.0"`)
- Music cue guidance: Bundled preset `happy-beats-business-moves-vol-12-by-ende-dot-app.music-cues.md` (109.96 BPM). Strong cues locked at `8.74s`, `13.11s`, `17.47s`; beat-grid entrances at `0.56s`, `1.64s`, `4.91s`, `6.00s`, `7.09s`, `15.84s`, `16.38s`, `16.93s`, `17.47s`, `20.75s`.
- Audio-reactive treatment: subtle beat-synchronized breathing on ambient radial glow and scan-pulse accents (deterministic GSAP pulse aligned to 109.96 BPM beat grid; no equalizer bars).
- Selected low/medium HF-risk SFX in `composition/assets/sfx/`:
  - `impact/impactSoft_medium_001.ogg` (`0.52s`, hook headline)
  - `casino/card-slide-1.ogg` (`4.85s`, `5.95s`, V1/V2 card arrivals)
  - `interface/click_003.ogg` (`10.10s`, gripper clamp)
  - `interface/bong_001.ogg` (`11.95s`, `CMD_SCAN_WINDOW_OPEN` scan flash)
  - `impact/impactSoft_medium_004.ogg` (`13.05s`, Bin 1 deposit at `13.11s` strong cue)
  - `interface/click_002.ogg` (`15.80s`, `16.90s`, repo cards)
  - `impact/impactSoft_medium_002.ogg` (`17.42s`, 4th repo card at `17.47s` strong cue)
  - `impact/impactBell_heavy_000.ogg` (`19.70s`, outro logo landing)
