# AURA-LIFT: Electrical-Signature Diagnostics for Elevator Drives

**Team Immortal** · KONE Elevate Hackathon 2026 · **Winner (1 of 5 teams)**
Houshik R · Joshitha V G · Charunethra S, Kalaignar Karunanidhi Institute of Technology, Coimbatore

> **Question:** what can voltage and current alone prove about the health of an elevator drive?
> Every drive already measures both signals, so a diagnostic built on them deploys as software, with nothing added to the machine.

## What is built and what is simulated

| Layer | Status | What it is |
| --- | --- | --- |
| **Hardware bench rig** | Built, measured | ESP32 + ACS712 current sensor + voltage sensor + BTS7960 H-bridge driving a counterweighted DC-motor "elevator". Splits every trip into Idle / Start / Accel / Cruise / Decel / Level, pairs UP and DOWN runs, and serves a Wi-Fi dashboard with CSV export. |
| **Fault simulations** | Simulated | Six fault models on one 400 V elevator drive (capacitor ageing, inverter switch fault, rectifier phase loss, overcurrent, DC-bus overvoltage, supply sag). The fault edits the circuit and the solver produces the waveform. |

The two layers are validated separately today. Linking them is the next step.

## Result: an evidence map, not a detection claim

| Fault | Cause identified from V and I alone? |
| --- | --- |
| Capacitor ageing | Yes |
| DC-bus overvoltage | Yes |
| Rectifier phase loss | Partly |
| Supply sag | Partly |
| Inverter switch fault | No |
| Overcurrent | No |

All six faults are detectable. Only two can be traced to a cause, and the project shows where the evidence stops for the rest.

## Limits 

- The rig is a **12 V DC proxy**, not a scaled elevator drive.
- At 12 V the cruise currents are only tens of milliamps, and readings quantise near 0.01 A. The UP/DOWN separation shows the **method runs on real measurements**, not a calibrated load figure.
- No fault has been injected into physical hardware yet. Every fault waveform comes from a physics model.
- No accuracy, detection-rate or false-alarm figure is claimed, because none was measured.
- No encoder is fitted (speed reads "n/a") and the build has no fuse.

## My contribution (Joshitha)

- Idea and the project presentation.
- **Built the hardware rig on my own**: wiring, sensing chain and bench testing.
- Built the first simulation (DC-link capacitor lab), then **directed** the other five, which were AI-generated from my specifications.
- The ESP32 firmware was AI-assisted.

## Repository contents

```
docs/       Project presentation and bench-rig guide
hardware/   Wiring diagrams (12 V side and breadboard side)
firmware/   ESP32 sketch (Arduino core 3.x)
```

## Run the bench rig

1. Copy `firmware/aura_lift_firmware/` into your Arduino sketch folder.
2. Set `WIFI_SSID` and `WIFI_PASSWORD` in the sketch, or leave them empty to use the built-in hotspot.
3. Plug in USB, wait about 5 seconds, then plug in the 12 V adapter. The order matters.
4. Join the Wi-Fi `AURA-LIFT` and open `http://192.168.4.1`.
5. **Unplug the 12 V adapter before stopping, uploading or resetting.**

Full details: [`docs/bench-rig-guide.docx`](docs/bench-rig-guide.docx).

## Simulations

- [AURA-LIFT Fault Simulation Suite](https://claude.ai/artifact/BTXu8A7y76FC132QK7H6j7)
- [DC-Link Capacitor Lab](https://claude.ai/artifact/3S5nBWjyqjyChbmCHUqApi)
