# Lander Trajectory Tool

A desktop GUI for the [`vtvl-sim`](https://github.com/dfps16/vtvl-sim) planar VTVL lander
simulation engine, built for SUSF Starworks as a preliminary design tool. It informs key
parameters such as tank sizing and throttle profiles by letting you tune vehicle
parameters, controller gains, and flight phases, then inspect the resulting trajectory,
state, engine, and propellant plots.

![Cascaded PD, Altitude PID, Attitude PD, and LQR controllers supported](https://img.shields.io/badge/controllers-4-01788D)

## Feature Summary

- Configure vehicle parameters, initial state, and multi-phase waypoints
- Switch between the engine's controllers (Cascaded PD, Altitude PID, LQR, and an
  Attitude PD inner-loop demo), each with its own gain panel
- Run the simulation and review trajectory, state, engine, and propellant plots side by
  side, with summary metrics (touchdown accuracy, propellant usage, flameout detection)
- Optional descent animation
- Save/load scenarios as JSON, compatible with `vtvl-sim`'s own solver format

## Planned Features

- Interactive Plotly plots in place of the current static images
- O/F ratio tracking for exact fuel/oxidizer consumption, rather than a single
  propellant mass
- Gain sweeps / propellant usage studies across a range of scenarios
- Rework the State tab's plot grid (currently 3×2) into 2×3

## Install

Requires [uv](https://docs.astral.sh/uv/) and [git](https://git-scm.com/downloads)

### Quick Start
```bash
uv tool install git+https://github.com/SUSF-Starworks/lander-trajectory-tool.git
lander-trajectory-tool
```
This installs a standalone `lander-trajectory-tool` command. Run `lander-trajectory-tool` any time from your terminal/PowerShell.
Update it with `uv tool upgrade lander-trajectory-tool` whenever a new version is released.

### Extras
By default the app opens in its own desktop window. Set `VTVL_NATIVE=0` to run it as a
local web server and open it in a browser instead:

```bash
VTVL_NATIVE=0 lander-trajectory-tool
```

### Optional: MP4 animations
Descent animations (the **Generate animation** checkbox) render as MP4 if
[ffmpeg](https://ffmpeg.org/download.html) is on your `PATH`, and fall back to an
animated GIF otherwise. Not required for anything else.

```bash
brew install ffmpeg          # macOS
sudo apt install ffmpeg      # Debian/Ubuntu
winget install ffmpeg        # Windows
```

## Usage

1. **Set up the vehicle and flight.** Fill in *Vehicle parameters* (dry mass, inertia,
   moment arm, thrust limits, Isp, gimbal/tilt limits) and *Initial state* (position,
   velocity, attitude, and wet mass (must exceed dry mass of course)). Add one or more entries
   under *Phases*, each a waypoint (`x_target`, `z_target`) to reach by `t_end`.
2. **Pick a controller.** Each has its own gain panel in the *Controller* card:
   - **Cascaded PD** / **LQR** are the complete ones, these can handle full mission profiles.
     - The Cascaded PD is made of three PD controllers and tuned as such.
     - LQR has a more complex tuning process, and the current default gains are not optimal. You are welcome to tweak them around.
   - **Attitude PD (inner-loop demo)** — holds a commanded pitch (the θ target field,
     which only appears for this controller) rather than landing. This is a gimbal-only
     inner-loop demo
   - **Altitude PID** — a simple altitude controller, useful for testing vertical ascent/descent only
3. **Run.** Click **Run** to simulate. Tick **Generate animation** first if you also
   want a rendered descent video/GIF (it adds noticeably to the run time, and the mp4 video needs ffmpeg).
4. **Read the results**, across five tabs:
   - **State** — position/rate/attitude time histories, peak excursions, final state
   - **Trajectory** — flight path and touchdown outcome (landed / flameout — propellant
     exhausted / no touchdown — ran out of simulated time), plus propellant usage split
   - **Engine** — commanded vs. applied thrust, throttle envelope
   - **Propellant** — mass depletion over time, with a cross-check between the solver's
     actual mass loss and the post-hoc thrust integral
   - **Animation** — populated only if it was enabled before running
5. **Save/load scenarios.** *Save scenario* downloads the current fields as a JSON file
   in `vtvl-sim`'s solver format, *Load scenario* re-populates every field from one.

## Development

```bash
git clone https://github.com/SUSF-Starworks/lander-trajectory-tool.git
cd lander-trajectory-tool
uv run lander-trajectory-tool
```

`uv run` uses the project's own virtual environment (`uv sync` to set it up explicitly).
Set `VTVL_RELOAD=1` alongside `VTVL_NATIVE=0` to auto-reload the browser view on source
changes while iterating.

## License

MIT — see [LICENSE](LICENSE).
