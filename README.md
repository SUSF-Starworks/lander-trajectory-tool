# Lander Trajectory Tool

A desktop GUI for the [`vtvl-sim`](https://github.com/dfps16/vtvl-sim) planar VTVL lander
simulation engine, built for SUSF Starworks as a preliminary design tool. It informs key
parameters such as tank sizing and throttle profiles by letting you tune vehicle
parameters, controller gains, and flight phases, then inspect the resulting trajectory,
state, engine, and propellant plots.

![Cascaded PD, Altitude PID, Attitude PD, and LQR controllers supported](https://img.shields.io/badge/controllers-4-01788D)

## Features

- Configure vehicle parameters, initial state, and multi-phase waypoints
- Switch between the engine's controllers (Cascaded PD, Altitude PID, LQR, and an
  Attitude PD inner-loop demo), each with its own gain panel
- Run the simulation and review trajectory, state, engine, and propellant plots side by
  side, with summary metrics (touchdown accuracy, propellant usage, flameout detection)
- Optional descent animation
- Save/load scenarios as JSON, compatible with `vtvl-sim`'s own solver format

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
