# MPC_Humanoid

An external Isaac Lab project containing an installable Python package and Isaac Sim extension.

## Installation

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create the project environment:

```bash
uv sync
```

Commit both `pyproject.toml` files and `uv.lock` so collaborators use the same environment.

## Commands

```bash
uv run python scripts/run_env.py --viz kit                    # load the Robonion MPC env and play it (headless without --viz)
uv run python scripts/build_controller.py                     # build the controller (C functions + acados solver), no Isaac
uv run python scripts/run_stepping.py --steps 20                # stage-2 stepping in place (crouch, calibrate, step)
uv run python scripts/run_stepping.py --push x+1.75            # one push mid single support while stepping
uv run python scripts/run_stepping_nocalib.py --viz kit        # spawned crouched, no AHRS calibration (holds: bias > 0.2 deg)
uv run python scripts/benchmark_stepping.py run.npz            # replay a run recorded with run_stepping.py --record, no Isaac
```

## Development

Run formatting and lint checks through the project environment:

```bash
uv run pre-commit run --all-files
```

To configure VS Code, run the `setup_python_env` task or invoke its command directly:

```bash
uv run python .vscode/tools/setup_vscode.py
```

## Troubleshooting

If Pylance cannot resolve simulator modules, run the VS Code setup command above and reload the window. If indexing uses
too much memory, remove unused simulator extension paths from `.vscode/settings.json`.
