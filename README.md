# RoboticArm Simulation Example

This repository models a simple two-joint robotic arm (shoulder and elbow) in
[OpenModelica](https://openmodelica.org/). The Python scripts around it help you
explore the servo trade space: choosing a real off-the-shelf servo for each
joint while balancing cost against how quickly the arm completes its motion.

You do not need prior experience with Modelica or Python to use this project.
This guide walks you through installation, setup, simulation, and plotting in a
straightforward way.

---

## 1. Prerequisites

You need four things installed on your machine:

1. **Visual Studio Code** — recommended for editing files and running commands
   from the integrated terminal. Install it from
   [code.visualstudio.com](https://code.visualstudio.com/).

2. **Python 3.10+** — check the installation with:
   ```
   python --version
   ```
   If that fails, install Python from [python.org](https://www.python.org/downloads/).
   On Windows, make sure you tick "Add python.exe to PATH" during installation.

3. **OpenModelica** — required to run new simulations.
   - **Windows:** install from [openmodelica.org](https://openmodelica.org/)
     and keep the default settings.
   - **macOS:** native OpenModelica builds were discontinued; use Docker
     instead (see [macOS setup](#macos-openmodelica-via-docker) below).

4. **Git** — needed to clone the repository and track changes. Install it from
   [git-scm.com](https://git-scm.com/downloads/) and accept the default options
   during setup.

---

## macOS: OpenModelica via Docker

On Mac, run OpenModelica inside Docker. That gives you both the GUI
(`OMEdit`) and the command-line compiler (`omc`) that this project's Python
scripts call.

1. Install and start [Docker Desktop](https://www.docker.com/products/docker-desktop/).

2. For the OpenModelica GUI, also install [XQuartz](https://www.xquartz.org/).
   Open XQuartz, go to **Preferences → Security**, enable **Allow connections
   from network clients**, then quit and reopen XQuartz. In a terminal run:
   ```bash
   xhost +localhost
   ```

3. Add the official `docker-om` helper (from
   [OpenModelica's Docker page](https://openmodelica.org/download/docker/)):
   ```bash
   echo $'alias docker-om=\'docker run -it --rm -v "$HOME:$HOME" -e "HOME=$HOME" -w "$PWD" -e "DISPLAY=`ifconfig | grep -o "inet [0-9.]*" | grep -Eo "[0-9.]{7,}" | grep -Fv 127.0.0.1 | head -1`:0" --user $UID openmodelica/openmodelica:v1.27.0-gui\'' >> "$HOME/.zshrc"
   source "$HOME/.zshrc"
   ```

4. Pull the image and check that `omc` works:
   ```bash
   docker pull openmodelica/openmodelica:v1.27.0-gui
   docker-om omc --version
   ```

5. Expose `omc` on your PATH so the Python scripts can find it. Create a small
   wrapper (once). The lines between `EOF` markers must start at column 0:
   ```bash
   mkdir -p "$HOME/bin"
   cat > "$HOME/bin/omc" <<'EOF'
#!/bin/bash
docker run --rm -v "$HOME:$HOME" -e "HOME=$HOME" -w "$PWD" --user "$UID" \
  openmodelica/openmodelica:v1.27.0-gui omc "$@"
EOF
   chmod +x "$HOME/bin/omc"
   echo 'export PATH="$HOME/bin:$PATH"' >> "$HOME/.zshrc"
   source "$HOME/.zshrc"
   omc --version
   ```

6. Open the model GUI from the repository folder:
   ```bash
   cd /path/to/RoboticArm_example
   docker-om OMEdit
   ```
   In OMEdit, open `Simulation/RoboticArm.mo`, then simulate the default
   example as in step 3 below.

After that, the same Python commands in steps 4–6 work on Mac as on Windows
(`python simulate_one.py`, `python simulate_all.py`, and so on), as long as
Docker Desktop is running.

---

## 2. Setup

Open Visual Studio Code, then open a terminal and run the following commands from
the folder where you want to store the project:

```bash
git clone https://github.com/PatrykPrezyna/RoboticArm_example.git
cd RoboticArm_example
python3 -m venv .venv
```

Activate the virtual environment:

```bash
# Windows PowerShell
.\.venv\Scripts\Activate.ps1
# Windows Command Prompt
.\.venv\Scripts\activate
# macOS / Linux
source .venv/bin/activate
```

Then install dependencies:

```bash
pip install -r requirements.txt
```

---

## 3. Open Modelica and run the default parameters first

Open [Simulation/RoboticArm.mo](Simulation/RoboticArm.mo) in OpenModelica
(OMEdit on Windows, or `docker-om OMEdit` on macOS).
From there, load the model and run the default example to confirm that the
simulation works before moving on to the Python workflow.

The images in the repository show the expected workflow in the OpenModelica GUI:

![Run Modelica example](Run%20Modelica%20example.png)

![Plot in modellica](Plot%20in%20modellica.png)

This first run is useful because it gives you a quick sanity check that the model
loads correctly and that your OpenModelica installation is working.

---

## 4. Run the simulation for one default setup

You can also run the model from Python with the default settings:

```bash
cd Simulation
python simulate_one.py
```

This runs the simulation once and writes a result row to the output folder.
The script uses the default parameter values defined by the project and produces
a timestamped CSV file in the output directory, named like
`sim_result_YYYYMMDD_HHMMSS.csv`.

If you want to try a different value, you can override individual parameters from
the command line. For example:

```bash
python simulate_one.py --shoulder-servo-tau-stall 0.92 --elbow-servo-tau-stall 5.88
```

---

## 5. Run the simulation for combinations of setups

The project also includes a JSON-based input file in [Simulation](Simulation/),
which makes it easier to run one configuration or many combinations.

Run a single configuration first:

```bash
python simulate_one.py
```

Then run a sweep over every combination defined in the JSON file:

```bash
python simulate_all.py full_input.json
```

The example sweep file is [Simulation/full_input.json](Simulation/full_input.json).
Its `servo_options` lists must match keys in `servo_catalog` (currently
`AD002`, `AD004`, `MG996R`, and `DS3218MG` for both joints).

The simulation-all script explores the Cartesian product of the candidate values
and writes its results to a timestamped CSV file in the `output` folder, named
like `sim_result_YYYYMMDD_HHMMSS.csv`.

---

## 6. Generate an early trade space without Modelica

The early trade-space workflow uses the component options in
[Simple_tradespace/inout.json](Simple_tradespace/inout.json). It enumerates all
combinations and calculates total component cost and mass without running a
simulation:

```bash
# Run from the repository root
python plot_tradespace.py early
```

This writes a timestamped CSV, the main cost-versus-mass plot, and a multi-panel
decision plot to `output`. The decision figure colors the concepts separately
by user interface, battery, servo motor, and controller. Every plotted point has
a concept ID (`C01`, `C02`, and so on), and the terminal prints the complete
configuration of each Pareto-optimal concept. The servo motor has `quantity: 5`,
so its mass and cost account for the wrist, gripper, base, elbow, and shoulder
actuators.

To use a different input or add labels to every candidate point:

```bash
python plot_tradespace.py early path/to/components.json --labels
```

## 7. Generate a simulation trade space

Once you have sweep results, you can generate a trade-space plot:

```bash
# Run from the repository root
python plot_tradespace.py simulation
```

This uses the most recently created result CSV in the `output` folder and writes
a timestamped plot there. The plot shows servo cost on the horizontal axis and
motion time on the vertical axis. Servo prices are read from
[Simulation/full_input.json](Simulation/full_input.json), so the same catalog
drives both the simulation sweep and its plot.

If you want to save the plot under a different name, use:

```bash
python plot_tradespace.py simulation --output output/my_plot.png
```

The old `python plot_tradespace.py` command still works when run from inside
the `Simulation` folder.

---

## 8. Troubleshooting

**"OpenModelica 'omc' not found"** — the scripts could not locate the
OpenModelica compiler. Either:

- **Windows:** reinstall OpenModelica and let the setup finish completely, or
  find your OpenModelica install folder and add its `bin` subfolder to your
  `PATH` (or set `OPENMODELICAHOME`).
- **macOS:** make sure Docker Desktop is running, and that the `omc` wrapper
  from the [macOS setup](#macos-openmodelica-via-docker) section is on your
  `PATH` (`which omc` should print something like `/Users/you/bin/omc`).

**A plot or result file looks stale** — delete the output folder and run the
simulation again. The project regenerates its outputs each time.

**`ModuleNotFoundError: No module named 'matplotlib'`** — run
`pip install -r requirements.txt` from the repository root.

---

## 9. Git basics (if you have never used Git before)

Git tracks changes to this project over time. A few commands cover most of what
you will need day to day. Run them from inside the repository folder in a
terminal:

| Command | What it does |
|---|---|
| `git status` | Shows what has changed since the last save point (commit) |
| `git add <file>` | Stages a file so it is included in the next commit |
| `git commit -m "message"` | Saves a snapshot of the staged changes with a short description |
| `git pull` | Downloads and merges changes from GitHub |
| `git push` | Uploads your commits to GitHub |
| `git log --oneline` | Shows the history of commits |

A typical workflow after changing a file, for example a sweep input, looks like
this:

```bash
git status
git add Simulation/full_input.json
git commit -m "Try a wider servo sweep"
git push
```

If you are not sure whether something is safe to run, `git status` is always a
safe first step because it only shows information and does not change anything.
