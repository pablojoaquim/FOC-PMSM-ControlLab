# BLDC FOC Simulation with LTSpice and Python

This repository contains a demonstration of brushless DC motor simulation using LTSpice and field-oriented control (FOC) implemented in Python.
The Python script generates three-phase control waveforms, executes the LTSpice motor model, and reads phase currents back from the LTSpice `.raw` output using `PyLTSpice`.

## Prerequisites

- Python 3.10+
- LTSpice installed on Windows (or Wine + LTSpice on Linux/macOS)
- `PyLTSpice`, `numpy`, `scipy`, and `matplotlib`

## Install

From the project root:

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run the simulation

From the project root:

```powershell
python src/main.py
```

This will:
- generate FOC phase voltage commands
- run `ltspice/motor_model.cir`
- read phase currents from the LTSpice raw output
- plot voltages, currents, and dq-axis currents
- save `foc_simulation.png`

## LTSpice path configuration

If LTSpice is installed in a different location, set the environment variable:

```powershell
setx LTSPICE_CMD '"C:\Users\pablo.joaquim\AppData\Local\Programs\ADI\LTspice\LTspice.exe" -Run -b'
```

Then reopen PowerShell and rerun the script.
### Running from WSL

If you are running the Python script from WSL and do not have `wine` installed, you can use the Windows LTSpice executable via `cmd.exe`:

```bash
export LTSPICE_CMD='cmd.exe /c "C:\Users\pablo.joaquim\AppData\Local\Programs\ADI\LTspice\LTspice.exe" -Run -b'
python3 src/main.py
```

Alternatively, install wine in WSL:

```bash
sudo apt update
sudo apt install wine64
```
