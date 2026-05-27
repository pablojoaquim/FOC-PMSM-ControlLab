#!/usr/bin/env python3

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PyLTSpice import RawRead


def parse_command_string(command_string):
    command_string = command_string.replace('\\"', '"')
    return shlex.split(command_string)


def get_ltspice_command():
    custom_command = os.getenv('LTSPICE_CMD')
    if custom_command:
        return parse_command_string(custom_command)

    if sys.platform.startswith('linux'):
        if shutil.which('wine'):
            return ['wine', r'C:\Program Files\LTC\LTSpiceXVII\XVIIx64.exe', '-Run', '-b']

        cmd_exe = shutil.which('cmd.exe')
        if cmd_exe:
            win_paths = [
                r'C:\Users\pablo.joaquim\AppData\Local\Programs\ADI\LTspice\LTspice.exe',
                r'C:\Program Files\LTC\LTSpiceXVII\XVIIx64.exe',
                r'C:\Program Files (x86)\LTC\LTSpiceXVII\XVIIx64.exe',
            ]
            for win_path in win_paths:
                wsl_path = '/mnt/c' + win_path[2:].replace('\\', '/')
                if os.path.exists(wsl_path):
                    return [cmd_exe, '/c', f'"{win_path}" -Run -b']

        raise FileNotFoundError(
            'wine not found and Windows LTSpice executable not available in WSL. '
            'Install wine or set LTSPICE_CMD to a valid LTSpice command. '
            'Example: export LTSPICE_CMD="cmd.exe /c \"C:\\Program Files\\LTC\\LTSpiceXVII\\XVIIx64.exe\" -Run -b"'
        )
    if sys.platform == 'darwin':
        return ['/Applications/LTSpice.app/Contents/MacOS/LTSpice', '-b']

    windows_paths = [
        Path(r'C:\Users\pablo.joaquim\AppData\Local\Programs\ADI\LTspice\LTspice.exe'),
        Path(r'C:\Program Files\LTC\LTSpiceXVII\XVIIx64.exe'),
        Path(r'C:\Program Files (x86)\LTC\LTSpiceXVII\XVIIx64.exe'),
    ]
    for path in windows_paths:
        if path.exists():
            return [str(path), '-Run', '-b']

    return [r'C:\Program Files\LTC\LTSpiceXVII\XVIIx64.exe', '-Run', '-b']


def write_pwl(filename, time, values):
    filename = Path(filename)
    with filename.open('w', newline='\n') as f:
        for t, v in zip(time, values):
            f.write(f'{t:.9e}\t{v:.9e}\n')


def run_bldc_motor_simulation(netlist_path, time, va, vb, vc, ltspice_command=None, verbose=False):
    netlist_path = Path(netlist_path)
    workdir = netlist_path.parent

    if not netlist_path.exists():
        raise FileNotFoundError(f'LTSpice netlist not found: {netlist_path}')

    if ltspice_command is None:
        ltspice_command = get_ltspice_command()
    if isinstance(ltspice_command, str):
        ltspice_command = shlex.split(ltspice_command)

    write_pwl(workdir / 'phaseA.csv', time, va)
    write_pwl(workdir / 'phaseB.csv', time, vb)
    write_pwl(workdir / 'phaseC.csv', time, vc)

    if verbose:
        print('LTSpice command:', ' '.join(ltspice_command + [netlist_path.name]))
        print('Working directory:', workdir)

    if ltspice_command[0].lower().endswith('cmd.exe') and len(ltspice_command) >= 3 and ltspice_command[1].lower() == '/c':
        inner_cmd = ltspice_command[2:] + [str(netlist_path.name)]
        ltspice_command = [ltspice_command[0], ltspice_command[1], subprocess.list2cmdline(inner_cmd)]
    else:
        ltspice_command = ltspice_command + [netlist_path.name]
    subprocess.run(ltspice_command, cwd=workdir, check=True)

    raw_path = workdir / f'{netlist_path.stem}.raw'
    if not raw_path.exists():
        raise FileNotFoundError(f'LTSpice raw output not found: {raw_path}')

    ltr = RawRead(str(raw_path))
    if verbose:
        print('Raw trace names:', ltr.get_trace_names())

    ia = np.array(ltr.get_trace('I(Va)').get_wave(0))
    ib = np.array(ltr.get_trace('I(Vb)').get_wave(0))
    ic = np.array(ltr.get_trace('I(Vc)').get_wave(0))
    time_raw = np.array(ltr.get_trace('time').get_wave(0))

    return {'time': time_raw, 'ia': ia, 'ib': ib, 'ic': ic}
