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
    """
    Parse command string into list, handling Windows paths with spaces.
    
    For Windows commands with spaces in paths, it's safer to pass as a direct list.
    This function attempts basic parsing but is best used with simple commands.
    """
    command_string = command_string.replace('\\"', '"')
    return shlex.split(command_string, posix=False)


def get_ltspice_command(command_override=None):
    """
    Get the LTSpice command to execute.
    
    Args:
        command_override: Optional override command (list or string). If provided, use this instead of environment or platform detection.
    
    Returns:
        List representing the LTSpice command.
    """
    if command_override:
        return command_override if isinstance(command_override, list) else parse_command_string(command_override)

    custom_command = os.getenv('LTSPICE_CMD')
    if custom_command:
        return parse_command_string(custom_command)

    # Try to find ltspice.exe or ltspice in PATH
    ltspice_exe = shutil.which('ltspice') or shutil.which('ltspice.exe')
    if ltspice_exe:
        return [ltspice_exe, '-Run', '-b']

    # Platform-specific fallbacks
    if sys.platform.startswith('linux'):
        wine = shutil.which('wine')
        if wine:
            return [wine, 'ltspice.exe', '-Run', '-b']

    raise FileNotFoundError(
        'LTSpice not found. Please install LTSpice and ensure it is in PATH, '
        'or set LTSPICE_CMD environment variable. '
        'Examples:\n'
        '  export LTSPICE_CMD="/path/to/ltspice -Run -b"\n'
        '  export LTSPICE_CMD="wine ltspice.exe -Run -b"'
    )


def write_pwl(filename, time, values):
    filename = Path(filename)
    with filename.open('w', newline='\n') as f:
        for t, v in zip(time, values):
            f.write(f'{t:.9e}\t{v:.9e}\n')


def write_stimulus_files(workdir, stimuli):
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)

    for filename, (time, values) in stimuli.items():
        file_path = workdir / filename
        write_pwl(file_path, time, values)


def format_ltspice_command(raw_command, netlist_name):
    if isinstance(raw_command, str):
        raw_command = parse_command_string(raw_command)

    if raw_command[0].lower().endswith('cmd.exe') and len(raw_command) >= 3 and raw_command[1].lower() == '/c':
        inner_cmd = raw_command[2:] + [str(netlist_name)]
        return [raw_command[0], raw_command[1], subprocess.list2cmdline(inner_cmd)]

    return raw_command + [str(netlist_name)]


def run_ltspice_simulation(netlist_path, stimuli=None, ltspice_command=None, verbose=False):
    netlist_path = Path(netlist_path)
    if not netlist_path.exists():
        raise FileNotFoundError(f'LTSpice netlist not found: {netlist_path}')

    workdir = netlist_path.parent
    if stimuli:
        write_stimulus_files(workdir, stimuli)

    if ltspice_command is None:
        ltspice_command = get_ltspice_command()

    ltspice_command = format_ltspice_command(ltspice_command, netlist_path.name)

    if verbose:
        print('LTSpice command:', ' '.join(str(c) for c in ltspice_command))
        print('Working directory:', workdir)

    subprocess.run(ltspice_command, cwd=workdir, check=True)

    raw_path = workdir / f'{netlist_path.stem}.raw'
    if not raw_path.exists():
        raise FileNotFoundError(f'LTSpice raw output not found: {raw_path}')

    raw_reader = RawRead(str(raw_path))
    trace_names = raw_reader.get_trace_names()
    traces = {name: np.array(raw_reader.get_trace(name).get_wave(0)) for name in trace_names}

    if verbose:
        print('Raw trace names:', trace_names)

    return {
        'raw_path': raw_path,
        'raw': raw_reader,
        'trace_names': trace_names,
        'traces': traces,
    }
