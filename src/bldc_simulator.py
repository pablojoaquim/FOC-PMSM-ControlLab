#!/usr/bin/env python3
"""
BLDC motor simulation wrapper using the generic LTSpice driver.
"""

from pathlib import Path

import numpy as np

from ltspice_driver import run_ltspice_simulation


def run_bldc_motor_simulation(netlist_path, time, va, vb, vc, ltspice_command=None, verbose=False):
    """
    Simulate a BLDC motor with three-phase voltage inputs.

    Args:
        netlist_path: Path to the LTSpice netlist file.
        time: Time array for the simulation.
        va: Phase A voltage array.
        vb: Phase B voltage array.
        vc: Phase C voltage array.
        ltspice_command: Optional custom LTSpice command (list or string).
        verbose: Print diagnostic information.

    Returns:
        Dictionary with 'time', 'ia', 'ib', 'ic' arrays.
    """
    stimuli = {
        'phaseA.csv': (time, va),
        'phaseB.csv': (time, vb),
        'phaseC.csv': (time, vc),
    }
    result = run_ltspice_simulation(
        netlist_path,
        stimuli=stimuli,
        ltspice_command=ltspice_command,
        verbose=verbose
    )
    traces = result['traces']

    return {
        'time': traces['time'],
        'ia': traces['I(Va)'],
        'ib': traces['I(Vb)'],
        'ic': traces['I(Vc)'],
    }
