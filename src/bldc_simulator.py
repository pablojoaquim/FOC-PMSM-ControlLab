# ******************************************************************************
#
# @file bldc_simulator.py
#
# ******************************************************************************
# @copyright Copyright (c) 2026 - Pablo Joaquim
#             MIT License: https://opensource.org/licenses/MIT
# ******************************************************************************
#
# @section DESC DESCRIPTION:
#   BLDC simulation wrapper that prepares three-phase stimuli for LTSpice.
#
# @section ABBR ABBREVIATIONS:
#   - BLDC: Brushless DC motor.
#   - LTSpice: Linear Technology SPICE simulator.
#
# @section TRACE TRACEABILITY INFO:
#   - Design Document(s):
#     - doc/foc_motor_control_technical.md
#
#   - Requirements Document(s):
#     - README.md
#
#   - Applicable Standards (in order of precedence: highest first):
#     - MIT License
#
# ******************************************************************************


# ******************************************************************************
# * import modules
# ******************************************************************************
from pathlib import Path

import numpy as np

from ltspice_driver import run_ltspice_simulation


# ******************************************************************************
# * Objects Declarations
# ******************************************************************************


# ******************************************************************************
# * Object and variables Definitions
# ******************************************************************************


# ******************************************************************************
# * Function Definitions
# ******************************************************************************
# ******************************************************************************
# * @fn         run_bldc_motor_simulation
# * @brief      Simulate BLDC phase currents from three-phase voltage stimuli.
# * @param [in] netlist_path - Path to the LTSpice netlist file.
# * @param [in] time - Timebase array for the simulation.
# * @param [in] va - Phase A voltage waveform.
# * @param [in] vb - Phase B voltage waveform.
# * @param [in] vc - Phase C voltage waveform.
# * @param [in] ltspice_command - Optional LTSpice command override.
# * @param [in] verbose - Enables diagnostic printing when True.
# * @return     Dictionary with time and phase current arrays.
# ******************************************************************************
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
