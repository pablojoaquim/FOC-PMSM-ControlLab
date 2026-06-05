# ******************************************************************************
#
# @file main.py
#
# ******************************************************************************
# @copyright Copyright (c) 2026 - Pablo Joaquim
#             MIT License: https://opensource.org/licenses/MIT
# ******************************************************************************
#
# @section DESC DESCRIPTION:
#   Entry point for the FOC PMSM Control Lab closed-loop simulation.
#
# @section ABBR ABBREVIATIONS:
#   - FOC: Field-Oriented Control.
#   - PMSM: Permanent Magnet Synchronous Motor.
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
import matplotlib.pyplot as plt
from config import load_config_with_paths
from foc_controller import FOCController, run_foc_closed_loop_simulation


# ******************************************************************************
# * Objects Declarations
# ******************************************************************************


# ******************************************************************************
# * Object and variables Definitions
# ******************************************************************************


# ******************************************************************************
# * Function Definitions
# ******************************************************************************
if __name__ == '__main__':
    # Load configuration and project paths
    config, paths = load_config_with_paths()

    netlist_path = config['_netlist_path_resolved']
    duration = config['simulation']['duration']
    timestep = config['simulation']['timestep']

    print('Preparing BLDC FOC simulation...')

    # Initialize FOC controller
    controller = FOCController(config)

    # Run closed-loop simulation
    verbose = config['simulation']['verbose']
    results = run_foc_closed_loop_simulation(
        controller,
        netlist_path,
        duration,
        timestep,
        config['ltspice'],
        verbose=verbose
    )

    # Extract results
    sim_time = results['sim_time']
    ia = results['ia']
    ib = results['ib']
    ic = results['ic']
    id_current = results['id_current']
    iq_current = results['iq_current']
    phase_a_sim = results['phase_a_sim']
    phase_b_sim = results['phase_b_sim']
    phase_c_sim = results['phase_c_sim']

    # Plot results
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(sim_time * 1e3, phase_a_sim, label='Va')
    plt.plot(sim_time * 1e3, phase_b_sim, label='Vb')
    plt.plot(sim_time * 1e3, phase_c_sim, label='Vc')
    plt.title('FOC Voltage Commands (closed-loop control)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(sim_time * 1e3, ia, label='Ia')
    plt.plot(sim_time * 1e3, ib, label='Ib')
    plt.plot(sim_time * 1e3, ic, label='Ic')
    plt.title('Simulated Phase Currents')
    plt.ylabel('Current (A)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(sim_time * 1e3, id_current, label='id')
    plt.plot(sim_time * 1e3, iq_current, label='iq')
    plt.title('Park Transform of Measured Currents')
    plt.xlabel('Time (ms)')
    plt.ylabel('Current (A)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(paths['project_root'] / 'foc_simulation.png')
    plt.show()

    print('Simulation complete. Results saved to foc_simulation.png')
