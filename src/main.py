from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from ltspice_driver import run_bldc_motor_simulation

SQRT3 = np.sqrt(3)


def inverse_park(vd, vq, theta):
    valpha = vd * np.cos(theta) - vq * np.sin(theta)
    vbeta = vd * np.sin(theta) + vq * np.cos(theta)
    return valpha, vbeta


def inverse_clarke(valpha, vbeta):
    va = valpha
    vb = -0.5 * valpha + SQRT3 / 2.0 * vbeta
    vc = -0.5 * valpha - SQRT3 / 2.0 * vbeta
    return va, vb, vc


def abc_to_alpha_beta(ia, ib, ic):
    alpha = ia
    beta = (ia + 2.0 * ib) / SQRT3
    return alpha, beta


def alpha_beta_to_dq(alpha, beta, theta):
    id_current = alpha * np.cos(theta) + beta * np.sin(theta)
    iq_current = -alpha * np.sin(theta) + beta * np.cos(theta)
    return id_current, iq_current


if __name__ == '__main__':
    project_root = Path(__file__).resolve().parent.parent
    netlist_file = project_root / 'ltspice' / 'motor_model.cir'

    print('Preparing BLDC FOC simulation...')

    duration = 0.02
    timestep = 50e-6
    time = np.arange(0.0, duration + timestep / 2, timestep)

    pole_pairs = 4
    mechanical_rpm = 1200.0
    mechanical_omega = mechanical_rpm * 2.0 * np.pi / 60.0
    electrical_omega = pole_pairs * mechanical_omega
    rotor_angle = electrical_omega * time

    vd_ref = 0.0
    vq_ref = 7.5
    valpha, vbeta = inverse_park(vd_ref, vq_ref, rotor_angle)
    phase_a, phase_b, phase_c = inverse_clarke(valpha, vbeta)

    print('Running LTSpice motor model...')
    results = run_bldc_motor_simulation(netlist_file, time, phase_a, phase_b, phase_c, verbose=True)

    ia = results['ia']
    ib = results['ib']
    ic = results['ic']
    sim_time = results['time']

    # compute rotor electrical angle on LTSpice timebase (sim_time)
    rotor_angle_sim = electrical_omega * sim_time

    alpha, beta = abc_to_alpha_beta(ia, ib, ic)
    id_current, iq_current = alpha_beta_to_dq(alpha, beta, rotor_angle_sim)

    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time * 1e3, phase_a, label='Va')
    plt.plot(time * 1e3, phase_b, label='Vb')
    plt.plot(time * 1e3, phase_c, label='Vc')
    plt.title('FOC Voltage Commands')
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
    plt.savefig(project_root / 'foc_simulation.png')
    plt.show()

    print('Simulation complete. Results saved to foc_simulation.png')
