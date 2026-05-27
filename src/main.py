from pathlib import Path
import json

import numpy as np
import matplotlib.pyplot as plt
from bldc_simulator import run_bldc_motor_simulation
from ltspice_driver import get_ltspice_command

SQRT3 = np.sqrt(3)


def load_config(config_path):
    with open(config_path, 'r') as f:
        return json.load(f)


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
    config_path = project_root / 'config.json'
    config = load_config(config_path)

    netlist_path = project_root / config['ltspice']['netlist_path']
    duration = config['simulation']['duration']
    timestep = config['simulation']['timestep']
    verbose = config['simulation']['verbose']

    pole_pairs = config['motor']['pole_pairs']
    mechanical_rpm = config['motor']['mechanical_rpm']

    foc = config['foc_controller']
    control_period = foc['control_period']
    Kp_d = foc['kp_d']
    Ki_d = foc['ki_d']
    Kp_q = foc['kp_q']
    Ki_q = foc['ki_q']
    id_ref = foc['id_ref']
    iq_ref = foc['iq_ref']
    vd_ref = foc['vd_ref_init']
    vq_ref = foc['vq_ref_init']
    integral_clamp = foc['integral_clamp']
    voltage_clamp = foc['voltage_clamp']

    print('Preparing BLDC FOC simulation...')

    time = np.arange(0.0, duration + timestep / 2, timestep)

    mechanical_omega = mechanical_rpm * 2.0 * np.pi / 60.0
    electrical_omega = pole_pairs * mechanical_omega
    rotor_angle = electrical_omega * time

    # Closed-loop FOC control with segment-based updates
    # Integral state
    id_integral = 0.0
    iq_integral = 0.0

    # Accumulate results
    sim_time = np.array([])
    ia = np.array([])
    ib = np.array([])
    ic = np.array([])
    id_measured_log = np.array([])
    iq_measured_log = np.array([])
    vd_ref_log = np.array([])
    vq_ref_log = np.array([])

    print('Running closed-loop FOC simulation (segment-based updates)...')

    num_segments = int(np.ceil(duration / control_period))
    for seg in range(num_segments):
        t0 = seg * control_period
        t1 = min(duration, (seg + 1) * control_period)
        time_seg = np.arange(t0, t1 + timestep / 2, timestep)

        # Rotor angle for this segment
        rotor_angle_seg = electrical_omega * time_seg

        # Generate phase voltages using current vd/q references
        valpha_seg, vbeta_seg = inverse_park(vd_ref, vq_ref, rotor_angle_seg)
        phase_a_seg, phase_b_seg, phase_c_seg = inverse_clarke(valpha_seg, vbeta_seg)

        # Run LTSpice for this segment
        ltspice_cmd = get_ltspice_command(config['ltspice']['command'])
        results = run_bldc_motor_simulation(
            netlist_path,
            time_seg,
            phase_a_seg,
            phase_b_seg,
            phase_c_seg,
            ltspice_command=ltspice_cmd,
            verbose=verbose
        )

        ia_seg = results['ia']
        ib_seg = results['ib']
        ic_seg = results['ic']
        time_raw = results['time']

        # Transform measured currents to dq on LTSpice timebase
        rotor_angle_sim_seg = electrical_omega * time_raw
        alpha_seg, beta_seg = abc_to_alpha_beta(ia_seg, ib_seg, ic_seg)
        id_seg, iq_seg = alpha_beta_to_dq(alpha_seg, beta_seg, rotor_angle_sim_seg)

        # Use last measured sample for control update
        id_meas = id_seg[-1]
        iq_meas = iq_seg[-1]

        # PI controller for d and q axes
        id_error = id_ref - id_meas
        iq_error = iq_ref - iq_meas

        id_integral += id_error * control_period
        iq_integral += iq_error * control_period

        # Clamp integrals to prevent windup
        id_integral = np.clip(id_integral, -integral_clamp, integral_clamp)
        iq_integral = np.clip(iq_integral, -integral_clamp, integral_clamp)

        vd_ref += Kp_d * id_error + Ki_d * id_integral
        vq_ref += Kp_q * iq_error + Ki_q * iq_integral

        # Clamp voltage references to reasonable range
        vd_ref = np.clip(vd_ref, -voltage_clamp, voltage_clamp)
        vq_ref = np.clip(vq_ref, -voltage_clamp, voltage_clamp)

        # Log for diagnostics
        vd_ref_log = np.append(vd_ref_log, vd_ref)
        vq_ref_log = np.append(vq_ref_log, vq_ref)
        id_measured_log = np.append(id_measured_log, id_meas)
        iq_measured_log = np.append(iq_measured_log, iq_meas)

        # Accumulate results
        sim_time = np.concatenate((sim_time, time_raw)) if sim_time.size else time_raw
        ia = np.concatenate((ia, ia_seg)) if ia.size else ia_seg
        ib = np.concatenate((ib, ib_seg)) if ib.size else ib_seg
        ic = np.concatenate((ic, ic_seg)) if ic.size else ic_seg

    # Compute dq currents for the full sim
    rotor_angle_sim = electrical_omega * sim_time
    alpha, beta = abc_to_alpha_beta(ia, ib, ic)
    id_current, iq_current = alpha_beta_to_dq(alpha, beta, rotor_angle_sim)

    # Reconstruct applied phase voltages (via interpolation of voltage references across control periods)
    time_control = np.arange(0, duration + control_period, control_period)[:len(vd_ref_log)]
    vd_ref_interp = np.interp(sim_time, time_control, vd_ref_log)
    vq_ref_interp = np.interp(sim_time, time_control, vq_ref_log)
    va_interp, vb_interp = inverse_park(vd_ref_interp, vq_ref_interp, rotor_angle_sim)
    phase_a_sim, phase_b_sim, phase_c_sim = inverse_clarke(va_interp, vb_interp)

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
    plt.savefig(project_root / 'foc_simulation.png')
    plt.show()

    print('Simulation complete. Results saved to foc_simulation.png')
