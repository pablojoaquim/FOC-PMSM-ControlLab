#!/usr/bin/env python3
"""
Field-Oriented Control (FOC) module for BLDC motor simulation.

Implements Park and Clarke transforms, PI current control loops, and closed-loop
FOC simulation with configurable gains and limits.
"""

import numpy as np
from bldc_simulator import run_bldc_motor_simulation
from ltspice_driver import get_ltspice_command

SQRT3 = np.sqrt(3)


class FOCController:
    """Field-Oriented Control controller for BLDC motors."""

    def __init__(self, config):
        """
        Initialize FOC controller from configuration.

        Args:
            config: Dictionary with 'foc_controller' and 'motor' keys.
        """
        foc_cfg = config['foc_controller']
        motor_cfg = config['motor']

        # Control parameters
        self.control_period = foc_cfg['control_period']
        self.Kp_d = foc_cfg['kp_d']
        self.Ki_d = foc_cfg['ki_d']
        self.Kp_q = foc_cfg['kp_q']
        self.Ki_q = foc_cfg['ki_q']

        # Current references
        self.id_ref = foc_cfg['id_ref']
        self.iq_ref = foc_cfg['iq_ref']

        # Voltage references (initial)
        self.vd_ref = foc_cfg['vd_ref_init']
        self.vq_ref = foc_cfg['vq_ref_init']

        # Limits
        self.integral_clamp = foc_cfg['integral_clamp']
        self.voltage_clamp = foc_cfg['voltage_clamp']

        # Motor parameters
        self.pole_pairs = motor_cfg['pole_pairs']
        mechanical_rpm = motor_cfg['mechanical_rpm']
        self.mechanical_omega = mechanical_rpm * 2.0 * np.pi / 60.0
        self.electrical_omega = self.pole_pairs * self.mechanical_omega

        # Integral state
        self.id_integral = 0.0
        self.iq_integral = 0.0

        # Logs
        self.vd_ref_log = np.array([])
        self.vq_ref_log = np.array([])
        self.id_measured_log = np.array([])
        self.iq_measured_log = np.array([])

    def reset(self):
        """Reset controller state for a new simulation."""
        self.id_integral = 0.0
        self.iq_integral = 0.0
        self.vd_ref_log = np.array([])
        self.vq_ref_log = np.array([])
        self.id_measured_log = np.array([])
        self.iq_measured_log = np.array([])

    def update(self, id_meas, iq_meas):
        """
        Update FOC controller with measured D-Q currents.

        Args:
            id_meas: Measured D-axis current.
            iq_meas: Measured Q-axis current.

        Returns:
            Tuple of (vd_ref, vq_ref) voltage references.
        """
        # PI controller errors
        id_error = self.id_ref - id_meas
        iq_error = self.iq_ref - iq_meas

        # Integral terms
        self.id_integral += id_error * self.control_period
        self.iq_integral += iq_error * self.control_period

        # Anti-windup clamping
        self.id_integral = np.clip(self.id_integral, -self.integral_clamp, self.integral_clamp)
        self.iq_integral = np.clip(self.iq_integral, -self.integral_clamp, self.integral_clamp)

        # PI control output
        self.vd_ref += self.Kp_d * id_error + self.Ki_d * self.id_integral
        self.vq_ref += self.Kp_q * iq_error + self.Ki_q * self.iq_integral

        # Voltage saturation
        self.vd_ref = np.clip(self.vd_ref, -self.voltage_clamp, self.voltage_clamp)
        self.vq_ref = np.clip(self.vq_ref, -self.voltage_clamp, self.voltage_clamp)

        # Log
        self.vd_ref_log = np.append(self.vd_ref_log, self.vd_ref)
        self.vq_ref_log = np.append(self.vq_ref_log, self.vq_ref)
        self.id_measured_log = np.append(self.id_measured_log, id_meas)
        self.iq_measured_log = np.append(self.iq_measured_log, iq_meas)

        return self.vd_ref, self.vq_ref


# Transform functions
def inverse_park(vd, vq, theta):
    """Transform D-Q voltages to alpha-beta (stationary frame)."""
    valpha = vd * np.cos(theta) - vq * np.sin(theta)
    vbeta = vd * np.sin(theta) + vq * np.cos(theta)
    return valpha, vbeta


def inverse_clarke(valpha, vbeta):
    """Transform alpha-beta voltages to three-phase A-B-C."""
    va = valpha
    vb = -0.5 * valpha + SQRT3 / 2.0 * vbeta
    vc = -0.5 * valpha - SQRT3 / 2.0 * vbeta
    return va, vb, vc


def abc_to_alpha_beta(ia, ib, ic):
    """Transform three-phase A-B-C currents to alpha-beta (stationary frame)."""
    alpha = ia
    beta = (ia + 2.0 * ib) / SQRT3
    return alpha, beta


def alpha_beta_to_dq(alpha, beta, theta):
    """Transform alpha-beta currents to D-Q (rotor-aligned frame)."""
    id_current = alpha * np.cos(theta) + beta * np.sin(theta)
    iq_current = -alpha * np.sin(theta) + beta * np.cos(theta)
    return id_current, iq_current


def run_foc_closed_loop_simulation(controller, netlist_path, duration, timestep, config_ltspice, verbose=False):
    """
    Run closed-loop FOC simulation with segment-based control updates.

    Args:
        controller: FOCController instance.
        netlist_path: Path to LTSpice netlist.
        duration: Total simulation duration (seconds).
        timestep: Simulation timestep (seconds).
        config_ltspice: LTSpice configuration dictionary.
        verbose: Print debug information.

    Returns:
        Dictionary with sim_time, ia, ib, ic, id_current, iq_current, and voltage logs.
    """
    controller.reset()

    # Time vectors
    time = np.arange(0.0, duration + timestep / 2, timestep)

    # Get LTSpice command
    ltspice_cmd = get_ltspice_command(config_ltspice.get('command'))

    # Accumulate results
    sim_time = np.array([])
    ia = np.array([])
    ib = np.array([])
    ic = np.array([])

    print('Running closed-loop FOC simulation (segment-based updates)...')

    num_segments = int(np.ceil(duration / controller.control_period))
    for seg in range(num_segments):
        t0 = seg * controller.control_period
        t1 = min(duration, (seg + 1) * controller.control_period)
        time_seg = np.arange(t0, t1 + timestep / 2, timestep)

        # Rotor angle for this segment
        rotor_angle_seg = controller.electrical_omega * time_seg

        # Generate phase voltages using current vd/q references
        valpha_seg, vbeta_seg = inverse_park(controller.vd_ref, controller.vq_ref, rotor_angle_seg)
        phase_a_seg, phase_b_seg, phase_c_seg = inverse_clarke(valpha_seg, vbeta_seg)

        # Run LTSpice simulation
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

        # Transform measured currents to D-Q on LTSpice timebase
        rotor_angle_sim_seg = controller.electrical_omega * time_raw
        alpha_seg, beta_seg = abc_to_alpha_beta(ia_seg, ib_seg, ic_seg)
        id_seg, iq_seg = alpha_beta_to_dq(alpha_seg, beta_seg, rotor_angle_sim_seg)

        # Use last measured sample for control update
        id_meas = id_seg[-1]
        iq_meas = iq_seg[-1]

        # Update controller
        controller.update(id_meas, iq_meas)

        # Accumulate results
        sim_time = np.concatenate((sim_time, time_raw)) if sim_time.size else time_raw
        ia = np.concatenate((ia, ia_seg)) if ia.size else ia_seg
        ib = np.concatenate((ib, ib_seg)) if ib.size else ib_seg
        ic = np.concatenate((ic, ic_seg)) if ic.size else ic_seg

    # Compute D-Q currents for the full simulation
    rotor_angle_sim = controller.electrical_omega * sim_time
    alpha, beta = abc_to_alpha_beta(ia, ib, ic)
    id_current, iq_current = alpha_beta_to_dq(alpha, beta, rotor_angle_sim)

    # Reconstruct applied phase voltages via interpolation
    time_control = np.arange(0, duration + controller.control_period, controller.control_period)[:len(controller.vd_ref_log)]
    vd_ref_interp = np.interp(sim_time, time_control, controller.vd_ref_log)
    vq_ref_interp = np.interp(sim_time, time_control, controller.vq_ref_log)
    va_interp, vb_interp = inverse_park(vd_ref_interp, vq_ref_interp, rotor_angle_sim)
    phase_a_sim, phase_b_sim, phase_c_sim = inverse_clarke(va_interp, vb_interp)

    return {
        'sim_time': sim_time,
        'ia': ia,
        'ib': ib,
        'ic': ic,
        'id_current': id_current,
        'iq_current': iq_current,
        'phase_a_sim': phase_a_sim,
        'phase_b_sim': phase_b_sim,
        'phase_c_sim': phase_c_sim,
        'vd_ref_log': controller.vd_ref_log,
        'vq_ref_log': controller.vq_ref_log,
        'id_measured_log': controller.id_measured_log,
        'iq_measured_log': controller.iq_measured_log,
    }
