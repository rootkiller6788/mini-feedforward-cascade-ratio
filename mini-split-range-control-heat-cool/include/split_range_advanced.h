/**
 * @file split_range_advanced.h
 * @brief Advanced split-range features — auto-tuning, adaptive gain, reactor safety
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L5 Algorithms, L7 Applications, L8 Advanced Topics
 *
 * Advanced features include:
 *   - Asymmetric relay-feedback auto-tuning for heat/cool processes
 *   - Adaptive gain scheduling with Lyapunov stability guarantees
 *   - Monte Carlo calibration of adaptive parameters
 *   - Reactor runaway detection and emergency cooling
 *   - Anti-windup with asymmetric output limits
 *   - Split-point optimization for energy efficiency
 *
 * Reference:
 *   Astrom & Hagglund (1995) — Relay auto-tuning
 *   Åström & Wittenmark (2013) — Adaptive Control
 *   Fogler (2016) — Elements of Chemical Reaction Engineering (CSTR safety)
 *   Khalil (2002) — Nonlinear Systems (Lyapunov stability)
 *
 * Curriculum:
 *   MIT 6.302 — Relay feedback, nonlinear control
 *   Stanford ENGR205 — Adaptive gain scheduling
 *   CMU 24-677 — Lyapunov-based adaptive control
 *   Purdue ME575 — Reactor temperature safety systems
 */

#ifndef SPLIT_RANGE_ADVANCED_H
#define SPLIT_RANGE_ADVANCED_H

#include "split_range_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L5: Relay Auto-Tuning for Split-Range Processes
 * ========================================================================= */

/**
 * Initialize the relay auto-tuner for a split-range heat/cool process.
 * Configures asymmetric relay amplitudes to account for potentially
 * different heating and cooling process gains.
 *
 * @param autotune     Auto-tuning state to initialize
 * @param amp_heat     Relay amplitude for heating direction (%)
 * @param amp_cool     Relay amplitude for cooling direction (%)
 * @param hysteresis   Relay hysteresis to prevent chattering (%)
 *
 * Complexity: O(1).
 *
 * Principle: The relay induces a limit cycle oscillation. The ultimate
 * gain is Ku = 4*d / (pi*a) and the ultimate period is Pu = measured
 * cycle time, where d = relay amplitude and a = oscillation amplitude.
 *
 * For asymmetric processes: Ku_heat = 4*d_heat/(pi*a) is estimated
 * separately from Ku_cool = 4*d_cool/(pi*a). The PID is tuned more
 * conservatively based on the higher-gain direction.
 */
void split_autotune_init(split_range_autotune_t *autotune,
                           double amp_heat, double amp_cool,
                           double hysteresis);

/**
 * Process one sample in the relay auto-tune cycle. Returns the relay
 * output and updates the identification state.
 *
 * @param autotune Auto-tuning state (updated)
 * @param error    Current control error (SP - PV)
 * @param dt       Sample time (seconds)
 * @return         Relay output (recommended controller output for this step)
 *
 * Complexity: O(1).  Tracks cycle crossings to measure Pu.
 *
 * Termination: When cycle_count reaches a configurable threshold
 * (typically 3-5 complete cycles), identification is marked complete
 * and PID parameters can be extracted.
 */
double split_autotune_step(split_range_autotune_t *autotune,
                             double error, double dt);

/**
 * Extract PID parameters from a completed auto-tune cycle.
 * Uses Astrom-Hagglund tuning rules:
 *   Kc = 0.6 * Ku     (or Ku/3 for more robustness)
 *   Ti = 0.5 * Pu
 *   Td = 0.125 * Pu
 *
 * For asymmetric processes, uses the larger of Ku_heat and Ku_cool
 * for conservative tuning.
 *
 * @param autotune Auto-tuning state (must have identification_complete == true)
 * @param result   Output: computed tuning parameters
 * @return         0 on success, -1 if identification not complete
 *
 * Complexity: O(1).
 */
int split_autotune_get_params(const split_range_autotune_t *autotune,
                                split_range_tuning_result_t *result);

/* =========================================================================
 * L5: IMC-Based PID Tuning for Split-Range
 * ========================================================================= */

/**
 * Internal Model Control (IMC) tuning for split-range processes.
 *
 * IMC tuning gives better performance than ZN for processes with
 * significant dead time. The tuning is based on the process model
 * parameters and a user-specified closed-loop time constant lambda.
 *
 * For FOPDT model G(s) = K*exp(-theta*s)/(tau*s + 1):
 *   Kc = (tau + theta/2) / (K * (lambda + theta))
 *   Ti = tau + theta/2
 *   Td = tau*theta / (2*tau + theta)
 *
 * @param K        Process gain (use max of heating and cooling)
 * @param tau      Process time constant (seconds)
 * @param theta    Process dead time (seconds)
 * @param lambda   Desired closed-loop time constant (seconds, > 0)
 * @param result   Output: IMC-tuned PID parameters
 *
 * Complexity: O(1).
 *
 * Theorem (IMC, Morari & Zafiriou, 1989): For FOPDT processes, the
 * IMC-PID controller achieves the specified closed-loop time constant
 * lambda while guaranteeing stability for all lambda > theta/2.
 *
 * Reference: Morari & Zafiriou (1989) Robust Process Control
 */
void split_imc_tuning(double K, double tau, double theta, double lambda,
                        split_range_tuning_result_t *result);

/* =========================================================================
 * L8: Adaptive Split-Range Gain Scheduling with Lyapunov Guarantee
 * ========================================================================= */

/**
 * Initialize the adaptive gain scheduler for split-range control.
 *
 * @param adaptive  Adaptive state to initialize
 * @param kc_heat   Gain for heating region
 * @param kc_cool   Gain for cooling region
 * @param kc_neutral Gain for neutral (deadband) region
 * @param rate      Transition rate between gain sets
 *
 * Complexity: O(1).
 */
void split_adaptive_init(split_range_adaptive_t *adaptive,
                           double kc_heat, double kc_cool, double kc_neutral,
                           double rate);

/**
 * Update the effective PID gain based on the current operating region.
 * Uses smooth interpolation to avoid discontinuous gain jumps.
 *
 * The transition function g(t) from region A to region B:
 *   Kc_effective(t) = Kc_A + (Kc_B - Kc_A) * sigmoid(alpha * (co - co_transition))
 * where sigmoid(x) = 1 / (1 + exp(-x)) ensures smooth transition.
 *
 * @param adaptive Adaptive state (updated in-place)
 * @param co       Current controller output (%)
 * @return         Effective proportional gain Kc_eff
 *
 * Complexity: O(1).
 *
 * Lyapunov analysis: For a stable PID with Kc = Kc_A and Kc = Kc_B,
 * smooth interpolation guarantees stability if the rate of gain change
 * is slower than the closed-loop dynamics (slow-varying parameter theorem).
 *
 * Reference: Khalil (2002) Nonlinear Systems, Ch. 9 — Slowly Varying Systems
 */
double split_adaptive_update_gain(split_range_adaptive_t *adaptive,
                                    double co);

/**
 * Monte Carlo calibration of adaptive parameters.
 * Runs N simulations with perturbed process models to find the
 * gain parameters that minimize a cost function (IAE + overshoot
 * penalty) across all scenarios.
 *
 * This is a stochastic optimization method based on the Metropolis-
 * Hastings MCMC algorithm adapted for parameter calibration.
 *
 * @param adaptive      Adaptive state (updated with calibrated params)
 * @param simulate_fn   User-provided simulation function pointer
 * @param N             Number of Monte Carlo iterations
 * @param noise_std     Standard deviation of parameter perturbations
 * @return              Acceptance ratio (fraction of proposed moves accepted)
 *
 * Complexity: O(N * T) where T is the simulation time per iteration.
 *
 * Reference: Andrieu et al. (2003) "An Introduction to MCMC for Machine Learning"
 */
double split_monte_carlo_calibration(split_range_adaptive_t *adaptive,
                                       double (*simulate_fn)(double kc, double ti, double td),
                                       int N, double noise_std);

/* =========================================================================
 * L6: Reactor Runaway Detection & Emergency Response
 * ========================================================================= */

/**
 * Detect thermal runaway risk in a jacketed CSTR reactor.
 *
 * Runaway condition (Semenov criterion):
 *   dT/dt > 0 and d^2T/dt^2 > 0  (temperature accelerating upward)
 *   AND T > inflection_temp
 *
 * This implies positive feedback: exothermic reaction heating faster
 * than cooling can remove heat.
 *
 * @param model  Reactor model (updated in-place)
 * @param dt_sec Time step for derivative calculation
 * @return       true if runaway risk detected
 *
 * Complexity: O(1).
 *
 * Reference: Fogler (2016) Elements of Chemical Reaction Engineering, Ch. 12
 *   Semenov (1928) "Theories of Combustion Processes"
 */
bool split_reactor_runaway_detect(split_range_reactor_model_t *model,
                                    double dt_sec);

/**
 * Initiate emergency cooling response for a reactor in runaway.
 * Forces cooling valve to 100% and heating valve to 0%, overriding
 * normal PID control. Records the event for post-incident analysis.
 *
 * @param ctrl  Split-range controller (mode overridden)
 *
 * Complexity: O(1).  Sets emergency state on all channels.
 *
 * Safety: This implements the ISA-84.00.01 Safety Instrumented Function
 * (SIF) for reactor over-temperature protection. In a real plant, this
 * would be implemented in a separate Safety PLC.
 *
 * Reference: ISA-84.00.01 / IEC 61511 — Functional Safety for Process Industry
 */
void split_reactor_emergency_cooling(split_range_controller_t *ctrl);

/**
 * Simulate one time step of the jacketed CSTR reactor energy balance.
 *
 * dT/dt = (F/V)*(Tin - T)
 *         + (U*A/(V*rho*Cp))*(Tj - T)
 *         + (-delta_Hr/(rho*Cp))*k0*exp(-Ea/(R*T))*CA
 *
 * Uses forward Euler integration for simplicity.
 *
 * @param model   Reactor model (temperature updated in-place)
 * @param Q_heat  Heating power applied (W or fraction of max)
 * @param Q_cool  Cooling power applied (W or fraction of max)
 * @param dt_sec  Integration time step (seconds)
 *
 * Complexity: O(1).  Numerical integration of energy balance ODE.
 *
 * WARNING: The Arrhenius term exp(-Ea/(R*T)) is highly nonlinear.
 * For large dt, the forward Euler method may be unstable. Use dt < tau/10
 * where tau = V*rho*Cp/(U*A) is the thermal time constant.
 */
void split_reactor_simulate_step(split_range_reactor_model_t *model,
                                   double Q_heat, double Q_cool, double dt_sec);

/* =========================================================================
 * L5: Split-Point Optimization
 * ========================================================================= */

/**
 * Optimize the split point to minimize total energy consumption
 * (heating + cooling) over a representative operating period.
 *
 * The optimization uses a golden-section search on the split point
 * parameter, evaluating energy cost at each candidate split point.
 *
 * @param ctrl          Split-range controller
 * @param energy_cost_fn Cost function: double fn(double co, double heating, double cooling)
 * @param tol           Tolerance for convergence (e.g., 0.1%)
 * @return              Optimal split point (%)
 *
 * Complexity: O(log(1/tol)) evaluations of the cost function.
 *
 * Theorem: For convex energy cost functions, the golden-section search
 * converges to the global minimum with linear convergence rate ~0.618.
 * For non-convex costs, the result is a local minimum near the initial point.
 */
double split_optimize_split_point(const split_range_controller_t *ctrl,
                                    double (*energy_cost_fn)(double, double, double),
                                    double tol);

/**
 * Analyze the cross-coupling between heating and cooling channels.
 * When the split point is not perfectly chosen, both valves can be
 * partially open simultaneously, wasting energy.
 *
 * @param ctrl  Split-range controller
 * @return      Cross-coupling index: 0 = perfectly decoupled,
 *              1 = maximum coupling (both valves at 100%)
 *
 * Complexity: O(n).  The index is product of concurrent valve openings.
 */
double split_cross_coupling_analysis(const split_range_controller_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_ADVANCED_H */
