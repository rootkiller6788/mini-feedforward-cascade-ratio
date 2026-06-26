/**
 * @file ratio_core.c
 * @brief Core ratio computation — ratio station, master-slave tracking,
 *        flow unit conversion, and ratio validation.
 *
 * Level: L1 Definitions + L2 Core Concepts + L3 Engineering Structures
 *
 * This file implements the fundamental ratio control computations:
 *   - Ratio computation: R = F_slave / F_master
 *   - Slave setpoint generation: SP_slave = R_sp * F_master
 *   - Flow unit conversion (mass/volume/molar/normalized)
 *   - Ratio validation and safety checking
 *   - Ratio station (feedforward multiplier)
 *
 * Each function represents one independent knowledge point from
 * process control engineering.
 *
 * Industrial References:
 *   ISO 5167 — Orifice plate flow measurement for ratio control
 *   Toyota Production System — Just-in-time blending ratio management
 *   Tesla Gigafactory — Precision solvent ratio for battery manufacturing
 */

#include "ratio_types.h"
#include "ratio_controller.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L1: Ratio Definition & Computation
 * Reference: Seborg et al. (2016), Ch.15.3 — "Ratio Control"
 *
 * Ratio control is a feedforward strategy where one variable (slave)
 * is maintained at a prescribed ratio to another variable (master).
 *
 *   R = (slave flow) / (master flow)
 *
 * The ratio R is dimensionless when both flows share the same unit.
 * ========================================================================= */

/**
 * @brief Compute the basic ratio between slave and master flows.
 *
 * This is the fundamental ratio control calculation:
 *   R = F_slave / F_master
 *
 * Requires: F_master > 0 (to avoid division by zero)
 *
 * Knowledge point: Ratio definition in process control.
 * Without a correct ratio definition, all downstream control
 * (trim, cross-limiting, blending) is meaningless.
 *
 * Complexity: O(1)
 */
double ratio_compute_basic(double F_slave, double F_master)
{
    if (F_master <= 0.0) {
        return 0.0;
    }
    return F_slave / F_master;
}

/* =========================================================================
 * L1: Ratio Setpoint — Target Ratio
 *
 * The ratio setpoint is the desired ratio between slave and master flows.
 * In combustion: R_sp = AFR_target (air-fuel ratio target)
 * In blending:   R_sp = component_flow / total_flow
 * In absorption: R_sp = L/G (liquid-to-gas ratio for column design)
 * ========================================================================= */

/**
 * @brief Validate a ratio setpoint.
 *
 * A valid ratio setpoint must be:
 *   1. Positive (R_sp > 0)
 *   2. Within physical limits (R_min ≤ R_sp ≤ R_max)
 *   3. R_min > 0 and R_max > R_min
 *
 * Knowledge point: Ratio setpoint validation — ensures the control
 * system operates within physically realizable bounds.
 *
 * Complexity: O(1)
 */
int ratio_setpoint_validate(double R_sp, double R_min, double R_max)
{
    if (R_sp <= 0.0)     return 0;
    if (R_min <= 0.0)    return 0;
    if (R_max <= R_min)  return 0;
    if (R_sp < R_min)    return 0;
    if (R_sp > R_max)    return 0;
    return 1;
}

/**
 * @brief Clamp a ratio value to specified limits.
 *
 * Applies saturation:
 *   R_clamped = max(R_min, min(R_max, R))
 *
 * Knowledge point: Ratio limit enforcement — hard clamping prevents
 * the slave flow from being commanded to unsafe values.
 *
 * Complexity: O(1)
 */
double ratio_clamp(double R, double R_min, double R_max)
{
    if (R < R_min) return R_min;
    if (R > R_max) return R_max;
    return R;
}

/* =========================================================================
 * L2: Master-Slave Ratio Architecture
 *
 * The ratio station is a feedforward multiplier:
 *   SP_slave = R * F_master
 *
 * This is the classic ratio control topology:
 *
 *   F_master ──→ [× R] ──→ SP_slave ──→ [FC_slave] ──→ F_slave
 *                                ↑                    │
 *                                └──── feedback ──────┘
 *
 * The slave flow controller (FC_slave) is a standard PID loop.
 * The ratio station is a pure feedforward element.
 * ========================================================================= */

/**
 * @brief Compute slave flow setpoint from ratio and master flow.
 *
 * Core ratio control equation:
 *   SP_slave = R * F_master
 *
 * This implements the ratio station (also called ratio relay or
 * ratio multiplier in DCS function block libraries).
 *
 * Knowledge point: Ratio station — the fundamental feedforward
 * computation that translates a ratio into a flow setpoint.
 *
 * Complexity: O(1)
 */
double ratio_station(double R, double F_master)
{
    if (F_master <= 0.0) return 0.0;
    if (R <= 0.0)        return 0.0;
    return R * F_master;
}

/**
 * @brief Compute required ratio to achieve a desired slave flow.
 *
 * Inverse of ratio_station:
 *   R_required = SP_slave / F_master
 *
 * Useful for: operator entry of desired slave flow, then computing
 * the required ratio setpoint.
 *
 * Knowledge point: Inverse ratio computation — allows operators to
 * think in terms of absolute flows while the control system
 * operates in ratio space.
 *
 * Complexity: O(1)
 */
double ratio_inverse(double SP_slave, double F_master)
{
    if (F_master <= 0.0) return 0.0;
    return SP_slave / F_master;
}

/**
 * @brief Compute master flow required for a given slave flow at a target ratio.
 *
 *   F_master_required = F_slave / R
 *
 * Useful for capacity planning and constraint checking.
 *
 * Complexity: O(1)
 */
double ratio_master_required(double F_slave, double R)
{
    if (R <= 0.0) return 0.0;
    return F_slave / R;
}

/* =========================================================================
 * L2: Ratio Error Computation
 *
 * Ratio error drives the trim feedback loop:
 *   e_R = R_actual - R_effective
 *
 * A positive error: slave flow too high (excess slave relative to master)
 * A negative error: slave flow too low (insufficient slave)
 * ========================================================================= */

/**
 * @brief Compute ratio error.
 *
 *   e_R = R_actual - R_sp
 *
 * This error is used:
 *   1. As feedback for ratio trim controller
 *   2. For alarming and operator display
 *   3. For performance monitoring
 *
 * Knowledge point: Ratio error — the fundamental deviation signal
 * that drives all ratio correction mechanisms.
 *
 * Complexity: O(1)
 */
double ratio_error_compute(double R_actual, double R_sp)
{
    return R_actual - R_sp;
}

/**
 * @brief Compute ratio error as percentage of setpoint.
 *
 *   e_R_pct = (R_actual - R_sp) / R_sp * 100%
 *
 * Percentage error is more meaningful for operators:
 *   "Ratio is 3% above target" vs "Ratio error is 0.45"
 *
 * Knowledge point: Normalized ratio error — enables consistent
 * alarming thresholds across different ratio setpoints.
 *
 * Complexity: O(1)
 */
double ratio_error_percent(double R_actual, double R_sp)
{
    if (R_sp <= 0.0) return 0.0;
    return (R_actual - R_sp) / R_sp * 100.0;
}

/* =========================================================================
 * L3: Flow Unit Conversion
 *
 * Ratio control requires consistent units. In gas-liquid systems,
 * the master (gas) and slave (liquid) often have different units.
 * Conversion to a common basis (mass flow) is essential.
 *
 * Mass flow = volumetric flow × density
 * Molar flow = mass flow / molar mass
 * Normalized flow = flow / full_scale
 * ========================================================================= */

/**
 * @brief Convert volumetric flow to mass flow.
 *
 *   F_mass = F_volume * rho
 *
 * Knowledge point: Volumetric-to-mass conversion — the most common
 * unit conversion in ratio control, essential when gas and liquid
 * flows are measured in different units.
 *
 * Complexity: O(1)
 */
double flow_volume_to_mass(double F_volume, double density)
{
    return F_volume * density;
}

/**
 * @brief Convert mass flow to volumetric flow.
 *
 *   F_volume = F_mass / rho
 *
 * Complexity: O(1)
 */
double flow_mass_to_volume(double F_mass, double density)
{
    if (density <= 0.0) return 0.0;
    return F_mass / density;
}

/**
 * @brief Convert mass flow to molar flow.
 *
 *   F_molar = F_mass / M
 *
 * where M is the molar mass (kg/mol or g/mol depending on units).
 * Molar flow is used in reaction engineering for stoichiometric
 * ratio control.
 *
 * Knowledge point: Molar flow conversion — required for stoichiometric
 * ratio control in chemical reactors and combustion systems.
 *
 * Complexity: O(1)
 */
double flow_mass_to_molar(double F_mass, double molar_mass)
{
    if (molar_mass <= 0.0) return 0.0;
    return F_mass / molar_mass;
}

/**
 * @brief Convert molar flow to mass flow.
 *
 *   F_mass = F_molar * M
 *
 * Complexity: O(1)
 */
double flow_molar_to_mass(double F_molar, double molar_mass)
{
    return F_molar * molar_mass;
}

/**
 * @brief Convert actual gas flow to normal flow (Nm³/h equivalent).
 *
 * Uses the combined gas law:
 *   Q_normal = Q_actual * (P_actual / P_normal) * (T_normal / T_actual)
 *
 * This is essential in gas-liquid ratio control because gas density
 * varies significantly with temperature and pressure, while the
 * ratio control is typically specified at normal/reference conditions.
 *
 * Knowledge point: Normal flow conversion — compensates gas flow
 * measurements for temperature and pressure variations, enabling
 * consistent ratio control regardless of ambient conditions.
 *
 * Reference: ISO 13443 — Standard reference conditions for natural gas
 *
 * Complexity: O(1)
 */
double gas_flow_normalize(double Q_actual, double P_actual, double T_actual,
                           double P_normal, double T_normal)
{
    if (P_normal <= 0.0 || T_actual <= 0.0) return 0.0;
    return Q_actual * (P_actual / P_normal) * (T_normal / T_actual);
}

/**
 * @brief Convert normal gas flow to actual flow.
 *
 * Inverse of gas_flow_normalize:
 *   Q_actual = Q_normal * (P_normal / P_actual) * (T_actual / T_normal)
 *
 * Complexity: O(1)
 */
double gas_flow_actualize(double Q_normal, double P_actual, double T_actual,
                           double P_normal, double T_normal)
{
    if (P_actual <= 0.0 || T_normal <= 0.0) return 0.0;
    return Q_normal * (P_normal / P_actual) * (T_actual / T_normal);
}

/**
 * @brief Convert between flow measurement units.
 *
 * Handles conversions between mass, volume, molar, and normalized flows.
 * Used by ratio controller when master and slave have different units.
 *
 * Knowledge point: Multi-unit flow conversion — the ratio controller's
 * internal unit normalization layer, ensuring consistent ratio
 * computation regardless of field instrument configuration.
 *
 * Complexity: O(1)
 */
double flow_unit_convert(double flow, flow_unit_t from_unit, flow_unit_t to_unit,
                          double density, double molar_mass, double full_scale)
{
    /* Convert to mass flow first as common basis */
    double mass_flow;

    switch (from_unit) {
        case FLOW_UNIT_MASS:
            mass_flow = flow;
            break;
        case FLOW_UNIT_VOLUME:
            mass_flow = flow_volume_to_mass(flow, density);
            break;
        case FLOW_UNIT_MOLAR:
            mass_flow = flow_molar_to_mass(flow, molar_mass);
            break;
        case FLOW_UNIT_NORMALIZED:
            mass_flow = flow * full_scale;
            break;
        default:
            return 0.0;
    }

    /* Convert from mass to target unit */
    switch (to_unit) {
        case FLOW_UNIT_MASS:
            return mass_flow;
        case FLOW_UNIT_VOLUME:
            return flow_mass_to_volume(mass_flow, density);
        case FLOW_UNIT_MOLAR:
            return flow_mass_to_molar(mass_flow, molar_mass);
        case FLOW_UNIT_NORMALIZED:
            if (full_scale <= 0.0) return 0.0;
            return mass_flow / full_scale;
        default:
            return 0.0;
    }
}

/* =========================================================================
 * L3: Ratio Signal Filtering
 *
 * Flow measurements contain noise (turbulence, sensor noise, electrical).
 * Filtering the master flow before ratio multiplication prevents
 * the noise from propagating to the slave setpoint.
 *
 * First-order exponential filter (Exponentially Weighted Moving Average):
 *   y(k) = α * x(k) + (1-α) * y(k-1)
 *
 * where α = Ts / (Ts + τ) and τ is the filter time constant.
 * ========================================================================= */

/**
 * @brief First-order low-pass filter (EWMA) for flow signals.
 *
 *   y_filtered = α * x_raw + (1-α) * y_prev
 *
 * Filter time constant τ controls the cutoff frequency:
 *   f_cutoff = 1 / (2π * τ)
 *
 * For gas flow measurement: τ = 1-3 seconds typical
 * For liquid flow measurement: τ = 0.5-1 second typical
 * (liquid flow is less noisy than gas flow due to incompressibility)
 *
 * Knowledge point: Exponential filtering — the standard noise rejection
 * technique for flow measurements in ratio control. Critical for
 * preventing ratio jitter from turbulent gas flow noise.
 *
 * Complexity: O(1)
 */
double flow_ewma_filter(double x_raw, double y_prev, double Ts, double tau)
{
    if (tau < 0.0) return x_raw;            /* No filtering if τ negative */
    if (tau == 0.0 || Ts <= 0.0) return x_raw; /* No filtering */
    double alpha = Ts / (Ts + tau);
    if (alpha > 1.0) alpha = 1.0;
    return alpha * x_raw + (1.0 - alpha) * y_prev;
}

/**
 * @brief Moving average filter over a fixed window.
 *
 * Simple moving average: requires storing the last N samples.
 * This function computes the moving average update when a new
 * sample arrives (requires the oldest sample to be dropped).
 *
 *   sum_new = sum_old - oldest_sample + new_sample
 *   average = sum_new / N
 *
 * Knowledge point: Moving average — provides uniform weighting of
 * the last N samples, unlike EWMA which exponentially decays
 * older samples. Better for periodic flow pulsations.
 *
 * Complexity: O(1) per update (if sum is maintained)
 */
double flow_moving_average_update(double new_sample, double oldest_sample,
                                   double sum_old, int window_size,
                                   double *sum_new)
{
    if (window_size <= 0) {
        *sum_new = new_sample;
        return new_sample;
    }
    *sum_new = sum_old - oldest_sample + new_sample;
    return (*sum_new) / (double)window_size;
}

/* =========================================================================
 * L4: Ratio Safety & Validation
 *
 * Ratio control systems must validate measurements before computing
 * control actions. Bad measurements (sensor failure, communication
 * loss) must not propagate into the control output.
 * ========================================================================= */

/**
 * @brief Validate a flow measurement for use in ratio control.
 *
 * Checks:
 *   1. Flow value is finite (not NaN or Inf)
 *   2. Flow value is non-negative (negative flow is physically impossible)
 *   3. Signal quality is acceptable (≥ 1, i.e., not "bad")
 *
 * Returns: 1 if valid, 0 if invalid
 *
 * Knowledge point: Measurement validation — the first line of defense
 * against bad data in ratio control. Running ratio control on invalid
 * measurements can produce dangerously incorrect slave setpoints.
 *
 * Complexity: O(1)
 */
int ratio_validate_measurement(double flow, int quality)
{
    if (!isfinite(flow))   return 0;
    if (flow < 0.0)        return 0;
    if (quality < 1)       return 0;  /* 0=bad, require at least uncertain */
    return 1;
}

/**
 * @brief Check if the ratio is within a specified tolerance band.
 *
 * |R_actual - R_sp| ≤ tolerance * R_sp
 *
 * This is used for:
 *   - Ratio control quality assessment
 *   - Operator alarming
 *   - Product quality guarantee (for blending)
 *
 * Knowledge point: Ratio tolerance — the acceptable error band for
 * ratio control. For combustion, ±5% is typical; for pharmaceutical
 * blending, ±1% or tighter.
 *
 * Complexity: O(1)
 */
int ratio_within_tolerance(double R_actual, double R_sp, double tolerance)
{
    if (R_sp <= 0.0) return 0;
    double abs_err = fabs(R_actual - R_sp);
    return abs_err <= tolerance * R_sp;
}

/**
 * @brief Detect ratio divergence (runaway condition).
 *
 * If the ratio error is growing monotonically over several samples,
 * the ratio control loop may be unstable. This function checks
 * for divergence by testing if successive errors have the same sign
 * and increasing magnitude.
 *
 * Knowledge point: Ratio divergence detection — early warning of
 * ratio loop instability, enabling preemptive operator intervention
 * or automatic fallback to manual mode.
 *
 * Complexity: O(1)
 */
int ratio_detect_divergence(double error_k, double error_km1, double error_km2)
{
    int same_sign = (error_k > 0 && error_km1 > 0 && error_km2 > 0) ||
                    (error_k < 0 && error_km1 < 0 && error_km2 < 0);
    if (!same_sign) return 0;

    int growing = (fabs(error_k) > fabs(error_km1)) &&
                  (fabs(error_km1) > fabs(error_km2));
    return growing ? 1 : 0;
}

/* =========================================================================
 * L5: Stoichiometric Ratio Constants
 *
 * Stoichiometric ratios are fundamental to combustion ratio control.
 * They represent the exact amount of oxidizer (air) required to
 * completely combust a given amount of fuel, with no excess.
 * ========================================================================= */

/**
 * @brief Get stoichiometric air-fuel ratio (mass basis) for a given fuel.
 *
 * Fuel codes:
 *   0 = natural gas (CH4): 17.2 kg_air/kg_fuel
 *   1 = propane (C3H8):    15.7 kg_air/kg_fuel
 *   2 = fuel oil #2:       14.5 kg_air/kg_fuel
 *   3 = coal (bituminous): 11.5 kg_air/kg_fuel
 *   4 = hydrogen (H2):    34.3 kg_air/kg_fuel
 *   5 = gasoline:         14.7 kg_air/kg_fuel
 *   6 = diesel:           14.5 kg_air/kg_fuel
 *   7 = ethanol:           9.0 kg_air/kg_fuel
 *
 * Knowledge point: Stoichiometric AFR — the theoretical minimum air
 * required for complete combustion. All practical combustion operates
 * lean of stoichiometric (excess air) to ensure complete fuel burnout.
 *
 * Reference: Turns, "An Introduction to Combustion" (2012), Ch.2
 *
 * Complexity: O(1)
 */
double stoichiometric_afr_get(int fuel_code)
{
    switch (fuel_code) {
        case 0: return 17.2;   /* Natural gas */
        case 1: return 15.7;   /* Propane */
        case 2: return 14.5;   /* Fuel oil #2 */
        case 3: return 11.5;   /* Coal (bituminous) */
        case 4: return 34.3;   /* Hydrogen */
        case 5: return 14.7;   /* Gasoline */
        case 6: return 14.5;   /* Diesel */
        case 7: return  9.0;   /* Ethanol */
        default: return 0.0;
    }
}

/**
 * @brief Compute the excess air ratio λ from measured AFR.
 *
 *   λ = AFR_actual / AFR_stoich
 *
 * λ = 1.0 : stoichiometric (no excess air)
 * λ > 1.0 : lean (excess air)
 * λ < 1.0 : rich (excess fuel, incomplete combustion)
 *
 * Knowledge point: Lambda (excess air ratio) — the standard measure
 * of combustion air-fuel balance. Automotive O2 sensors directly
 * measure λ. Industrial boilers target λ = 1.1-1.3.
 *
 * Complexity: O(1)
 */
double lambda_from_afr(double AFR_actual, double AFR_stoich)
{
    if (AFR_stoich <= 0.0) return 0.0;
    return AFR_actual / AFR_stoich;
}

/**
 * @brief Compute percent excess air from lambda.
 *
 *   excess_air_pct = (λ - 1) * 100%
 *
 * For industrial boilers:
 *   Natural gas: 5-15% excess air (λ=1.05-1.15)
 *   Fuel oil:    10-20% excess air (λ=1.10-1.20)
 *   Coal:        15-40% excess air (λ=1.15-1.40)
 *
 * Knowledge point: Percent excess air — more intuitive for operators
 * than lambda. Excess air costs efficiency (heating extra air) but
 * prevents CO formation from incomplete combustion.
 *
 * Complexity: O(1)
 */
double excess_air_percent(double lambda)
{
    return (lambda - 1.0) * 100.0;
}

/**
 * @brief Compute combustion efficiency loss due to excess air.
 *
 * Each 1% excess O2 in flue gas corresponds to approximately
 * 0.3-0.5% efficiency loss for natural gas, 0.4-0.6% for oil.
 *
 * Stack loss estimate:
 *   efficiency_loss_pct = k_fuel * excess_air_pct
 *
 * where k_fuel depends on the fuel composition.
 *
 * Knowledge point: Excess air efficiency penalty — the economic
 * motivation for precise ratio control. Reducing excess air from
 * 20% to 10% can save 1-3% fuel cost.
 *
 * Complexity: O(1)
 */
double combustion_efficiency_loss(double excess_air_pct, double k_fuel)
{
    return k_fuel * excess_air_pct;
}

/* =========================================================================
 * L6: Ratio Scheduling
 *
 * The optimal ratio may change with operating conditions (load,
 * temperature, feedstock composition). Ratio scheduling adjusts
 * the ratio setpoint based on a measured condition.
 * ========================================================================= */

/**
 * @brief Compute a scheduled ratio setpoint based on load.
 *
 * For boilers: optimal excess air increases at low loads
 *   R_sp(load) = R_nominal * (1 + penalty * (1 - load/load_max))
 *
 * For absorbers: optimal L/G increases with inlet concentration
 *   R_sp(C_in) = R_min + (R_max - R_min) * (C_in - C_min) / (C_max - C_min)
 *
 * Knowledge point: Ratio scheduling — the ratio is not constant but
 * depends on operating conditions. This is a form of gain scheduling
 * applied to the ratio setpoint.
 *
 * Complexity: O(1)
 */
double ratio_schedule_linear(double condition, double cond_min, double cond_max,
                              double R_min, double R_max)
{
    if (cond_max <= cond_min) return R_min;
    double frac = (condition - cond_min) / (cond_max - cond_min);
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;
    return R_min + frac * (R_max - R_min);
}

/**
 * @brief Compute a deadband around the ratio setpoint.
 *
 * A deadband prevents the ratio controller from reacting to
 * small, insignificant ratio errors (measurement noise).
 *
 *   If |e_R| < deadband → treat as zero error
 *
 * This reduces actuator wear and prevents ratio hunting.
 *
 * Knowledge point: Ratio deadband — a practical necessity in
 * industrial ratio control. Without a deadband, the slave flow
 * controller would constantly chase small ratio variations
 * caused by measurement noise.
 *
 * Complexity: O(1)
 */
double ratio_deadband_apply(double ratio_error, double deadband)
{
    if (deadband <= 0.0) return ratio_error;
    if (fabs(ratio_error) < deadband) return 0.0;
    return ratio_error;
}

/**
 * @brief Check for ratio control loop "hunting" (limit cycling).
 *
 * Hunting occurs when the ratio oscillates persistently around
 * the setpoint, often due to stiction in the slave flow control
 * valve or interaction with other control loops.
 *
 * Detection: check if the ratio error sign alternates every sample
 * for N consecutive samples.
 *
 * Returns: estimated oscillation amplitude, or 0 if no hunting detected
 *
 * Knowledge point: Ratio hunting detection — persistent oscillation
 * degrades product quality and increases actuator wear. Early
 * detection enables corrective tuning.
 *
 * Complexity: O(1) per call
 */
double ratio_detect_hunting(double error_k, double error_km1, double error_km2,
                             double error_km3)
{
    int sign_alternates = 1;
    if (error_k * error_km1 >= 0) sign_alternates = 0;
    if (error_km1 * error_km2 >= 0) sign_alternates = 0;
    if (error_km2 * error_km3 >= 0) sign_alternates = 0;

    if (!sign_alternates) return 0.0;

    /* Estimate amplitude as average of absolute errors */
    double amp = (fabs(error_k) + fabs(error_km1) +
                  fabs(error_km2) + fabs(error_km3)) / 4.0;
    return amp;
}

/* =========================================================================
 * L5: Cascade Ratio Control — Outer Loop
 *
 * In cascade ratio control, an outer loop (e.g., temperature or
 * composition controller) adjusts the ratio setpoint, while the
 * inner loop maintains the flow ratio.
 *
 *   Outer (slow):  Quality → adjusts R_sp
 *   Inner (fast):  R_sp → adjusts F_slave
 * ========================================================================= */

/**
 * @brief Compute cascade ratio adjustment.
 *
 * The outer controller output (0-100%) maps to a ratio trim range:
 *   ΔR = (outer_output - 50%) / 50% * R_trim_range
 *
 * This way, 50% output means "no trim" (maintain base ratio),
 * 0% means "minimum ratio", and 100% means "maximum ratio".
 *
 * Knowledge point: Cascade ratio outer loop — the ratio setpoint
 * becomes the manipulated variable for a higher-level quality
 * controller, forming a cascade structure.
 *
 * Complexity: O(1)
 */
double cascade_ratio_adjust(double outer_output, double output_min,
                             double output_max, double R_trim_range)
{
    if (output_max <= output_min) return 0.0;
    double midpoint = (output_min + output_max) / 2.0;
    double half_range = (output_max - output_min) / 2.0;
    double frac = (outer_output - midpoint) / half_range;
    if (frac < -1.0) frac = -1.0;
    if (frac >  1.0) frac =  1.0;
    return frac * R_trim_range;
}

/**
 * @brief Compute the anti-reset-windup measure for ratio cascade.
 *
 * When the slave flow controller saturates (valve fully open or closed),
 * the outer cascade loop must stop adjusting the ratio to prevent windup.
 *
 * Knowledge point: Cascade anti-windup — essential for cascade ratio
 * control stability when the inner loop saturates.
 *
 * Complexity: O(1)
 */
int cascade_anti_windup_check(double slave_output, double slave_output_max,
                               double slave_output_min)
{
    /* Inner loop saturated high: don't increase ratio further */
    if (slave_output >= slave_output_max) return 1;
    /* Inner loop saturated low: don't decrease ratio further */
    if (slave_output <= slave_output_min) return 1;
    return 0;
}
