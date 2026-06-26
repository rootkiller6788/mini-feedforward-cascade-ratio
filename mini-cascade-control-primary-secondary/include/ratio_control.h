/**
 * @file ratio_control.h
 * @brief Ratio Control — Flow Ratio, Blending, and Cross-Ratio Strategies
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L1 Definitions, L2 Core Concepts, L3 Engineering Structures
 *
 * Ratio control maintains a specified proportion between two or more process
 * streams. The "wild" (primary/uncontrolled) stream is measured, and the
 * "controlled" (secondary) stream is adjusted to maintain the desired ratio:
 *
 *   R = Q_controlled / Q_wild    (or Q_controlled = R * Q_wild)
 *
 * Applications:
 *   - Fuel-air ratio in combustion (L6: stoichiometric ratio control)
 *   - Reagent stoichiometric ratio in chemical reactors
 *   - Blending two products to specification
 *   - Dilution ratio in wastewater treatment
 *   - Reflux ratio in distillation columns
 *
 * Ratio Control Architectures (L3):
 *   1. Fixed Ratio (R_sp set by operator)
 *   2. Ratio + Trim (R_sp adjusted by feedback from analyzer)
 *   3. Cross-Ratio (two interacting ratio loops, e.g., dual blending)
 *   4. Scheduled Ratio (R_sp varies with throughput or quality)
 *
 * Reference: Seborg, Edgar, Mellichamp (2016) Process Dynamics and Control, Ch. 16
 *            Liptak (2006) Instrument Engineers' Handbook, Vol. 2, Ch. 8
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#ifndef RATIO_CONTROL_H
#define RATIO_CONTROL_H

#include "cascade_types.h"
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Ratio Control Definitions
 * ========================================================================= */

/** Ratio control architecture type */
typedef enum {
    RATIO_ARCH_FIXED        = 0,  /**< Fixed ratio from operator           */
    RATIO_ARCH_TRIM         = 1,  /**< Ratio with feedback trim            */
    RATIO_ARCH_CROSS        = 2,  /**< Cross-ratio (two controlled streams)*/
    RATIO_ARCH_SCHEDULED    = 3,  /**< Scheduled ratio vs production rate  */
    RATIO_ARCH_CASCADE_RATIO = 4  /**< Cascade + ratio hybrid              */
} ratio_architecture_t;

/** Ratio calculation method */
typedef enum {
    RATIO_CALC_LINEAR      = 0,   /**< R = Qc / Qw (simple ratio)         */
    RATIO_CALC_PERCENT     = 1,   /**< R% = Qc/Qw * 100                   */
    RATIO_CALC_BLENDING    = 2,   /**< %A = Qa/(Qa+Qb) * 100              */
    RATIO_CALC_MOLE        = 3    /**< Stoichiometric (with MW factors)    */
} ratio_calc_method_t;

/** Ratio station — the core ratio control block */
typedef struct {
    /* Configuration */
    ratio_architecture_t  architecture;
    ratio_calc_method_t   calc_method;
    double                ratio_sp;         /**< Desired ratio setpoint    */
    double                ratio_pv;         /**< Actual computed ratio     */
    double                ratio_trim;       /**< Trim adjustment from FB  */
    double                ratio_bias;       /**< Static ratio bias        */
    double                wild_flow_min;    /**< Minimum wild flow cutoff  */
    double                wild_flow_max;    /**< Maximum wild flow limit   */
    double                controlled_sp_min; /**< Min controlled flow SP   */
    double                controlled_sp_max; /**< Max controlled flow SP   */
    bool                  ratio_active;     /**< Ratio control active      */
    bool                  trim_active;      /**< Trim feedback active      */
    char                  tag[32];          /**< Instrument tag (e.g., FFIC-201) */

    /* Process signals */
    double                wild_flow;        /**< Measured primary flow     */
    double                controlled_flow;  /**< Measured secondary flow   */
    double                controlled_sp;    /**< Computed secondary SP     */
    double                controlled_pv;    /**< Secondary flow PV          */
    double                analyzer_signal;  /**< Quality analyzer (for trim)*/

    /* Internal state */
    double                accumulated_error; /**< Ratio error integrator   */
    double                last_wild_flow;   /**< Previous wild flow sample */
    double                last_ratio;       /**< Previous computed ratio   */
    uint64_t              sample_count;

    /* Scaling factors */
    double                mw_factor_a;      /**< Molecular weight factor A */
    double                mw_factor_b;      /**< Molecular weight factor B */
    double                density_a;        /**< Density of stream A       */
    double                density_b;        /**< Density of stream B       */
} ratio_station_t;

/** Blending control station (multiple input streams → one blended output) */
#define MAX_BLEND_STREAMS 6
typedef struct {
    ratio_station_t       streams[MAX_BLEND_STREAMS];
    uint32_t              num_streams;
    double                total_flow_sp;     /**< Total production rate    */
    double                total_flow_pv;     /**< Total measured flow      */
    double                blend_property_sp; /**< Target blend property    */
    double                blend_property_pv; /**< Measured blend property  */
    double                blend_gains[MAX_BLEND_STREAMS];
    bool                  total_flow_control;/**< Master flow controller    */
} blend_station_t;

/* =========================================================================
 * L2: Ratio Control Initialization & Configuration
 * ========================================================================= */

/**
 * ratio_init: Initialize a ratio station.
 *
 * Sets default architecture to fixed ratio, ratio SP = 1.0,
 * linear calculation method, and zeroes all state.
 *
 * Complexity: O(1)
 */
void ratio_init(ratio_station_t *ratio, const char *tag);

/**
 * ratio_configure_fixed: Configure for fixed ratio control.
 *
 * The controlled flow setpoint is simply:
 *   Qc_sp = R_sp * Q_wild + bias
 *
 * This is the simplest and most common ratio control scheme.
 * Used when the required ratio is constant and well-known.
 *
 * Complexity: O(1)
 */
void ratio_configure_fixed(ratio_station_t *ratio,
                            double ratio_sp, double bias,
                            double wild_flow_min);

/**
 * ratio_configure_trim: Configure ratio with analyzer trim.
 *
 * The ratio setpoint is adjusted by a slow feedback controller
 * based on product quality (e.g., composition, octane, viscosity):
 *
 *   R_effective = R_sp + K_trim * (quality_sp - quality_pv)
 *
 * Architecture: Quality analyzer → Trim PID → Ratio SP adjustment
 * This is a cascade-like structure where the outer quality loop
 * adjusts the ratio for the inner flow loop.
 */
void ratio_configure_trim(ratio_station_t *ratio,
                           double base_ratio_sp,
                           double trim_gain,
                           double trim_integral_time,
                           double analyzer_sp);

/* =========================================================================
 * L3: Ratio Computation Algorithms
 * ========================================================================= */

/**
 * ratio_compute_linear: Compute ratio using simple linear division.
 *
 *   R = Q_controlled / Q_wild    (if Q_wild > wild_flow_min)
 *   R = last_valid               (if Q_wild below cutoff to avoid blow-up)
 *
 * The wild flow minimum prevents division by zero or extremely
 * large ratios at low flows.
 *
 * @param ratio  Ratio station with current flow measurements
 * @return       Computed ratio value (or last valid if flows invalid)
 *
 * Complexity: O(1)
 */
double ratio_compute_linear(ratio_station_t *ratio);

/**
 * ratio_compute_blend: Compute blend fraction.
 *
 *   %A = Q_a / (Q_a + Q_b) * 100
 *
 * For multi-component blending, computes the volume or mass fraction
 * of each component in the total blend.
 *
 * @param blend       Blend station with all stream measurements
 * @param fractions   Output array of blend fractions [0..1] (caller allocates)
 * @return            Total flow rate, or -1 on error
 *
 * Complexity: O(n) where n = num_streams
 */
double ratio_compute_blend(blend_station_t *blend, double *fractions);

/**
 * ratio_compute_mole: Compute stoichiometric ratio.
 *
 * For chemical reactions, the molar ratio accounts for molecular weights:
 *
 *   R_mole = (Q_c / MW_c) / (Q_w / MW_w) * stoichiometric_coefficient
 *
 * Used for combustion control (fuel/air), neutralization (acid/base),
 * and polymerization (monomer/catalyst).
 *
 * @param ratio           Ratio station with MW factors configured
 * @param stoich_coeff    Stoichiometric coefficient (moles_c / moles_w)
 * @return                Molar ratio
 *
 * Complexity: O(1)
 */
double ratio_compute_mole(ratio_station_t *ratio, double stoich_coeff);

/* =========================================================================
 * L3: Ratio Setpoint Calculation
 * ========================================================================= */

/**
 * ratio_calculate_setpoint: Calculate controlled flow setpoint from ratio.
 *
 * Core ratio equation:
 *   Qc_sp = R_effective * Q_wild + bias
 *
 * where R_effective = ratio_sp + ratio_trim (if trim active)
 *
 * Applies rate-of-change limiting to controlled_sp for smooth operation.
 * Clamps to [controlled_sp_min, controlled_sp_max].
 *
 * @param ratio           Ratio station with all configuration
 * @param max_rate_change Maximum SP change per second (0 = unlimited)
 * @return                Controlled flow setpoint
 *
 * Complexity: O(1)
 */
double ratio_calculate_setpoint(ratio_station_t *ratio,
                                 double max_rate_change);

/**
 * ratio_trim_update: Update the ratio trim from analyzer feedback.
 *
 * Implements PI trim control:
 *   error = analyzer_sp - analyzer_signal
 *   ratio_trim = K_trim * (error + 1/Ti_trim * integral(error) * Ts)
 *
 * The trim is applied slowly (Ti typically 5-30 minutes) to avoid
 * interaction with the faster flow ratio loop.
 *
 * @param ratio     Ratio station with trim configuration
 * @param Ts        Trim controller sample time [seconds]
 * @return          Updated ratio trim value
 *
 * Complexity: O(1)
 */
double ratio_trim_update(ratio_station_t *ratio, double Ts);

/* =========================================================================
 * L5: Cross-Ratio Control for Multiple Streams
 * ========================================================================= */

/**
 * ratio_cross_update: Cross-ratio control for dual-stream blending.
 *
 * When two streams are both ratio-controlled (e.g., A follows wild flow,
 * B follows total flow), cross-ratio logic ensures:
 *
 *   1. Q_A = R_A * Q_wild
 *   2. Q_B = R_B * Q_total = R_B * (Q_wild + Q_A)
 *
 * This maintains both ratio specifications simultaneously.
 * The equations are solved iteratively or algebraically.
 *
 * @param station_a  Ratio station for stream A
 * @param station_b  Ratio station for stream B
 * @param wild_flow  Measured wild flow
 * @return           0 on success, -1 if infeasible
 *
 * Complexity: O(1)
 */
int ratio_cross_update(ratio_station_t *station_a,
                        ratio_station_t *station_b,
                        double wild_flow);

/* =========================================================================
 * L5: Ratio Control Performance & Optimization
 * ========================================================================= */

/**
 * ratio_performance_metrics: Compute ratio control variance metrics.
 *
 * Measures the standard deviation of the actual ratio around
 * the setpoint, as well as the flow variability.
 *
 *   CV_ratio = sigma_ratio / mean_ratio * 100 [%]
 *
 * @param ratio      Ratio station with history
 * @param cv_ratio   Output: coefficient of variation of ratio [%]
 * @param std_ratio  Output: standard deviation of ratio
 * @return           Number of valid samples used
 *
 * Complexity: O(1) — uses running statistics
 */
int ratio_performance_metrics(const ratio_station_t *ratio,
                               double *cv_ratio, double *std_ratio);

/**
 * ratio_optimize_blend: Optimize blend ratios for minimum cost.
 *
 * Given component costs and quality constraints, computes the
 * blend fractions that minimize total cost while meeting
 * specifications. Uses simplex algorithm for linear blending.
 *
 * @param blend          Blend station with costs and constraints
 * @param costs          Array of costs per unit for each stream
 * @param property_coeff Array of property contributions per stream
 * @param prop_min       Minimum required blend property
 * @param prop_max       Maximum required blend property
 * @param optimal        Output optimal fractions array
 * @return               Minimum cost per unit, -1 if infeasible
 *
 * Complexity: O(2^n) worst case (simplex), typically O(n)
 */
double ratio_optimize_blend(blend_station_t *blend,
                             const double *costs,
                             const double *property_coeff,
                             double prop_min, double prop_max,
                             double *optimal);

/**
 * ratio_characterize_flow: Characterize flow measurement for ratio control.
 *
 * Compensates for non-linear flow measurement (e.g., orifice plate
 * square-root relationship):
 *   Q_actual = K * sqrt(ΔP)
 *
 * Linearizes the measurement before ratio computation to ensure
 * accurate ratio control across the full flow range.
 *
 * @param raw_measurement  Raw differential pressure or meter reading
 * @param is_square_root   1 if square-root compensation needed (orifice)
 * @param K                 Meter factor
 * @return                  Corrected flow value
 *
 * Complexity: O(1)
 */
double ratio_characterize_flow(double raw_measurement,
                                int is_square_root, double K);

/**
 * ratio_cascade_setpoint: Ratio control as cascade primary.
 *
 * In a cascade structure, the ratio controller is the outer (primary) loop
 * and a flow controller is the inner (secondary) loop. The ratio
 * controller computes the flow setpoint:
 *
 *   Qc_sp = ratio_sp * Q_wild
 *
 * and sends it as the remote setpoint to the flow controller.
 *
 * @param ratio              Ratio station
 * @param flow_controller    Secondary flow PID controller
 * @param wild_flow          Current wild flow measurement
 * @return                   0 on success
 */
int ratio_cascade_setpoint(ratio_station_t *ratio,
                            cascade_pid_controller_t *flow_controller,
                            double wild_flow);

#ifdef __cplusplus
}
#endif

#endif /* RATIO_CONTROL_H */
