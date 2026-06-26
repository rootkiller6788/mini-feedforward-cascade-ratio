/**
 * @file smith_types.h
 * @brief Core type definitions for Smith Predictor dead-time compensation.
 *
 * Levels: L1 (Definitions), L2 (Core Concepts), L3 (Engineering Structures)
 *
 * The Smith Predictor (O.J.M. Smith, 1957) separates dead time from the
 * process dynamics to enable controller design for the delay-free portion.
 *
 * Key references:
 *   Smith, O.J.M. (1957) "Closer control of loops with dead time"
 *       Chemical Engineering Progress, 53(5), 217-219
 *   Smith, O.J.M. (1959) "A controller to overcome dead time"
 *       ISA Journal, 6(2), 28-33
 *   Palmor, Z.J. (1996) "Time-delay compensation — Smith predictor and its
 *       modifications" in The Control Handbook, CRC Press, pp. 224-237
 *   Normey-Rico, J.E. & Camacho, E.F. (2007) "Control of Dead-time Processes"
 *       Springer-Verlag, London
 *   Astrom, K.J. & Hagglund, T. (2005) "Advanced PID Control"
 *       Chapter 7: "Dead-Time Compensation"
 *
 * Course mapping:
 *   MIT 6.302: Feedback Systems — Nyquist criterion for delayed systems
 *   Stanford ENGR205: Process Control — dead-time compensation
 *   Berkeley ME233: Advanced Control — Smith predictor structure
 *   CMU 24-677: Adv Ctrl Systems — time-delay systems
 *   Purdue ME 575: Industrial Control — practical dead-time handling
 *   RWTH Aachen: Industrial Control Systems — Smith predictor in PLC
 *   ISA/IEC: dead-time compensation in DCS blocks
 */

#ifndef SMITH_TYPES_H
#define SMITH_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L1: Core Smith Predictor Parameters and Definitions
 *===========================================================================*/

/**
 * @brief Time-delay process model:  G(s) = Gp(s) * e^(-theta*s)
 *
 * The Smith predictor decomposes a process with dead time into:
 *   - A rational delay-free part Gp(s) (typically FOPDT or SOPDT)
 *   - A pure time delay e^(-theta*s)
 *
 * FOPDT (First-Order Plus Dead Time):
 *   G(s) = K * e^(-theta*s) / (tau*s + 1)
 *
 * SOPDT (Second-Order Plus Dead Time):
 *   G(s) = K * e^(-theta*s) / [(tau1*s + 1)*(tau2*s + 1)]
 *
 * SOPDT (underdamped):
 *   G(s) = K * omega_n^2 * e^(-theta*s) / (s^2 + 2*zeta*omega_n*s + omega_n^2)
 */
typedef enum {
    SMITH_MODEL_FOPDT = 0,       /**< First-order plus dead time */
    SMITH_MODEL_SOPDT = 1,       /**< Second-order plus dead time */
    SMITH_MODEL_SOPDT_UNDER = 2, /**< Underdamped second-order plus dead time */
    SMITH_MODEL_IFOPDT = 3,      /**< Integrating first-order plus dead time */
    SMITH_MODEL_HIGH_ORDER = 4   /**< Higher-order plus dead time (user-defined) */
} smith_model_order_t;

/**
 * @brief Discretization method for the digital Smith predictor.
 *
 * The delay-free model Gp(s) must be discretized for digital implementation.
 * Different methods trade off accuracy, stability, and computational cost.
 */
typedef enum {
    SMITH_DISC_EULER_FWD   = 0,  /**< Forward Euler (simple, potential instability) */
    SMITH_DISC_EULER_BWD   = 1,  /**< Backward Euler (stable, first-order) */
    SMITH_DISC_TUSTIN      = 2,  /**< Tustin/Bilinear (recommended, preserves stability) */
    SMITH_DISC_ZOH         = 3,  /**< Zero-Order Hold (exact step-invariant) */
    SMITH_DISC_FOH         = 4   /**< First-Order Hold (ramp-invariant) */
} smith_disc_method_t;

/**
 * @brief Delay approximation method for digital delay realization.
 *
 * Since e^(-theta*s) is infinite-dimensional, digital implementation requires
 * finite approximation of the delay operator z^(-d).
 */
typedef enum {
    SMITH_DELAY_EXACT      = 0,  /**< Exact integer-delay buffer d = floor(theta/Ts) */
    SMITH_DELAY_PADE_1     = 1,  /**< First-order Pade: (1 - s*theta/2)/(1 + s*theta/2) */
    SMITH_DELAY_PADE_2     = 2,  /**< Second-order Pade approximation */
    SMITH_DELAY_FIR        = 3,  /**< FIR filter approximation of fractional delay */
    SMITH_DELAY_LAGRANGE   = 4,  /**< Lagrange interpolation for fractional delay */
    SMITH_DELAY_THIRAN     = 5   /**< Thiran allpass filter for fractional delay */
} smith_delay_approx_t;

/**
 * @brief Smith predictor configuration variants.
 *
 * Different structures address specific weaknesses of the classic Smith predictor:
 * robustness to model mismatch, disturbance rejection, and unstable processes.
 */
typedef enum {
    SMITH_VARIANT_CLASSIC      = 0, /**< O.J.M. Smith, 1957 — original structure */
    SMITH_VARIANT_MODIFIED     = 1, /**< Watanabe & Ito, 1981 — improved disturbance rejection */
    SMITH_VARIANT_FILTERED     = 2, /**< Normey-Rico et al., 1997 — robustness filter */
    SMITH_VARIANT_TWO_DOF      = 3, /**< Two-degree-of-freedom Smith (setpoint + disturbance) */
    SMITH_VARIANT_DISTURB_OBS  = 4, /**< Disturbance-observer-based Smith predictor */
    SMITH_VARIANT_PREDICTIVE   = 5  /**< Prediction-error-based Smith predictor */
} smith_variant_t;

/*===========================================================================
 * L2: Process Model Parameters and State
 *===========================================================================*/

/**
 * @brief FOPDT Process Model Parameters
 *
 * Represents K * e^(-theta*s) / (tau*s + 1)
 *
 * gain (K): static gain — ratio of steady-state output change to input change
 * time_constant (tau): time to reach 63.2% of steady-state response
 * dead_time (theta): transport delay before any response begins
 *
 * Relative dead time: theta/tau determines how "hard" the process is to control.
 *   < 0.1 : easy (PI controller alone often sufficient)
 *   0.1-1.0 : moderate (Smith predictor beneficial)
 *   > 1.0 : difficult (Smith predictor recommended)
 */
typedef struct {
    double K;           /**< Static gain (process output units / controller output units) */
    double tau;         /**< Time constant (seconds) */
    double theta;       /**< Dead time (seconds) */
    double noise_std;   /**< Measurement noise standard deviation estimate */
} smith_fopdt_model_t;

/**
 * @brief SOPDT Process Model Parameters
 *
 * Represents K * e^(-theta*s) / [(tau1*s + 1)*(tau2*s + 1)]
 */
typedef struct {
    double K;           /**< Static gain */
    double tau1;        /**< First (dominant) time constant (seconds) */
    double tau2;        /**< Second time constant (seconds) */
    double theta;       /**< Dead time (seconds) */
    double zeta;        /**< Damping ratio (for underdamped SOPDT, 0 < zeta < 1) */
    double omega_n;     /**< Natural frequency (rad/s, for underdamped SOPDT) */
    int    is_underdamped; /**< Non-zero if zeta < 1 (oscillatory) */
} smith_sopdt_model_t;

/**
 * @brief Unified process model for the Smith predictor.
 *
 * Holds both the actual process model parameters and the delay-free model
 * used internally by the Smith predictor controller.
 */
typedef struct {
    /* Model identification */
    smith_model_order_t   order;        /**< Model order type */
    smith_fopdt_model_t   fopdt;        /**< FOPDT parameters (when order == FOPDT) */
    smith_sopdt_model_t   sopdt;        /**< SOPDT parameters (when order == SOPDT/UNDER) */

    /* Nominal operating point */
    double                u0;           /**< Nominal input (bias) */
    double                y0;           /**< Nominal output (bias) */

    /* Delay-free model for internal controller design */
    double                K_delay_free; /**< Delay-free gain (same as K for FOPDT/SOPDT) */
    double                tau_delay_free; /**< Delay-free dominant time constant */

    /* Model quality metrics */
    double                model_fit;    /**< Normalized RMSE fit [0,1], higher = better */
    double                param_uncertainty_K;    /**< Relative uncertainty in gain */
    double                param_uncertainty_tau;  /**< Relative uncertainty in time constant */
    double                param_uncertainty_theta; /**< Relative uncertainty in dead time */
} smith_process_model_t;

/*===========================================================================
 * L3: Digital Smith Predictor State
 *===========================================================================*/

/**
 * @brief Circular delay buffer for implementing digital delay.
 *
 * Classic Smith predictor requires a digital delay of d = floor(theta/Ts) samples.
 * For fractional delays, interpolation is applied on this buffer.
 *
 * Memory complexity: O(d_max) — a circular buffer of length d_max + 1.
 */
typedef struct {
    double   *buffer;     /**< Ring buffer storing past delayed-free model outputs */
    size_t    capacity;   /**< Buffer size = d_max + 1 + interpolation taps */
    size_t    head;       /**< Write position (index of most recent sample) */
    size_t    delay_int;  /**< Integer delay: d = floor(theta / Ts) */
    double    delay_frac; /**< Fractional delay remainder: theta/Ts - floor(theta/Ts) */
    double    Ts;         /**< Sampling period (seconds) */
} smith_delay_buffer_t;

/**
 * @brief Complete digital Smith Predictor state machine.
 *
 * Implements the block diagram:
 *
 *           ┌─────────┐
 *   r ──+──►│  C(s)   │──+──┬──►│  G(s)  │──► y
 *       │-  └─────────┘  │  │   └────────┘
 *       │                │  │
 *       │   ┌─────────┐  │  │   ┌──────────────┐
 *       └───│Gp̃(s)    │◄─┘  └───│e^(-θ̃s)       │
 *           │(no delay)│         │(delay buffer) │
 *           └─────────┘         └──────────────┘
 *                └──────(+)──────────┘
 *
 * Where C(s) is the primary controller designed for Gp(s) (delay-free).
 * Gp̃(s) is the delay-free model; e^(-θ̃s) is the model delay.
 *
 * The predictor output yp = Gp̃(s)*u is fed back to C(s).
 * The error correction ym - y compensates for model mismatch and disturbance.
 */
typedef struct {
    /* === Configuration (L1-L3) === */
    smith_process_model_t model;        /**< Process model */
    smith_variant_t       variant;      /**< Smith predictor variant */
    smith_disc_method_t   disc_method;  /**< Discretization method */
    smith_delay_approx_t  delay_approx; /**< Delay approximation type */

    /* === Controller parameters (designed for delay-free process) === */
    double Kp;              /**< Primary controller proportional gain */
    double Ti;              /**< Primary controller integral time (s, 0 = no I) */
    double Td;              /**< Primary controller derivative time (s, 0 = no D) */
    double N;               /**< Derivative filter factor */

    /* === Setpoint prefilter (Two-DOF structure) === */
    double b;               /**< Setpoint weight on proportional term */
    double T_ref;           /**< Reference model time constant for setpoint smoothing */

    /* === Robustness filter (Filtered Smith predictor) === */
    double Fr;              /**< Robustness filter time constant (Normey-Rico) */
    double filter_state;    /**< Internal state of robustness filter */

    /* === Sampling and delay === */
    double Ts;              /**< Sampling period (seconds) */
    smith_delay_buffer_t delay_buf; /**< Digital delay buffer */

    /* === Running state (predictor) === */
    double yp_model;       /**< Delay-free model output (prediction) */
    double yp_delayed;     /**< Delayed model output (ym in block diagram) */
    double prediction_error; /**< y - ym (model mismatch indicator) */

    /* === Running state (primary controller) === */
    double integrator;     /**< Integral term accumulator */
    double prev_error;     /**< Previous control error e(k-1) */
    double prev_error2;    /**< Error from two steps ago e(k-2) */
    double prev_output;    /**< Previous controller output u(k-1) */
    double derivative_state; /**< State of derivative filter */

    /* === Delay-free model state === */
    double model_state_1;  /**< First state of delay-free model (FOPDT: single state) */
    double model_state_2;  /**< Second state of delay-free model (SOPDT: extra state) */

    /* === Constraints (L3 Engineering Structures) === */
    double u_min;          /**< Lower actuator saturation limit */
    double u_max;          /**< Upper actuator saturation limit */
    double rate_limit;     /**< Maximum rate of change per sample (0 = disabled) */
    int    saturating;     /**< Flag: 1 if actuator is currently saturated */

    /* === Diagnostics === */
    double IAE;            /**< Integrated Absolute Error metric */
    double ISE;            /**< Integrated Squared Error metric */
    double ITAE;           /**< Integrated Time-weighted Absolute Error */
    uint64_t sample_count; /**< Number of control iterations executed */
} smith_predictor_t;

/*===========================================================================
 * L2: Performance Metrics
 *===========================================================================*/

/**
 * @brief Control loop performance assessment for Smith predictor.
 *
 * Evaluates both setpoint tracking and disturbance rejection capability.
 * Performance is compared against the theoretical best achievable control
 * (minimum-variance control bound).
 */
typedef struct {
    double setpoint_IAE;        /**< IAE for setpoint changes */
    double setpoint_overshoot;  /**< Max overshoot ratio (0 = none) */
    double setpoint_rise_time;  /**< 10%-90% rise time (seconds) */
    double setpoint_settling;   /**< 5% settling time (seconds) */

    double disturbance_IAE;     /**< IAE for load disturbances */
    double disturbance_peak;    /**< Peak deviation during disturbance */
    double disturbance_recovery; /**< Recovery time after disturbance (seconds) */

    double noise_sensitivity;   /**< Output variance due to measurement noise */
    double control_effort_var;  /**< Variance of controller output (actuator wear proxy) */

    double robustness_margin;   /**< Gain margin under model uncertainty (0..inf) */
    double phase_margin;        /**< Phase margin (degrees) */
    double delay_margin;        /**< Additional delay allowed before instability (seconds) */
} smith_performance_t;

/*===========================================================================
 * L5: Model Identification Data Structures
 *===========================================================================*/

/**
 * @brief Step-test data for process identification.
 *
 * Used by open-loop step-response methods to estimate FOPDT/SOPDT
 * parameters for the Smith predictor model.
 */
typedef struct {
    double  *time;          /**< Time vector (seconds) */
    double  *output;        /**< Process output (PV) */
    double  *input;         /**< Process input (CO), includes step change */
    size_t   n_samples;     /**< Number of data points */
    double   input_step;    /**< Magnitude of step change in input */
    double   initial_value; /**< Initial steady-state output before step */
    double   final_value;   /**< Final steady-state output after step */
} smith_step_test_t;

/**
 * @brief Closed-loop identification data for online model update.
 *
 * Collected during normal control operation for adaptive Smith predictor.
 * Uses recursive least squares on the delay-free portion of the process.
 */
typedef struct {
    double   forgetting_factor; /**< RLS forgetting factor (0.95-0.999) */
    double   cov_matrix[4];     /**< 2x2 covariance matrix (flat storage) */
    double   param_vector[2];   /**< Estimated [K, tau] for FOPDT */
    double   theta_estimate;    /**< Estimated dead time */
    int      initialized;       /**< Whether RLS has converged */
    double   prediction_error;  /**< Current prediction error */
} smith_rls_identifier_t;

/*===========================================================================
 * L8: Adaptive Smith Predictor
 *===========================================================================*/

/**
 * @brief Adaptive Smith predictor with online model identification.
 *
 * Uses RLS to update FOPDT model parameters online, then redesigns
 * the primary controller and robustness filter.
 *
 * Reference: Hagglund & Astrom (2002) "Revisiting the Ziegler-Nichols
 * step response method for PID control", J. Process Control
 */
typedef struct {
    smith_predictor_t       predictor;     /**< Embedded Smith predictor */
    smith_rls_identifier_t  identifier;    /**< Online model identifier */
    double                  adaptation_rate; /**< How fast to update (0-1) */
    double                  model_change_threshold; /**< Min change to trigger redesign */
    double                  last_K;        /**< Previous gain for change detection */
    double                  last_tau;      /**< Previous time constant */
    double                  last_theta;    /**< Previous dead time */
    int                     adapt_enabled; /**< Flag: enable/disable adaptation */
    uint64_t                redesign_count; /**< Number of controller redesigns */
} smith_adaptive_t;

/*===========================================================================
 * L7: Industrial Communication Structures
 *===========================================================================*/

/**
 * @brief Modbus register mapping for Smith predictor parameters.
 *
 * Enables integration with PLC/SCADA systems via Modbus RTU/TCP.
 * Each parameter occupies 2 registers (4 bytes, IEEE 754 float).
 */
typedef struct {
    uint16_t Kp_reg;        /**< Modbus holding register base for Kp */
    uint16_t Ti_reg;        /**< Modbus holding register base for Ti */
    uint16_t Td_reg;        /**< Modbus holding register base for Td */
    uint16_t theta_reg;     /**< Modbus holding register base for dead time estimate */
    uint16_t model_K_reg;   /**< Modbus holding register base for model gain */
    uint16_t model_tau_reg; /**< Modbus holding register base for model tau */
    uint16_t yp_reg;        /**< Read-only register for predictor output */
    uint16_t status_reg;    /**< Status word (bit 0: predictor active, bit 1: adapting) */
} smith_modbus_map_t;

/**
 * @brief OPC UA node IDs for Smith predictor variables.
 *
 * Structural mapping for OPC UA server integration (IEC 62541).
 */
typedef struct {
    uint32_t namespace_idx; /**< OPC UA namespace index */
    uint32_t Kp_node;       /**< Node ID for proportional gain */
    uint32_t Ti_node;       /**< Node ID for integral time */
    uint32_t Td_node;       /**< Node ID for derivative time */
    uint32_t theta_node;    /**< Node ID for dead time */
    uint32_t predictor_out_node;  /**< Node ID for predictor output */
    uint32_t mismatch_node; /**< Node ID for model mismatch indicator */
    uint32_t mode_node;     /**< Node ID for predictor mode (auto/manual) */
} smith_opcua_map_t;

#ifdef __cplusplus
}
#endif

#endif /* SMITH_TYPES_H */
