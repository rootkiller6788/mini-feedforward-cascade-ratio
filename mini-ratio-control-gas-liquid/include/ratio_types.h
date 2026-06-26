/**
 * @file ratio_types.h
 * @brief Core type definitions for gas-liquid ratio control systems.
 *
 * Level: L1 Definitions + L3 Engineering Structures
 * Reference: Seborg, Edgar, Mellichamp, "Process Dynamics and Control" (2016), Ch.15
 *            Shinskey, "Process Control Systems" (1996), Ch.7 Ratio Control
 *            ISA-5.1 Instrumentation Symbols and Identification
 *            IEC 61131-3: Programming Industrial Automation
 *
 * This header defines all data types for ratio control of gas-liquid systems:
 * ratio stations, master/slave flow configurations, cross-limiting structures,
 * gas-liquid equilibrium models, and dynamic compensation parameters.
 *
 * Course mapping:
 *   Stanford ENGR205: Process Control — feedforward & ratio control
 *   MIT 6.302: Feedback Systems — cascade & ratio structures
 *   Purdue ME 575: Industrial Control — blending & ratio applications
 *   RWTH Aachen: Industrial Control Systems — combustion ratio control
 */

#ifndef RATIO_TYPES_H
#define RATIO_TYPES_H

#include <stdint.h>
#include <stddef.h>

/*===========================================================================
 * L1: Core Definitions — Ratio Control Fundamentals
 *===========================================================================*/

/**
 * @brief Flow measurement units commonly used in ratio control.
 *
 * MASS_FLOW: kg/s, kg/h — preferred for ratio control (mass conserved)
 * VOLUME_FLOW: m³/s, L/min — requires density compensation
 * MOLAR_FLOW: mol/s, kmol/h — used in reaction engineering
 * NORMALIZED_FLOW: dimensionless [0..1] — normalized to full scale
 */
typedef enum {
    FLOW_UNIT_MASS          = 0,
    FLOW_UNIT_VOLUME        = 1,
    FLOW_UNIT_MOLAR         = 2,
    FLOW_UNIT_NORMALIZED    = 3
} flow_unit_t;

/**
 * @brief Ratio control configuration types.
 *
 * FIXED_RATIO:        R = R_setpoint, no trim (open-loop ratio)
 * WILD_STREAM_TRACK:  Slave tracks master flow Fs = R * Fm (feedforward)
 * TRIMMED_RATIO:      R = R_setpoint + PID_trim(error) (feedback trim)
 * CROSS_LIMITED:      Both flows limit each other (combustion safety)
 * BLENDING_RATIO:     Multiple streams mixed to target composition
 * CASCADE_RATIO:      Outer loop adjusts ratio, inner loop controls flow
 */
typedef enum {
    RATIO_MODE_FIXED           = 0,
    RATIO_MODE_WILD_STREAM     = 1,
    RATIO_MODE_TRIMMED         = 2,
    RATIO_MODE_CROSS_LIMITED   = 3,
    RATIO_MODE_BLENDING        = 4,
    RATIO_MODE_CASCADE         = 5
} ratio_mode_t;

/**
 * @brief Gas-liquid system type classification.
 *
 * GAS_LIQUID_ABSORBER:    Gas absorbed into liquid (scrubber, absorber)
 * GAS_LIQUID_REACTOR:     Gas-liquid chemical reactor (CSTR, bubble column)
 * GAS_LIQUID_STRIPPER:    Gas strips volatile from liquid (stripper)
 * GAS_LIQUID_COMBUSTION:  Air-fuel combustion (boiler, furnace)
 * GAS_LIQUID_EJECTOR:     Gas-liquid ejector/eductor
 * GAS_LIQUID_SEPARATOR:   Gas-liquid separation vessel
 * GAS_LIQUID_PIPELINE:    Two-phase flow in pipelines
 */
typedef enum {
    GL_SYSTEM_ABSORBER    = 0,
    GL_SYSTEM_REACTOR     = 1,
    GL_SYSTEM_STRIPPER    = 2,
    GL_SYSTEM_COMBUSTION  = 3,
    GL_SYSTEM_EJECTOR     = 4,
    GL_SYSTEM_SEPARATOR   = 5,
    GL_SYSTEM_PIPELINE    = 6
} gl_system_type_t;

/**
 * @brief Cross-limiting mode for combustion safety.
 *
 * AIR_LEADS:   Air increase leads fuel increase (rich→lean safe)
 * FUEL_LEADS:  Fuel increase leads air decrease (lean→rich, risk CO)
 * DOUBLE_CROSS: Both air-leads and fuel-leads for maximum safety
 * NONE:        No cross-limiting (direct ratio control)
 */
typedef enum {
    CROSS_LIMIT_NONE         = 0,
    CROSS_LIMIT_AIR_LEADS    = 1,
    CROSS_LIMIT_FUEL_LEADS   = 2,
    CROSS_LIMIT_DOUBLE       = 3
} cross_limit_mode_t;

/*===========================================================================
 * L1: Core Definitions — Stoichiometric & Physical Constants
 *===========================================================================*/

/**
 * @brief Stoichiometric air-fuel ratios for common fuels.
 * Reference: Turns, "An Introduction to Combustion" (2012), Ch.2
 *
 * Stoichiometric AFR = mass of air required per mass of fuel for complete combustion.
 * Values below are kg_air / kg_fuel (mass basis).
 */
typedef struct {
    double natural_gas_afr;     /**< CH4 + 2O2 → CO2 + 2H2O: ~17.2 kg/kg */
    double propane_afr;          /**< C3H8 + 5O2 → 3CO2 + 4H2O: ~15.7 kg/kg */
    double fuel_oil_afr;         /**< ~14.5 kg/kg typical */
    double coal_afr;             /**< ~11.5 kg/kg (bituminous) */
    double hydrogen_afr;         /**< 2H2 + O2 → 2H2O: ~34.3 kg/kg */
    double gasoline_afr;         /**< ~14.7 kg/kg stoichiometric */
    double diesel_afr;           /**< ~14.5 kg/kg */
} stoichiometric_afr_t;

/**
 * @brief Gas-liquid equilibrium parameters.
 * Reference: Henry's Law — at constant temperature, dissolved gas
 * concentration is proportional to partial pressure of gas above liquid.
 *   C = kH * P_gas   (Henry's Law)
 * where kH is the Henry's law constant (mol/(L·atm)).
 */
typedef struct {
    double henry_constant;       /**< kH in mol/(L·atm), temperature-dependent */
    double temperature_k;        /**< Temperature at which kH applies (Kelvin) */
    double gas_partial_pressure; /**< Partial pressure of gas species (atm) */
    double liquid_volume;        /**< Liquid volume (L) */
    double mass_transfer_coeff;  /**< kLa — overall mass transfer coefficient (1/s) */
} gl_equilibrium_t;

/*===========================================================================
 * L2: Core Concepts — Ratio Control Structures
 *===========================================================================*/

/**
 * @brief Ratio setpoint configuration.
 *
 * The ratio R is defined as:
 *   R = (slave flow) / (master flow)
 *
 * For gas-liquid systems:
 *   Gas-liquid ratio (GLR):  R = G_gas / G_liquid  (volume or mass)
 *   Liquid-gas ratio (LGR):  R = G_liquid / G_gas
 *   Air-fuel ratio (AFR):    R = G_air / G_fuel    (mass basis)
 *   Excess air ratio (λ):    λ = actual_AFR / stoichiometric_AFR
 */
typedef struct {
    double      ratio_setpoint;    /**< Desired ratio R_sp (dimensionless) */
    double      ratio_min;         /**< Minimum allowed ratio (safety limit) */
    double      ratio_max;         /**< Maximum allowed ratio (safety limit) */
    double      ratio_deadband;    /**< Ratio change deadband (±) */
    flow_unit_t master_unit;       /**< Master flow measurement unit */
    flow_unit_t slave_unit;        /**< Slave flow measurement unit */
    int         master_is_gas;     /**< 1 if master is gas, 0 if liquid */
    int         slave_is_gas;      /**< 1 if slave is gas, 0 if liquid */
    double      density_master;    /**< Master stream density (kg/m³) for unit conversion */
    double      density_slave;     /**< Slave stream density (kg/m³) for unit conversion */
} ratio_config_t;

/**
 * @brief Flow measurement signal.
 *
 * Industrial flow transmitters output 4-20 mA analog or digital
 * (HART, Profibus PA, Foundation Fieldbus). This struct represents
 * the measured flow value after signal conditioning.
 */
typedef struct {
    double      raw_flow;          /**< Flow value in engineering units */
    double      filtered_flow;     /**< Low-pass filtered flow value */
    double      flow_rate_of_change;/**< dF/dt for trend detection */
    uint64_t    timestamp_ms;      /**< Measurement timestamp */
    int         signal_quality;    /**< Signal quality: 0=bad, 1=uncertain, 2=good */
    double      temperature;       /**< Temperature compensation value (°C) */
    double      pressure;          /**< Pressure compensation value (kPa_g) */
} flow_measurement_t;

/**
 * @brief Master-slave ratio control state.
 *
 * Ratio control architecture:
 *   Master flow (wild stream) ──→ [Ratio Station R] ──→ Slave SP
 *                                                       │
 *   Slave flow ←── [Slave FC] ←── Slave SP ────────────┘
 *
 * The slave flow controller setpoint is computed as:
 *   SP_slave = R_sp * F_master
 *
 * With ratio trim (feedback):
 *   SP_slave = (R_sp + R_trim) * F_master
 * where R_trim is adjusted by a quality/lab measurement.
 */
typedef struct {
    /* Configuration */
    ratio_config_t  config;         /**< Ratio configuration */
    ratio_mode_t    mode;           /**< Ratio control mode */

    /* Master (wild) stream state */
    flow_measurement_t master;      /**< Master flow measurement */

    /* Slave (controlled) stream state */
    flow_measurement_t slave;       /**< Slave flow measurement */

    /* Computed values */
    double      actual_ratio;       /**< Measured ratio: F_slave / F_master */
    double      ratio_error;        /**< R_actual - R_sp (for trim control) */
    double      slave_setpoint;     /**< Computed slave flow setpoint */
    double      ratio_trim;         /**< Trim correction from feedback loop */
    double      trimmed_ratio;      /**< Effective ratio: R_sp + R_trim */

    /* Status */
    int         ratio_valid;        /**< 1 if both flows > 0 and ratio computable */
    int         slave_saturated;    /**< 1 if slave flow at actuator limit */
    int         windup_active;      /**< 1 if ratio trim PID is in windup */
} ratio_control_state_t;

/*===========================================================================
 * L3: Engineering Structures — Cross-Limiting & Process Models
 *===========================================================================*/

/**
 * @brief Cross-limiting selection state for combustion control.
 *
 * Purpose: During load changes, prevents dangerous fuel-rich conditions
 * by constraining air and fuel flow setpoints relative to each other.
 *
 * Air-leads logic (fuel increase):
 *   SP_air = max( demand_air,  fuel_flow * AFR_stoich / r_air )
 *   SP_fuel = min( demand_fuel, air_flow * r_fuel / AFR_stoich )
 *
 * Fuel-leads logic (fuel decrease):
 *   SP_fuel = max( demand_fuel, air_flow * r_fuel / AFR_stoich )
 *   SP_air = min( demand_air,  fuel_flow * AFR_stoich / r_air )
 *
 * Double cross-limit uses both simultaneously with selectors.
 */
typedef struct {
    cross_limit_mode_t  mode;           /**< Cross-limiting mode */
    double              afr_stoich;     /**< Stoichiometric air-fuel ratio (mass) */
    double              r_air_rich;     /**< Air-rich margin ratio (e.g., 1.05 = 5% excess air) */
    double              r_fuel_rich;    /**< Fuel-rich margin ratio (e.g., 0.95 = 5% fuel rich) */
    double              demand_air;     /**< Unconstrained air demand signal */
    double              demand_fuel;    /**< Unconstrained fuel demand signal */
    double              air_flow;       /**< Measured air flow */
    double              fuel_flow;      /**< Measured fuel flow */
    double              sp_air;         /**< Cross-limited air setpoint (output) */
    double              sp_fuel;        /**< Cross-limited fuel setpoint (output) */

    /* Selector state tracking */
    int                 air_high_selected;  /**< 1 if high-select was active on air */
    int                 fuel_low_selected;  /**< 1 if low-select was active on fuel */
    uint64_t            last_update_ms;     /**< Timestamp of last update */
} cross_limiting_t;

/**
 * @brief Dynamic compensation block (lead-lag) for ratio control.
 *
 * When master and slave flow paths have different dynamics,
 * a dynamic compensator aligns their responses:
 *
 *   G_comp(s) = K * (T_lead * s + 1) / (T_lag * s + 1)
 *
 * In discrete time (Tustin/bilinear transform):
 *   y(k) = a1*y(k-1) + b0*x(k) + b1*x(k-1)
 *
 * where:
 *   denom = 2*T_lag + Ts
 *   a1 = (2*T_lag - Ts) / denom
 *   b0 = (2*K*T_lead + K*Ts) / denom
 *   b1 = (-2*K*T_lead + K*Ts) / denom
 *
 * Reference: Shinskey, "Process Control Systems" (1996), Ch.7
 */
typedef struct {
    double  gain;           /**< Static gain K */
    double  T_lead;         /**< Lead time constant (seconds) */
    double  T_lag;          /**< Lag time constant (seconds) */
    double  Ts;             /**< Sampling period (seconds) */

    /* Discrete coefficients */
    double  a1;             /**< y(k-1) coefficient */
    double  b0;             /**< x(k) coefficient */
    double  b1;             /**< x(k-1) coefficient */

    /* State */
    double  prev_input;     /**< x(k-1) */
    double  prev_output;    /**< y(k-1) */
    int     initialized;    /**< 1 after first update */
} lead_lag_compensator_t;

/**
 * @brief Ratio trim PI controller structure.
 *
 * A slow PI controller that adjusts the ratio setpoint based on
 * product quality or composition feedback.
 *
 *   R_trim(k) = Kp_trim * e_quality(k) + Ki_trim * sum(e_quality)
 *
 * The quality error drives the trim:
 *   e_quality = Q_sp - Q_measured
 *
 * where Q can be oxygen % in flue gas, product composition, pH, etc.
 */
typedef struct {
    double      Kp_trim;        /**< Trim proportional gain */
    double      Ti_trim;        /**< Trim integral time (seconds, 0=disable) */
    double      Ts;             /**< Trim sampling period (seconds) */
    double      trim_min;       /**< Minimum trim output */
    double      trim_max;       /**< Maximum trim output */
    double      trim_dz;        /**< Dead zone: |error| < dz → trim frozen */

    /* State */
    double      integrator;     /**< Trim integral accumulator */
    double      prev_error;     /**< Previous quality error */
    double      output;         /**< Current trim output */
    int         saturated;      /**< 1 if trim at limit */
} ratio_trim_controller_t;

/**
 * @brief Multi-component blending ratio specification.
 *
 * For blending N components:
 *   sum(F_i) = F_total
 *   F_i / F_total = x_i (volume or mass fraction)
 *   sum(x_i) = 1
 *
 * Each component i has a target fraction and acceptable range.
 */
typedef struct {
    double      target_fraction;    /**< Target volume/mass fraction x_i */
    double      fraction_min;       /**< Minimum allowed fraction */
    double      fraction_max;       /**< Maximum allowed fraction */
    double      density;            /**< Component density (kg/m³) */
    double      cost_per_unit;      /**< Component cost per unit mass or volume */
    char        name[32];           /**< Component name/ID */
} blend_component_t;

/**
 * @brief Blending system configuration.
 */
typedef struct {
    blend_component_t *components;  /**< Array of blend components */
    int             n_components;   /**< Number of components (>= 2) */
    double          total_flow_sp;  /**< Total product flow setpoint */
    double          total_flow;     /**< Measured total product flow */
    double          tolerance_pct;  /**< Blend tolerance (±%) */
    flow_unit_t     flow_unit;      /**< Flow measurement unit */
    int             use_cost_optim; /**< 1 to optimize for minimum cost */
} blending_config_t;

/*===========================================================================
 * L4: Engineering Laws — Gas-Liquid Process Parameters
 *===========================================================================*/

/**
 * @brief Ideal gas state for density compensation.
 *
 * Ideal Gas Law: PV = nRT
 *   ρ = P * M / (R * T)
 *
 * where:
 *   ρ = density (kg/m³)
 *   P = absolute pressure (Pa)
 *   M = molar mass (kg/mol)
 *   R = 8.314462618 J/(mol·K) — universal gas constant
 *   T = absolute temperature (K)
 *
 * For real gases, compressibility factor Z is applied:
 *   ρ = P * M / (Z * R * T)
 *
 * Reference: Poling, Prausnitz, O'Connell, "Properties of Gases and Liquids" (2001)
 */
typedef struct {
    double      molar_mass;       /**< Molar mass (kg/mol) */
    double      compressibility;  /**< Compressibility factor Z (≈1 for ideal gas) */
    double      pressure_pa;      /**< Absolute pressure (Pa) */
    double      temperature_k;    /**< Absolute temperature (K) */
    double      density;          /**< Computed density (kg/m³) */
    int         use_real_gas;     /**< 1 to apply compressibility correction */
} gas_state_t;

/**
 * @brief Liquid density compensation parameters.
 *
 * Temperature correction (linear approximation):
 *   ρ(T) = ρ_ref * (1 - β * (T - T_ref))
 *
 * where β is the volumetric thermal expansion coefficient (1/K).
 *
 * For water: β ≈ 2.1e-4 /K at 20°C
 * For organic liquids: β ≈ 1e-3 /K
 *
 * Reference: Perry's Chemical Engineers' Handbook (9th Ed), Sec.2
 */
typedef struct {
    double      density_ref;      /**< Reference density at T_ref (kg/m³) */
    double      temp_ref;         /**< Reference temperature (°C) */
    double      expansion_coeff;  /**< Volumetric thermal expansion coefficient β (1/K) */
    double      temperature_c;    /**< Current temperature (°C) */
    double      density;          /**< Compensated density (kg/m³) */
} liquid_density_t;

/**
 * @brief Two-phase gas-liquid flow parameters.
 *
 * Gas volume fraction (void fraction):
 *   α = Q_gas / (Q_gas + Q_liquid)
 *
 * Lockhart-Martinelli parameter:
 *   X = (ΔP_liquid / ΔP_gas)^(1/2)
 *
 * Reference: Lockhart & Martinelli (1949), "Proposed Correlation of Data
 *            for Isothermal Two-Phase, Two-Component Flow in Pipes"
 *            Chemical Engineering Progress, 45(1), 39-48
 */
typedef struct {
    double      gas_flow_vol;     /**< Gas volumetric flow (m³/s) */
    double      liquid_flow_vol;  /**< Liquid volumetric flow (m³/s) */
    double      void_fraction;    /**< Gas volume fraction α */
    double      slip_ratio;       /**< Gas-to-liquid velocity ratio */
    double      lockhart_martinelli; /**< X parameter */
    double      flow_regime;      /**< Flow regime: 0=bubble, 1=slug, 2=annular, 3=mist */
} two_phase_flow_t;

/*===========================================================================
 * L4: Engineering Laws — Performance & Safety Parameters
 *===========================================================================*/

/**
 * @brief Combustion efficiency parameters.
 *
 * Excess air ratio λ = actual_AFR / AFR_stoich
 *
 * Optimal λ ranges:
 *   Gas burners:    λ = 1.1 - 1.2  (10-20% excess air)
 *   Oil burners:    λ = 1.15 - 1.3 (15-30% excess air)
 *   Coal burners:   λ = 1.2 - 1.5  (20-50% excess air)
 *   Gas turbines:   λ = 2.5 - 4.0  (lean combustion for NOx control)
 *
 * CO in flue gas > 400 ppm indicates incomplete combustion (fuel-rich).
 * O2 in flue gas: typically 2-3% for optimal gas combustion.
 *
 * Reference: Baukal, "Industrial Burners Handbook" (2004)
 */
typedef struct {
    double      lambda_excess_air; /**< Excess air ratio λ */
    double      actual_afr;       /**< Actual air-fuel ratio */
    double      o2_flue_gas_pct;  /**< O2% in dry flue gas */
    double      co_flue_gas_ppm;  /**< CO concentration (ppm) */
    double      co2_flue_gas_pct; /**< CO2% in dry flue gas */
    double      nox_flue_gas_ppm; /**< NOx concentration (ppm) */
    double      stack_temp_c;     /**< Flue gas stack temperature (°C) */
    double      combustion_efficiency_pct; /**< Calculated efficiency */
    int         fuel_rich_alarm;  /**< 1 if CO > threshold (incomplete combustion) */
} combustion_efficiency_t;

/**
 * @brief Gas absorption efficiency parameters.
 *
 * Absorption efficiency:
 *   η = (C_in - C_out) / C_in * 100%
 *
 * Number of transfer units (NTU):
 *   NTU = kLa * V / Q_liquid
 *
 * Height of transfer unit (HTU):
 *   HTU = H_column / NTU
 *
 * Reference: Treybal, "Mass-Transfer Operations" (1980), Ch.8
 */
typedef struct {
    double      kLa;               /**< Overall mass transfer coefficient (1/s) */
    double      liquid_flow;       /**< Liquid flow rate (m³/s) */
    double      gas_flow;          /**< Gas flow rate (m³/s) */
    double      column_volume;     /**< Column/contactor volume (m³) */
    double      column_height;     /**< Column height (m) */
    double      ntu;               /**< Number of transfer units */
    double      htu;               /**< Height of transfer unit (m) */
    double      cin_ppm;           /**< Inlet gas concentration (ppm) */
    double      cout_ppm;          /**< Outlet gas concentration (ppm) */
    double      efficiency_pct;    /**< Removal efficiency (%) */
} absorption_efficiency_t;

/*===========================================================================
 * L8: Advanced — Adaptive & Optimization Structures
 *===========================================================================*/

/**
 * @brief Adaptive ratio trim using recursive least squares (RLS).
 *
 * RLS for online identification of process gain between ratio change
 * and quality response:
 *
 *   θ(k) = θ(k-1) + K(k) * [y(k) - φ(k)·θ(k-1)]
 *   K(k) = P(k-1)·φ(k) / [λ + φ(k)·P(k-1)·φ(k)]
 *   P(k) = [P(k-1) - K(k)·φ(k)·P(k-1)] / λ
 *
 * where:
 *   θ = estimated gain vector
 *   φ = regressor (input history)
 *   λ = forgetting factor (0.95-0.995)
 *
 * Reference: Ljung, "System Identification" (1999), Ch.11
 */
typedef struct {
    double      forgetting_factor; /**< λ: RLS forgetting factor */
    double      theta[4];          /**< Parameter estimate vector θ */
    double      P[4][4];           /**< Covariance matrix P */
    double      phi[4];            /**< Regressor vector φ */
    int         n_params;          /**< Number of parameters (≤ 4) */
    int         initialized;       /**< 1 after initial data collection */
    double      prediction_error;  /**< y - φ·θ */
} rls_identifier_t;

/**
 * @brief Economic optimization for blending ratio control.
 *
 * Minimizes: J = sum(c_i * F_i)  subject to:
 *   sum(F_i) = F_total
 *   F_i >= 0
 *   x_i_min ≤ F_i/F_total ≤ x_i_max
 *   sum(x_i) = 1
 *   Quality constraints: Q_min ≤ Q(product) ≤ Q_max
 *
 * This is a linear programming problem when quality constraints are linear.
 *
 * Reference: Edgar, Himmelblau, Lasdon, "Optimization of Chemical Processes" (2001), Ch.7
 */
typedef struct {
    double      *component_costs;  /**< Cost per unit of each component */
    double      *component_flows;  /**< Optimized flow for each component */
    double      *quality_coeffs;   /**< Quality contribution coefficients */
    int         n_components;      /**< Number of blend components */
    double      total_flow;        /**< Required total flow */
    double      quality_min;       /**< Minimum product quality */
    double      quality_max;       /**< Maximum product quality */
    double      optimal_cost;      /**< Optimal total cost per unit time */
    int         feasible;          /**< 1 if feasible solution found */
} blend_optimizer_t;

#endif /* RATIO_TYPES_H */
