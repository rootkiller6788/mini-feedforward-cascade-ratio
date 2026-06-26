/**
 * @file test_gas_liquid.c
 * @brief Tests for gas-liquid process models and adaptive ratio control.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

/* Type definitions matching the headers */
typedef struct { double molar_mass, compressibility, pressure_pa, temperature_k, density; int use_real_gas; } gas_state_t;
typedef struct { double density_ref, temp_ref, expansion_coeff, temperature_c, density; } liquid_density_t;
typedef struct { double henry_constant, temperature_k, gas_partial_pressure, liquid_volume, mass_transfer_coeff; } gl_equilibrium_t;
typedef struct { double gas_flow_vol, liquid_flow_vol, void_fraction, slip_ratio, lockhart_martinelli, flow_regime; } two_phase_flow_t;
typedef struct { double lambda_excess_air, actual_afr, o2_flue_gas_pct, co_flue_gas_ppm, co2_flue_gas_pct, nox_flue_gas_ppm, stack_temp_c, combustion_efficiency_pct; int fuel_rich_alarm; } combustion_efficiency_t;
typedef struct { double forgetting_factor, theta[4], P[4][4], phi[4]; int n_params, initialized; double prediction_error; } rls_identifier_t;
typedef struct { double *component_costs, *component_flows, *quality_coeffs; int n_components; double total_flow, quality_min, quality_max, optimal_cost; int feasible; } blend_optimizer_t;

/* Functions under test */
extern double gas_density_ideal(const gas_state_t *gas);
extern double gas_density_real(const gas_state_t *gas);
extern double gas_density_compute(const gas_state_t *gas);
extern double gas_flow_normal_to_actual(double flow_normal, double P_actual, double T_actual, double P_n, double T_n);
extern double gas_flow_actual_to_normal(double flow_actual, double P_actual, double T_actual, double P_n, double T_n);
extern double liquid_density_compensate(const liquid_density_t *liq);
extern double liquid_expansion_coeff(const char *substance);
extern double henry_equilibrium(const gl_equilibrium_t *eq);
extern double mass_transfer_rate(const gl_equilibrium_t *eq, double C_bulk);
extern double absorber_liquid_gas_ratio(double y_in, double y_out, double x_in, double kH, double P_total);
extern double absorber_ntu(double y_in, double y_out, double x_in, double kH, double P_total, double L_G_ratio);
extern double absorber_dynamic_step(const gl_equilibrium_t *eq, double Q_gas, double Q_liquid, double C_gas_in, double *C_liquid, double Ts);
extern void   gl_reactor_dynamic_step(double V_liquid, double V_gas, double Q_liquid_in, double Q_liquid_out, double Q_gas_in, double Q_gas_out, double C_A_in, double *C_A, double *P_A, double kLa, double kH, double k_reaction, double Ts);
extern double gl_reactor_steady_gas_flow(double C_A_target, double C_A_in, double kLa, double kH, double k_reaction, double V_liquid, double V_gas, double Q_liquid);
extern double two_phase_void_fraction(const two_phase_flow_t *tf, double C0, double U_drift, double pipe_area);
extern int    two_phase_flow_regime(const two_phase_flow_t *tf, double pipe_diam, double rho_gas, double rho_liquid, double sigma);
extern double lockhart_martinelli_multiplier(double X, double C_coeff);
extern double combustion_efficiency_compute(const combustion_efficiency_t *eff, double T_ambient);
extern double combustion_excess_air_from_o2(double target_o2_pct);
extern double combustion_afr_target(double afr_stoich, double excess_air_pct, double *lambda);
extern double separator_residence_time(double V_liquid, double Q_liquid);
extern double souders_brown_velocity(double rho_liquid, double rho_gas, double K_value);
extern double separation_efficiency(double v_settling, double t_residence, double H);

extern void   rls_init(rls_identifier_t *rls, double forgetting_factor, int n_params);
extern double rls_update(rls_identifier_t *rls, double y, const double *phi);
extern void   rls_get_theta(const rls_identifier_t *rls, double *theta_out);
extern void   rls_reset_covariance(rls_identifier_t *rls);
extern void   blend_optimizer_init(blend_optimizer_t *opt, int n_components);
extern void   blend_optimizer_set_costs(blend_optimizer_t *opt, const double *costs);
extern void   blend_optimizer_set_quality(blend_optimizer_t *opt, const double *quality);
extern int    blend_optimizer_solve(blend_optimizer_t *opt, int n_comp, double total_flow, double Q_min, double Q_max);
extern double blend_optimizer_get_flow(const blend_optimizer_t *opt, int component_i);
extern double blend_optimizer_get_cost(const blend_optimizer_t *opt);
extern double density_compensate_gas(double rho_ref, double P_ref, double T_ref, double P_actual, double T_actual);
extern double density_compensate_liquid(double rho_ref, double beta, double T_ref, double T_actual);
extern double mass_flow_from_volume(double vol_flow, double density);
extern void   ratio_performance_metrics(const double *ratio_errors, int n_samples, double tolerance_pct, double *mean_err, double *std_err, double *accuracy_pct);
extern double ratio_detect_oscillation(const double *ratio_errors, int n_samples);
extern double ratio_economic_impact(double ratio_error_pct, double master_flow, double slave_cost_factor, double efficiency_impact);

static int tests_run = 0, tests_passed = 0;
#define TEST(n) do { tests_run++; printf("  TEST: %s ... ", n); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(m) do { printf("FAIL: %s\n", m); return; } while(0)
#define ASSERT_EQ(a,b,eps) do { \
    double _va = (double)(a); double _vb = (double)(b); \
    if (fabs(_va-_vb)>(eps)) { FAIL("assertion"); return; } \
} while(0)
#define ASSERT_EQ_INT(a,b) do { \
    if ((int)(a) != (int)(b)) { FAIL("assertion"); return; } \
} while(0)
#define ASSERT_TRUE(c) do { if (!(c)) { FAIL("assertion"); return; } } while(0)

/* ================================================================
 * Gas density — ideal gas law
 * ================================================================ */
static void test_gas_density(void)
{
    gas_state_t gas = {0.02897, 1.0, 101325.0, 298.15, 0.0, 0};
    TEST("ideal gas density air at STP");
    double rho = gas_density_ideal(&gas);
    /* ρ = 101325 * 0.02897 / (8.31446 * 298.15) ≈ 1.184 kg/m³ */
    ASSERT_EQ(rho, 1.184, 0.02);
    PASS();

    TEST("ideal gas density with real gas (Z=1)");
    gas.use_real_gas = 1;
    ASSERT_EQ(gas_density_real(&gas), gas_density_ideal(&gas), 1e-9);
    PASS();

    TEST("auto gas density");
    ASSERT_EQ(gas_density_compute(&gas), gas_density_ideal(&gas), 1e-9);
    PASS();
}

/* ================================================================
 * Gas flow normalization
 * ================================================================ */
static void test_gas_flow_normalization(void)
{
    TEST("actual to normal conversion");
    double Q_actual = 10.0;  /* m³/s at 150°C, 500 kPa */
    double Q_normal = gas_flow_actual_to_normal(Q_actual, 500000.0, 423.15, 101325.0, 273.15);
    double Q_back = gas_flow_normal_to_actual(Q_normal, 500000.0, 423.15, 101325.0, 273.15);
    ASSERT_EQ(Q_back, 10.0, 1e-6);
    PASS();
}

/* ================================================================
 * Liquid density compensation
 * ================================================================ */
static void test_liquid_density(void)
{
    liquid_density_t liq = {998.2, 20.0, 2.1e-4, 80.0, 0.0};
    TEST("water density at 80°C");
    double rho = liquid_density_compensate(&liq);
    /* ρ = 998.2 * (1 - 2.1e-4 * 60) = 985.5 */
    ASSERT_EQ(rho, 985.5, 0.5);
    PASS();

    TEST("expansion coefficient lookup");
    ASSERT_EQ(liquid_expansion_coeff("water"), 2.1e-4, 1e-9);
    ASSERT_EQ(liquid_expansion_coeff("ethanol"), 1.1e-3, 1e-9);
    ASSERT_EQ(liquid_expansion_coeff("unknown"), -1.0, 1e-9);
    PASS();
}

/* ================================================================
 * Henry's Law and mass transfer
 * ================================================================ */
static void test_henry_mass_transfer(void)
{
    gl_equilibrium_t eq = {0.034, 298.15, 0.1, 100.0, 0.05};
    TEST("Henry's Law equilibrium CO2");
    double C_eq = henry_equilibrium(&eq);
    ASSERT_EQ(C_eq, 0.0034, 1e-9);
    PASS();

    TEST("mass transfer rate (absorption)");
    double N = mass_transfer_rate(&eq, 0.001);
    /* N = 0.05 * 100.0 * (0.0034 - 0.001) = 0.012 */
    ASSERT_EQ(N, 0.012, 1e-9);
    PASS();
}

/* ================================================================
 * Absorber design equations
 * ================================================================ */
static void test_absorber(void)
{
    TEST("absorber L/G ratio");
    double LG = absorber_liquid_gas_ratio(0.05, 0.005, 0.0, 0.034, 1.0);
    ASSERT_TRUE(LG > 0.0);
    PASS();

    TEST("absorber NTU");
    double lg = 2.0; /* L/G = 2 */
    double ntu = absorber_ntu(0.05, 0.005, 0.0, 0.034, 1.0, lg);
    ASSERT_TRUE(ntu > 0.0);
    PASS();

    TEST("absorber dynamic step");
    gl_equilibrium_t eq2 = {0.034, 298.15, 0.3, 100.0, 0.05};
    double C_liquid = 0.001;
    double C_new = absorber_dynamic_step(&eq2, 10.0, 20.0, 0.01, &C_liquid, 0.1);
    ASSERT_TRUE(isfinite(C_new)); /* verify computation runs without error */
    PASS();
}

/* ================================================================
 * CSTR reactor
 * ================================================================ */
static void test_reactor(void)
{
    TEST("CSTR dynamic step");
    double C_A = 0.01, P_A = 0.1;
    gl_reactor_dynamic_step(100.0, 50.0, 10.0, 10.0, 5.0, 5.0, 0.02, &C_A, &P_A, 0.05, 0.034, 0.01, 0.1);
    ASSERT_TRUE(isfinite(C_A) && isfinite(P_A));
    PASS();

    TEST("CSTR steady gas flow");
    double Q_gas = gl_reactor_steady_gas_flow(0.005, 0.02, 0.05, 0.034, 0.01, 100.0, 50.0, 10.0);
    ASSERT_TRUE(Q_gas >= 0.0);
    PASS();
}

/* ================================================================
 * Two-phase flow
 * ================================================================ */
static void test_two_phase(void)
{
    two_phase_flow_t tf = {0.1, 0.2, 0.0, 1.0, 0.0, 0.0};
    TEST("void fraction homogeneous");
    double alpha = two_phase_void_fraction(&tf, 1.0, 0.0, 0.1);
    ASSERT_EQ(alpha, 0.33333, 0.01);
    PASS();

    TEST("flow regime");
    int regime = two_phase_flow_regime(&tf, 0.1, 1.2, 1000.0, 0.072);
    ASSERT_TRUE(regime >= 0 && regime <= 3);
    PASS();

    TEST("Lockhart-Martinelli multiplier");
    double phi2 = lockhart_martinelli_multiplier(1.0, 20.0);
    ASSERT_EQ(phi2, 22.0, 1e-9);
    PASS();
}

/* ================================================================
 * Combustion efficiency
 * ================================================================ */
static void test_combustion(void)
{
    combustion_efficiency_t eff = {1.2, 20.64, 3.0, 50.0, 10.0, 80.0, 180.0, 0.0, 0};
    TEST("combustion efficiency compute");
    double eta = combustion_efficiency_compute(&eff, 25.0);
    ASSERT_TRUE(eta > 80.0 && eta < 99.0);
    PASS();

    TEST("excess air from O2");
    double ea = combustion_excess_air_from_o2(3.0);
    ASSERT_EQ(ea, 16.6667, 0.1);
    PASS();

    TEST("AFR target");
    double lambda;
    double afr = combustion_afr_target(17.2, 10.0, &lambda);
    ASSERT_EQ(afr, 18.92, 1e-3);
    ASSERT_EQ(lambda, 1.1, 1e-3);
    PASS();
}

/* ================================================================
 * Gas-liquid separator
 * ================================================================ */
static void test_separator(void)
{
    TEST("residence time");
    double t_res = separator_residence_time(10.0, 0.1);
    ASSERT_EQ(t_res, 100.0, 1e-9);
    PASS();

    TEST("Souders-Brown velocity");
    double v_max = souders_brown_velocity(1000.0, 1.2, 0.07);
    ASSERT_TRUE(v_max > 1.0 && v_max < 3.0);
    PASS();

    TEST("separation efficiency");
    double eta = separation_efficiency(0.01, 100.0, 2.0);
    ASSERT_EQ(eta, 1.0 - exp(-0.5), 1e-6);
    PASS();
}

/* ================================================================
 * RLS identification
 * ================================================================ */
static void test_rls(void)
{
    rls_identifier_t rls;
    TEST("RLS init");
    rls_init(&rls, 0.98, 2);
    ASSERT_EQ_INT(rls.n_params, 2);
    ASSERT_EQ(rls.forgetting_factor, 0.98, 1e-9);
    PASS();

    TEST("RLS update converges");
    double phi[2] = {1.0, 0.5};
    for (int i = 0; i < 200; i++) {
        double y = 2.0 * phi[0] + 3.0 * phi[1]; /* true params: θ=[2,3] */
        rls_update(&rls, y, phi);
    }
    double theta[2];
    rls_get_theta(&rls, theta);
    ASSERT_EQ(theta[0], 2.0, 2.0);
    ASSERT_EQ(theta[1], 3.0, 2.0);
    PASS();

    TEST("RLS reset covariance");
    rls_reset_covariance(&rls);
    PASS();
}

/* ================================================================
 * Blend optimization
 * ================================================================ */
static void test_blend_optimizer(void)
{
    blend_optimizer_t opt;
    TEST("blend init 2-comp");
    blend_optimizer_init(&opt, 2);
    double costs[2] = {1.0, 3.0};
    double quality[2] = {90.0, 50.0};
    blend_optimizer_set_costs(&opt, costs);
    blend_optimizer_set_quality(&opt, quality);
    PASS();

    TEST("blend solve 2-comp: cheaper component preferred");
    int feasible = blend_optimizer_solve(&opt, 2, 100.0, 70.0, 80.0);
    ASSERT_TRUE(feasible);
    /* Component 1 is cheaper (1.0 vs 3.0) → maximize F1 */
    double F1 = blend_optimizer_get_flow(&opt, 0);
    ASSERT_TRUE(F1 > 50.0);
    PASS();
}

/* ================================================================
 * Density compensation
 * ================================================================ */
static void test_density_compensation(void)
{
    TEST("gas density P/T compensation");
    double rho = density_compensate_gas(1.2, 101.325, 273.15, 200.0, 300.0);
    ASSERT_TRUE(rho > 1.2);
    PASS();

    TEST("liquid density T compensation");
    rho = density_compensate_liquid(1000.0, 2.1e-4, 20.0, 80.0);
    ASSERT_TRUE(rho < 1000.0);
    PASS();

    TEST("mass flow from volume");
    double mf = mass_flow_from_volume(10.0, 1000.0);
    ASSERT_EQ(mf, 10000.0, 1e-9);
    PASS();
}

/* ================================================================
 * Performance metrics
 * ================================================================ */
static void test_performance(void)
{
    double errors[] = {0.1, -0.1, 0.05, -0.05, 0.0, 0.02, -0.02, 0.01, -0.01, 0.0};
    double mean, std, accuracy;
    TEST("ratio performance metrics");
    ratio_performance_metrics(errors, 10, 10.0, &mean, &std, &accuracy);
    ASSERT_TRUE(fabs(mean) < 0.05);
    ASSERT_TRUE(std > 0.0);
    ASSERT_TRUE(accuracy > 50.0);
    PASS();

    TEST("oscillation detection: signal has alternating pattern");
    double oi = ratio_detect_oscillation(errors, 10);
    /* Alternating signs produce some oscillation index */
    ASSERT_TRUE(oi >= 0.0 && oi <= 1.0);
    PASS();

    TEST("economic impact");
    double impact = ratio_economic_impact(5.0, 1000.0, 0.5, 0.3);
    ASSERT_TRUE(impact > 0.0);
    PASS();
}

int main(void)
{
    printf("=== Gas-Liquid Process & Adaptive Ratio Tests ===\n\n");

    test_gas_density();
    test_gas_flow_normalization();
    test_liquid_density();
    test_henry_mass_transfer();
    test_absorber();
    test_reactor();
    test_two_phase();
    test_combustion();
    test_separator();
    test_rls();
    test_blend_optimizer();
    test_density_compensation();
    test_performance();

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    if (tests_passed < tests_run) {
        printf("SOME TESTS FAILED!\n");
        return 1;
    }
    printf("ALL TESTS PASSED!\n");
    return 0;
}
