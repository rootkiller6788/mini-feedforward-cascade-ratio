/**
 * @file test_mimo.c
 * @brief Comprehensive test suite for MIMO decoupling control module.
 *
 * Tests: L1 (model structs), L2 (operations), L3 (evaluation/state-space),
 *        L4 (RGA/NI/Routh-Hurwitz/controllability), L5 (matrix inv/SVD/poles/pairing),
 *        L6 (Wood-Berry/blending), L8 (Lyapunov/Monte Carlo).
 */

#include "mimo_model.h"
#include "mimo_decoupling_common.h"
#include "mimo_interaction.h"
#include "mimo_static_decoupling.h"
#include "mimo_dynamic_decoupling.h"
#include "mimo_inverted_decoupling.h"
#include "mimo_svd_decoupling.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static int passed = 0, failed = 0;

#define TEST(n) do { printf("  TEST: %s ... ", n); } while(0)
#define PASS() do { printf("PASSED\n"); passed++; } while(0)
#define FAIL(m) do { printf("FAILED: %s\n", m); failed++; return; } while(0)
#define CHECK(cond, msg) if(!(cond)) { FAIL(msg); }
#define CHECK_FEQ(a,b,tol,msg) if(fabs((a)-(b))>(tol)){printf("FAILED: %s (got %.6f exp %.6f)\n",msg,a,b);failed++;return;}

static void test_model_init(void) {
    TEST("Model init");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "test");
    CHECK(m.num_outputs == 2 && m.num_inputs == 2, "dims");
    CHECK(strcmp(m.name, "test") == 0, "name");
    PASS();
}

static void test_fopdt(void) {
    TEST("FOPDT setting");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "f");
    mimo_model_set_fopdt(&m, 0, 0, 2.5, 10.0, 3.0);
    MIMOTransferFunction *tf = &m.rows[0].elements[0];
    CHECK_FEQ(tf->gain, 2.5, 1e-10, "gain");
    CHECK_FEQ(tf->time_constant, 10.0, 1e-10, "tau");
    CHECK_FEQ(tf->time_delay, 3.0, 1e-10, "theta");
    PASS();
}

static void test_sopdt(void) {
    TEST("SOPDT setting");
    MIMOModel m;
    mimo_model_init(&m, 1, 1, "s");
    mimo_model_set_sopdt(&m, 0, 0, 1.5, 5.0, 0.7, 2.0);
    CHECK_FEQ(m.rows[0].elements[0].gain, 1.5, 1e-10, "gain");
    CHECK(m.rows[0].elements[0].den_order == 2, "den_order");
    PASS();
}

static void test_tf_eval(void) {
    TEST("TF evaluate at s=0");
    MIMOModel m;
    mimo_model_init(&m, 1, 1, "e");
    mimo_model_set_fopdt(&m, 0, 0, 3.0, 4.0, 0.0);
    double complex G0 = mimo_tf_evaluate(&m.rows[0].elements[0], 0.0+0.0*I);
    CHECK_FEQ(creal(G0), 3.0, 1e-10, "G(0)");
    CHECK_FEQ(cimag(G0), 0.0, 1e-10, "imag");
    PASS();
}

static void test_ss_gain(void) {
    TEST("Steady-state gain matrix");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "ss");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    double K[4];
    mimo_model_steady_state_gain(&m, K);
    CHECK_FEQ(K[0], 12.8, 1e-10, "K00");
    CHECK_FEQ(K[3], -19.4, 1e-10, "K11");
    PASS();
}

static void test_rga(void) {
    TEST("RGA (Wood-Berry)");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "wb");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    double K[4];
    mimo_model_steady_state_gain(&m, K);
    RGAMatrix rga;
    CHECK(mimo_rga_compute(K, 2, &rga) == 0, "RGA compute OK");
    CHECK(rga.rga[0][0] > 1.5, "strong interaction");
    double rs0 = rga.rga[0][0] + rga.rga[0][1];
    CHECK_FEQ(rs0, 1.0, 1e-6, "row sum = 1");
    PASS();
}

static void test_ni(void) {
    TEST("Niederlinski Index");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "ni");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    double K[4];
    mimo_model_steady_state_gain(&m, K);
    int pair[] = {0, 1};
    double ni = mimo_niederlinski_index(K, 2, pair);
    CHECK(ni > 0, "NI positive for Wood-Berry");
    PASS();
}

static void test_controllability(void) {
    TEST("Controllability/observability");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "co");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    MIMOStateSpace ss;
    mimo_model_to_state_space(&m, &ss);
    CHECK(ss.n_states > 0, "has states");
    CHECK(mimo_ss_is_controllable(&ss), "controllable");
    CHECK(mimo_ss_is_observable(&ss), "observable");
    PASS();
}

static void test_tustin(void) {
    TEST("Tustin discretization");
    MIMOModel m;
    mimo_model_init(&m, 1, 1, "tu");
    mimo_model_set_fopdt(&m, 0, 0, 1.0, 1.0, 0.0);
    MIMOStateSpace sc, sd;
    mimo_model_to_state_space(&m, &sc);
    mimo_ss_c2d_tustin(&sc, &sd, 0.1);
    CHECK(sd.is_discrete, "discrete flag");
    CHECK_FEQ(sd.sample_time, 0.1, 1e-10, "Ts");
    PASS();
}

static void test_routh(void) {
    TEST("Routh-Hurwitz");
    double den_s[] = {2.0, 3.0, 1.0}; /* s^2+3s+2 */
    double routh[12];
    CHECK(mimo_routh_hurwitz(den_s, 2, routh) == 0, "stable: 0 changes");
    double den_u[] = {-2.0, -1.0, 1.0}; /* s^2-s-2 */
    CHECK(mimo_routh_hurwitz(den_u, 2, routh) > 0, "unstable: sign changes");
    PASS();
}

static void test_poles(void) {
    TEST("Pole finding");
    double den[] = {2.0, 3.0, 1.0};
    double complex poles[4];
    int nr = mimo_find_poles(den, 2, poles);
    CHECK(nr == 2, "2 real poles");
    double p1 = creal(poles[0]), p2 = creal(poles[1]);
    CHECK((fabs(p1+1)<1e-6 && fabs(p2+2)<1e-6) ||
          (fabs(p1+2)<1e-6 && fabs(p2+1)<1e-6), "roots -1,-2");
    PASS();
}

static void test_pairing(void) {
    TEST("Pairing enumeration");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "pr");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    double K[4];
    mimo_model_steady_state_gain(&m, K);
    PairingSet ps;
    int n_pairs = mimo_enumerate_pairings(K, 2, &ps);
    CHECK(n_pairs >= 1, "at least 1 valid pairing");
    CHECK(ps.best_index >= 0, "best found");
    PASS();
}

static void test_decoupler_init(void) {
    TEST("Decoupler init");
    Decoupler D;
    decoupler_init(&D, 2, 2, DECOUPLER_STATIC);
    CHECK(D.n_inputs == 2 && D.n_outputs == 2, "dims");
    CHECK(D.elements[0][0].gain == 1.0, "diag=1");
    CHECK(D.elements[0][1].gain == 0.0, "off-diag=0");
    PASS();
}

static void test_decoupler_proper(void) {
    TEST("Decoupler properness");
    Decoupler D;
    decoupler_init(&D, 2, 2, DECOUPLER_STATIC);
    CHECK(decoupler_is_proper(&D), "static is proper");
    D.elements[0][0].num_order = 2;
    D.elements[0][0].den_order = 1;
    CHECK(!decoupler_is_proper(&D), "improper detected");
    PASS();
}

static void test_static_dec(void) {
    TEST("Static decoupler D=K^{-1}");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "sd");
    mimo_model_set_fopdt(&m, 0, 0, 2.0, 5.0, 0.0);
    mimo_model_set_fopdt(&m, 0, 1, 0.5, 3.0, 0.0);
    mimo_model_set_fopdt(&m, 1, 0, 0.3, 4.0, 0.0);
    mimo_model_set_fopdt(&m, 1, 1, 1.5, 6.0, 0.0);
    StaticDecoupler sd;
    CHECK(mimo_static_decoupler_design(&m, &sd) == 0, "design OK");
    double Ka[4];
    mimo_static_apparent_gain(&m, &sd, Ka);
    CHECK_FEQ(Ka[0], 1.0, 1e-6, "K*Kinv[0,0]=1");
    CHECK_FEQ(Ka[3], 1.0, 1e-6, "K*Kinv[1,1]=1");
    CHECK(fabs(Ka[1]) < 1e-6, "off-diag~0");
    PASS();
}

static void test_svd(void) {
    TEST("SVD decomposition");
    double K[] = {3.0, 1.0, 1.0, 2.0};
    SVDDecoupler sd;
    CHECK(mimo_svd_decompose(K, 2, &sd) == 0, "SVD OK");
    CHECK(sd.Sigma[0] > 0 && sd.Sigma[1] > 0, "positive");
    CHECK(sd.Sigma[0] >= sd.Sigma[1], "sorted");
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            double r = 0.0;
            for (int k = 0; k < 2; k++)
                r += sd.U[i][k] * sd.Sigma[k] * sd.V[j][k];
            CHECK_FEQ(r, K[i*2+j], 1e-6, "reconstruct");
        }
    PASS();
}

static void test_inverted_dec(void) {
    TEST("Inverted decoupler");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "id");
    mimo_model_set_fopdt(&m, 0, 0, 2.0, 5.0, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, 0.5, 3.0, 2.0);
    mimo_model_set_fopdt(&m, 1, 0, 0.3, 4.0, 1.0);
    mimo_model_set_fopdt(&m, 1, 1, 1.5, 6.0, 2.0);
    InvertedDecoupler id;
    CHECK(mimo_inverted_decoupler_design(&m, &id) == 0, "design OK");
    CHECK(mimo_inverted_no_algebraic_loop(&id), "no alg loop");
    double uc[] = {1.0, 0.0}, up[2];
    mimo_inverted_decoupler_step(&id, uc, up);
    CHECK(!isnan(up[0]) && !isnan(up[1]), "finite output");
    PASS();
}

static void test_dynamic_dec(void) {
    TEST("Dynamic decoupler");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "dd");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    DynamicDecoupler dd;
    CHECK(mimo_ideal_dynamic_decoupler(&m, &dd) == 0, "design OK");
    CHECK_FEQ(dd.base.elements[0][0].gain, 1.0, 1e-6, "D11=1");
    PASS();
}

static void test_wood_berry(void) {
    TEST("Wood-Berry model");
    MIMOModel m;
    mimo_wood_berry_model(&m);
    CHECK(m.num_outputs == 2 && m.num_inputs == 2, "dims");
    CHECK_FEQ(m.rows[0].elements[0].gain, 12.8, 1e-10, "G11");
    CHECK_FEQ(m.rows[1].elements[1].gain, -19.4, 1e-10, "G22");
    PASS();
}

static void test_blending(void) {
    TEST("Blending process model");
    MIMOModel m;
    mimo_blending_process_model(&m);
    CHECK(m.num_outputs == 2, "dims");
    CHECK_FEQ(m.rows[0].elements[0].gain, 1.0, 1e-10, "G11");
    CHECK_FEQ(m.rows[1].elements[1].gain, -0.2, 1e-10, "G22");
    PASS();
}

static void test_lyapunov(void) {
    TEST("Lyapunov stability");
    double A1[] = {-2.0, 0.0, 0.0, -3.0};
    double P[4];
    CHECK(mimo_lyapunov_stability(A1, 2, P), "stable system");
    double A2[] = {1.0, 0.0, 0.0, -1.0};
    CHECK(!mimo_lyapunov_stability(A2, 2, P), "unstable system");
    PASS();
}

static void test_monte_carlo(void) {
    TEST("Monte Carlo robustness");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "mc");
    mimo_model_set_fopdt(&m, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&m, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&m, 1, 1, -19.4, 14.4, 3.0);
    double Kc[] = {0.5, -0.1};
    double r = mimo_monte_carlo_robustness(&m, Kc, 100, 0.10);
    CHECK(r >= 0.0 && r <= 1.0, "valid range");
    PASS();
}

static void test_iq(void) {
    TEST("Interaction quotient");
    MIMOModel m;
    mimo_model_init(&m, 2, 2, "iq");
    mimo_model_set_fopdt(&m, 0, 0, 2.0, 1.0, 0.0);
    mimo_model_set_fopdt(&m, 0, 1, 0.1, 1.0, 0.0);
    mimo_model_set_fopdt(&m, 1, 0, 0.1, 1.0, 0.0);
    mimo_model_set_fopdt(&m, 1, 1, 1.5, 1.0, 0.0);
    double K[4];
    mimo_model_steady_state_gain(&m, K);
    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    double iq = mimo_interaction_quotient(&rga);
    CHECK(iq >= 0.0 && iq <= 1.0, "IQ range");
    PASS();
}

int main(void) {
    printf("=== MIMO Decoupling Control Test Suite ===\n\n");
    test_model_init();
    test_fopdt();
    test_sopdt();
    test_tf_eval();
    test_ss_gain();
    test_rga();
    test_ni();
    test_controllability();
    test_tustin();
    test_routh();
    test_poles();
    test_pairing();
    test_decoupler_init();
    test_decoupler_proper();
    test_static_dec();
    test_svd();
    test_inverted_dec();
    test_dynamic_dec();
    test_wood_berry();
    test_blending();
    test_lyapunov();
    test_monte_carlo();
    test_iq();
    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
