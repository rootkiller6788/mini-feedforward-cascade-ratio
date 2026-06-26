#ifndef GAIN_SCHEDULE_INTERP_H
#define GAIN_SCHEDULE_INTERP_H

#include "gain_schedule_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    gain_schedule_interp.h
 * @brief   Interpolation Methods for Gain Scheduling L3/L5
 *
 * Provides multiple interpolation strategies for computing PID gains
 * between scheduled breakpoints. Supported methods include nearest-neighbor,
 * linear, cubic Hermite (monotone-preserving), cubic spline (C2 smooth),
 * Lagrange polynomial, Akima spline, and Gaussian RBF.
 *
 * Each method provides specific guarantees:
 *   - Linear: monotonicity-preserving, continuous (C0)
 *   - Cubic Hermite: monotonicity-preserving, continuous derivative (C1)
 *   - Cubic Spline: natural or clamped, continuous 2nd derivative (C2)
 *   - Akima: locally determined, oscillation-free
 *   - Gaussian RBF: smooth multidimensional, global support
 *
 * References:
 *   Fritsch & Carlson, "Monotone Piecewise Cubic Interpolation", SINUM, 1980.
 *   Akima, "A New Method of Interpolation...", JACM, 1970.
 *   de Boor, "A Practical Guide to Splines", Springer, 1978.
 */

double gs_interp_nearest(const double *x, const double *y,
                         uint32_t n, double xq);

double gs_interp_linear(const double *x, const double *y,
                        uint32_t n, double xq);

double gs_interp_cubic_hermite(const double *x, const double *y,
                               uint32_t n, double xq);

double gs_interp_cubic_spline(const double *x, const double *y,
                              uint32_t n, double xq);

double gs_interp_lagrange(const double *x, const double *y,
                          uint32_t n, double xq, uint32_t order);

double gs_interp_akima(const double *x, const double *y,
                       uint32_t n, double xq);

double gs_interp_gaussian_rbf(const double *x, const double *y,
                              uint32_t n, double xq, double sigma);

/**
 * Compute cubic spline second derivatives for natural boundary conditions.
 * Solves a tridiagonal system via Thomas algorithm. O(n).
 */
bool gs_spline_compute_moments(const double *x, const double *y,
                               uint32_t n, double *M);

/**
 * Generic 1D interpolation dispatcher.
 */
double gs_interp_dispatch(const double *x, const double *y,
                          uint32_t n, double xq,
                          interp_method_t method,
                          double sigma);

int gs_interp_bracket_index(const double *x, uint32_t n, double xq);

/**
 * 2D bilinear interpolation on a rectangular grid.
 * Row-major: g[i][j] = gains[row_offset*i + j]
 */
pid_gain_set_t gs_interp_bilinear_2d(
    const schedule_entry_2d_t *grid,
    uint32_t rows, uint32_t cols,
    double xq1, double xq2);

/**
 * Extract x (scheduling values) and y (gain values) arrays from a
 * schedule table for a specific gain parameter.
 * gain_param: 0=Kp, 1=Ki, 2=Kd, 3=Ti, 4=Td, 5=N, 6=b, 7=c
 */
void gs_table_extract_xy(const gain_schedule_table_t *table,
                         int gain_param,
                         double *x, double *y, uint32_t *n);

/**
 * Interpolate all PID gains from a 1D schedule table using the
 * configured interpolation method.
 */
pid_gain_set_t gs_table_interpolate_gains(
    const gain_schedule_table_t *table,
    double sched_val);

#ifdef __cplusplus
}
#endif
#endif /* GAIN_SCHEDULE_INTERP_H */
