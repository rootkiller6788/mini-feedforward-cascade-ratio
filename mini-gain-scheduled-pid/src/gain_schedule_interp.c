#include "gain_schedule_interp.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int gs_interp_bracket_index(const double *x, uint32_t n, double xq) {
    if (!x || n < 2) return -1;
    if (xq <= x[0]) return 0;
    if (xq >= x[n-1]) return (int)(n - 2);
    uint32_t lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (x[mid] <= xq) lo = mid;
        else hi = mid;
    }
    return (int)lo;
}

double gs_interp_nearest(const double *x, const double *y,
                         uint32_t n, double xq) {
    if (!x || !y || n == 0) return 0.0;
    if (n == 1) return y[0];
    if (xq <= x[0]) return y[0];
    if (xq >= x[n-1]) return y[n-1];
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) return y[0];
    if ((double)idx < (double)(n-1) &&
        (xq - x[idx]) > (x[idx+1] - xq)) {
        return y[idx+1];
    }
    return y[idx];
}

double gs_interp_linear(const double *x, const double *y,
                        uint32_t n, double xq) {
    if (!x || !y || n == 0) return 0.0;
    if (n == 1) return y[0];
    if (xq <= x[0]) return y[0];
    if (xq >= x[n-1]) return y[n-1];
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) return y[0];
    uint32_t i = (uint32_t)idx;
    double dx = x[i+1] - x[i];
    if (dx == 0.0) return y[i];
    double t = (xq - x[i]) / dx;
    return y[i] * (1.0 - t) + y[i+1] * t;
}

double gs_interp_cubic_hermite(const double *x, const double *y,
                               uint32_t n, double xq) {
    if (!x || !y || n < 2) return (n == 1 && y) ? y[0] : 0.0;
    if (xq <= x[0]) return y[0];
    if (xq >= x[n-1]) return y[n-1];
    
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) return y[0];
    uint32_t i = (uint32_t)idx;
    
    double dx = x[i+1] - x[i];
    if (dx == 0.0) return y[i];
    double t = (xq - x[i]) / dx;
    
    /* Compute slopes using central differences with one-sided fallback */
    double m_i, m_ip1;
    if (i == 0) {
        m_i = (y[1] - y[0]) / (x[1] - x[0]);
    } else if (i == n - 2) {
        m_i = (y[n-1] - y[n-2]) / (x[n-1] - x[n-2]);
    } else {
        double d_left  = (y[i] - y[i-1]) / (x[i] - x[i-1]);
        double d_right = (y[i+1] - y[i]) / (x[i+1] - x[i]);
        m_i = (d_left + d_right) / 2.0;
    }
    
    if (i + 1 >= n - 1) {
        m_ip1 = (y[n-1] - y[n-2]) / (x[n-1] - x[n-2]);
    } else if (i + 1 == 0) {
        m_ip1 = (y[1] - y[0]) / (x[1] - x[0]);
    } else {
        double d_left2  = (y[i+1] - y[i]) / dx;
        double d_right2 = (y[i+2] - y[i+1]) / (x[i+2] - x[i+1]);
        m_ip1 = (d_left2 + d_right2) / 2.0;
    }
    
    /* Fritsch-Carlson monotonicity constraint */
    double delta = (y[i+1] - y[i]) / dx;
    if (delta == 0.0) {
        m_i = 0.0;
        m_ip1 = 0.0;
    } else {
        double alpha_i = m_i / delta;
        double alpha_ip1 = m_ip1 / delta;
        if (alpha_i < 0.0) m_i = 0.0;
        if (alpha_ip1 < 0.0) m_ip1 = 0.0;
        if (alpha_i > 3.0) m_i = 3.0 * delta;
        if (alpha_ip1 > 3.0) m_ip1 = 3.0 * delta;
    }
    
    double h00 = (1.0 + 2.0*t) * (1.0 - t) * (1.0 - t);
    double h10 = t * (1.0 - t) * (1.0 - t) * dx;
    double h01 = t * t * (3.0 - 2.0*t);
    double h11 = t * t * (t - 1.0) * dx;
    
    return h00 * y[i] + h10 * m_i + h01 * y[i+1] + h11 * m_ip1;
}

bool gs_spline_compute_moments(const double *x, const double *y,
                               uint32_t n, double *M) {
    if (!x || !y || !M || n < 2) return false;
    
    double *a = (double*)malloc(n * sizeof(double));
    double *b = (double*)malloc(n * sizeof(double));
    double *c = (double*)malloc(n * sizeof(double));
    double *d = (double*)malloc(n * sizeof(double));
    if (!a || !b || !c || !d) {
        free(a); free(b); free(c); free(d);
        return false;
    }
    
    /* Natural spline: M[0] = M[n-1] = 0 */
    M[0] = 0.0;
    M[n-1] = 0.0;
    
    if (n == 2) {
        M[0] = 0.0; M[1] = 0.0;
        free(a); free(b); free(c); free(d);
        return true;
    }
    
    for (uint32_t i = 1; i < n - 1; i++) {
        double hi  = x[i] - x[i-1];
        double hip = x[i+1] - x[i];
        if (hi == 0.0 || hip == 0.0) {
            free(a); free(b); free(c); free(d);
            return false;
        }
        a[i] = hi / (hi + hip);
        b[i] = 2.0;
        c[i] = hip / (hi + hip);
        d[i] = 6.0 * ((y[i+1]-y[i])/hip - (y[i]-y[i-1])/hi) / (hi + hip);
    }
    
    /* Thomas algorithm for tridiagonal system */
    for (uint32_t i = 2; i < n - 1; i++) {
        double w = a[i] / b[i-1];
        b[i] -= w * c[i-1];
        d[i] -= w * d[i-1];
    }
    
    M[n-2] = d[n-2] / b[n-2];
    for (int32_t i = (int32_t)n - 3; i >= 1; i--) {
        M[i] = (d[i] - c[i] * M[i+1]) / b[i];
    }
    M[0] = 0.0;
    M[n-1] = 0.0;
    
    free(a); free(b); free(c); free(d);
    return true;
}

double gs_interp_cubic_spline(const double *x, const double *y,
                              uint32_t n, double xq) {
    if (!x || !y || n < 2) return (n == 1 && y) ? y[0] : 0.0;
    if (xq <= x[0]) return y[0];
    if (xq >= x[n-1]) return y[n-1];
    
    double *M = (double*)malloc(n * sizeof(double));
    if (!M) return gs_interp_linear(x, y, n, xq);
    
    if (!gs_spline_compute_moments(x, y, n, M)) {
        free(M);
        return gs_interp_linear(x, y, n, xq);
    }
    
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) { free(M); return y[0]; }
    uint32_t i = (uint32_t)idx;
    
    double hi = x[i+1] - x[i];
    if (hi == 0.0) { free(M); return y[i]; }
    
    double A = (x[i+1] - xq) / hi;
    double B = (xq - x[i])   / hi;
    double C = (A*A*A - A) * hi * hi / 6.0;
    double D = (B*B*B - B) * hi * hi / 6.0;
    
    double result = A * y[i] + B * y[i+1] + C * M[i] + D * M[i+1];
    free(M);
    return result;
}

double gs_interp_lagrange(const double *x, const double *y,
                          uint32_t n, double xq, uint32_t order) {
    if (!x || !y || n == 0) return 0.0;
    if (n == 1 || order < 1) return y[0];
    
    if (order > n - 1) order = (uint32_t)(n - 1);
    if (order < 1) order = 1;
    
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) return y[0];
    
    int32_t start = idx - (int32_t)(order / 2);
    if (start < 0) start = 0;
    if (start + (int32_t)order >= (int32_t)n)
        start = (int32_t)n - 1 - (int32_t)order;
    if (start < 0) start = 0;
    
    double result = 0.0;
    for (uint32_t j = 0; j <= order; j++) {
        double Lj = 1.0;
        uint32_t ij = (uint32_t)(start + (int32_t)j);
        for (uint32_t k = 0; k <= order; k++) {
            if (k == j) continue;
            uint32_t ik = (uint32_t)(start + (int32_t)k);
            double denom = x[ij] - x[ik];
            if (fabs(denom) < 1e-15) continue;
            Lj *= (xq - x[ik]) / denom;
        }
        result += Lj * y[ij];
    }
    return result;
}

double gs_interp_akima(const double *x, const double *y,
                       uint32_t n, double xq) {
    if (!x || !y || n < 2) return (n == 1 && y) ? y[0] : 0.0;
    if (n < 3) return gs_interp_linear(x, y, n, xq);
    if (xq <= x[0]) return y[0];
    if (xq >= x[n-1]) return y[n-1];
    
    double *m = (double*)malloc(n * sizeof(double));
    if (!m) return gs_interp_linear(x, y, n, xq);
    
    for (uint32_t i = 0; i < n; i++) {
        if (i == 0) {
            m[i] = (y[1] - y[0]) / (x[1] - x[0]);
        } else if (i == n - 1) {
            m[i] = (y[n-1] - y[n-2]) / (x[n-1] - x[n-2]);
        } else {
            double d1 = (y[i] - y[i-1]) / (x[i] - x[i-1]);
            double d2 = (y[i+1] - y[i]) / (x[i+1] - x[i]);
            double w1 = fabs(d2 - d1);
            double w2 = 0.0;
            if (i < n - 2) {
                double d3 = (y[i+2] - y[i+1]) / (x[i+2] - x[i+1]);
                w2 = fabs(d3 - d2);
            } else if (i == 1) {
                w2 = fabs(d2 - d1);
            } else {
                w2 = fabs(d2 - d1);
            }
            if (w1 + w2 < 1e-15) {
                m[i] = (d1 + d2) / 2.0;
            } else {
                m[i] = (w1 * d1 + w2 * d2) / (w1 + w2);
            }
        }
    }
    
    int idx = gs_interp_bracket_index(x, n, xq);
    if (idx < 0) { free(m); return y[0]; }
    uint32_t i = (uint32_t)idx;
    
    double dx = x[i+1] - x[i];
    if (dx == 0.0) { free(m); return y[i]; }
    double t = (xq - x[i]) / dx;
    
    double t2 = t * t;
    double t3 = t2 * t;
    
    double p0 = 2.0*t3 - 3.0*t2 + 1.0;
    double p1 = t3 - 2.0*t2 + t;
    double p2 = -2.0*t3 + 3.0*t2;
    double p3 = t3 - t2;
    
    double result = p0*y[i] + p1*dx*m[i] + p2*y[i+1] + p3*dx*m[i+1];
    free(m);
    return result;
}

double gs_interp_gaussian_rbf(const double *x, const double *y,
                              uint32_t n, double xq, double sigma) {
    if (!x || !y || n == 0) return 0.0;
    if (sigma <= 0.0) sigma = 1.0;
    
    double numer = 0.0;
    double denom = 0.0;
    
    for (uint32_t i = 0; i < n; i++) {
        double r2 = (xq - x[i]) * (xq - x[i]);
        double phi = exp(-r2 / (2.0 * sigma * sigma));
        numer += phi * y[i];
        denom += phi;
    }
    
    if (denom < 1e-15) {
        double sum = 0.0;
        for (uint32_t i = 0; i < n; i++) sum += y[i];
        return sum / (double)n;
    }
    return numer / denom;
}

double gs_interp_dispatch(const double *x, const double *y,
                          uint32_t n, double xq,
                          interp_method_t method,
                          double sigma) {
    switch (method) {
        case INTERP_NEAREST_NEIGHBOR:
            return gs_interp_nearest(x, y, n, xq);
        case INTERP_LINEAR:
            return gs_interp_linear(x, y, n, xq);
        case INTERP_CUBIC_HERMITE:
            return gs_interp_cubic_hermite(x, y, n, xq);
        case INTERP_CUBIC_SPLINE:
            return gs_interp_cubic_spline(x, y, n, xq);
        case INTERP_POLYNOMIAL_LAGRANGE:
            return gs_interp_lagrange(x, y, n, xq, 3);
        case INTERP_AKIMA:
            return gs_interp_akima(x, y, n, xq);
        case INTERP_GAUSSIAN_RBF:
            return gs_interp_gaussian_rbf(x, y, n, xq, sigma);
        default:
            return gs_interp_linear(x, y, n, xq);
    }
}

pid_gain_set_t gs_interp_bilinear_2d(
    const schedule_entry_2d_t *grid,
    uint32_t rows, uint32_t cols,
    double xq1, double xq2) {
    
    pid_gain_set_t result;
    memset(&result, 0, sizeof(result));
    result.Kp = 0.1; result.Ki = 0.01;
    
    if (!grid || rows < 2 || cols < 2) return result;
    
    double x1_min = grid[0].sched_val_1;
    double x1_max = grid[0].sched_val_1;
    double x2_min = grid[0].sched_val_2;
    double x2_max = grid[0].sched_val_2;
    
    for (uint32_t i = 0; i < rows * cols; i++) {
        if (grid[i].sched_val_1 < x1_min) x1_min = grid[i].sched_val_1;
        if (grid[i].sched_val_1 > x1_max) x1_max = grid[i].sched_val_1;
        if (grid[i].sched_val_2 < x2_min) x2_min = grid[i].sched_val_2;
        if (grid[i].sched_val_2 > x2_max) x2_max = grid[i].sched_val_2;
    }
    
    if (xq1 < x1_min) xq1 = x1_min;
    if (xq1 > x1_max) xq1 = x1_max;
    if (xq2 < x2_min) xq2 = x2_min;
    if (xq2 > x2_max) xq2 = x2_max;
    
    double dx1 = x1_max - x1_min;
    double dx2 = x2_max - x2_min;
    if (dx1 == 0.0 || dx2 == 0.0) return result;
    
    double t1 = (xq1 - x1_min) / dx1;
    double t2 = (xq2 - x2_min) / dx2;
    
    uint32_t i00 = 0;
    uint32_t i10 = (rows - 1) * cols;
    uint32_t i01 = cols - 1;
    uint32_t i11 = rows * cols - 1;
    
    result.Kp = (1-t1)*(1-t2)*grid[i00].gains.Kp + t1*(1-t2)*grid[i10].gains.Kp
              + (1-t1)*t2*grid[i01].gains.Kp + t1*t2*grid[i11].gains.Kp;
    result.Ki = (1-t1)*(1-t2)*grid[i00].gains.Ki + t1*(1-t2)*grid[i10].gains.Ki
              + (1-t1)*t2*grid[i01].gains.Ki + t1*t2*grid[i11].gains.Ki;
    result.Kd = (1-t1)*(1-t2)*grid[i00].gains.Kd + t1*(1-t2)*grid[i10].gains.Kd
              + (1-t1)*t2*grid[i01].gains.Kd + t1*t2*grid[i11].gains.Kd;
    return result;
}

void gs_table_extract_xy(const gain_schedule_table_t *table,
                         int gain_param,
                         double *x, double *y, uint32_t *n) {
    if (!table || !x || !y || !n) return;
    uint32_t count = table->num_entries;
    if (count > GS_MAX_BREAKPOINTS) count = GS_MAX_BREAKPOINTS;
    *n = count;
    
    for (uint32_t i = 0; i < count; i++) {
        x[i] = table->entries[i].scheduling_value;
        const pid_gain_set_t *g = &table->entries[i].gains;
        switch (gain_param) {
            case 0: y[i] = g->Kp; break;
            case 1: y[i] = g->Ki; break;
            case 2: y[i] = g->Kd; break;
            case 3: y[i] = g->Ti; break;
            case 4: y[i] = g->Td; break;
            case 5: y[i] = g->N;  break;
            case 6: y[i] = g->b;  break;
            case 7: y[i] = g->c;  break;
            default: y[i] = 0.0; break;
        }
    }
}

pid_gain_set_t gs_table_interpolate_gains(
    const gain_schedule_table_t *table,
    double sched_val) {
    
    pid_gain_set_t result;
    memset(&result, 0, sizeof(result));
    result.Kp = table->default_Kp;
    result.Ki = table->default_Ki;
    result.Kd = table->default_Kd;
    result.Ti = 0.0;
    result.Td = 0.0;
    result.N  = 10.0;
    result.b  = 1.0;
    result.c  = 0.0;
    
    if (!table || table->num_entries == 0) return result;
    if (table->num_entries == 1) return table->entries[0].gains;
    
    uint32_t n = table->num_entries;
    double *xv = (double*)malloc(n * sizeof(double));
    double *yv = (double*)malloc(n * sizeof(double));
    if (!xv || !yv) {
        free(xv); free(yv);
        return result;
    }
    
    uint32_t count;
    
    gs_table_extract_xy(table, 0, xv, yv, &count);
    result.Kp = gs_interp_dispatch(xv, yv, count, sched_val,
                                    table->interp_method, 1.0);
    
    gs_table_extract_xy(table, 1, xv, yv, &count);
    result.Ki = gs_interp_dispatch(xv, yv, count, sched_val,
                                    table->interp_method, 1.0);
    
    gs_table_extract_xy(table, 2, xv, yv, &count);
    result.Kd = gs_interp_dispatch(xv, yv, count, sched_val,
                                    table->interp_method, 1.0);
    
    gs_table_extract_xy(table, 3, xv, yv, &count);
    result.Ti = gs_interp_dispatch(xv, yv, count, sched_val,
                                    table->interp_method, 1.0);
    
    gs_table_extract_xy(table, 4, xv, yv, &count);
    result.Td = gs_interp_dispatch(xv, yv, count, sched_val,
                                    table->interp_method, 1.0);
    
    gs_table_extract_xy(table, 5, xv, yv, &count);
    result.N = gs_interp_dispatch(xv, yv, count, sched_val,
                                   table->interp_method, 1.0);
    
    if (result.Kp < 0.0) result.Kp = table->default_Kp;
    if (result.Ki < 0.0) result.Ki = table->default_Ki;
    if (result.Kd < 0.0) result.Kd = table->default_Kd;
    
    free(xv); free(yv);
    return result;
}
