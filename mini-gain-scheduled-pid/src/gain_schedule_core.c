#include "gain_schedule_core.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

void gs_table_init(gain_schedule_table_t *table, sched_var_type_t svt) {
    if (!table) return;
    memset(table, 0, sizeof(gain_schedule_table_t));
    table->sched_var_type = svt;
    table->interp_method = INTERP_LINEAR;
    table->num_entries = 0;
    table->sched_min = +1e300;
    table->sched_max = -1e300;
    table->hysteresis_band = 0.0;
    table->extrapolate_low = false;
    table->extrapolate_high = false;
    table->default_Kp = 1.0;
    table->default_Ki = 0.1;
    table->default_Kd = 0.0;
}

bool gs_table_add_entry(gain_schedule_table_t *table,
                        double sched_val,
                        const pid_gain_set_t *gains,
                        const char *label) {
    if (!table || !gains || table->num_entries >= GS_MAX_BREAKPOINTS) {
        return false;
    }
    if (gains->Kp < 0.0 || gains->Ki < 0.0 || gains->Kd < 0.0) {
        return false;
    }

    uint32_t i = table->num_entries;
    while (i > 0 && table->entries[i-1].scheduling_value > sched_val) {
        table->entries[i] = table->entries[i-1];
        i--;
    }
    if (i > 0 && table->entries[i-1].scheduling_value == sched_val) {
        table->entries[i-1].gains = *gains;
        if (label) {
            strncpy(table->entries[i-1].label, label, 31);
            table->entries[i-1].label[31] = '\0';
        }
        return true;
    }

    table->entries[i].scheduling_value = sched_val;
    table->entries[i].gains = *gains;
    table->entries[i].region = REGION_NOMINAL;
    table->entries[i].bandwidth = 0.0;
    table->entries[i].gain_margin = 0.0;
    table->entries[i].phase_margin = 0.0;
    table->entries[i].sensitivity_peak = 0.0;
    table->entries[i].validated = false;
    if (label) {
        strncpy(table->entries[i].label, label, 31);
        table->entries[i].label[31] = '\0';
    } else {
        table->entries[i].label[0] = '\0';
    }

    table->num_entries++;
    if (sched_val < table->sched_min) table->sched_min = sched_val;
    if (sched_val > table->sched_max) table->sched_max = sched_val;
    return true;
}

void gs_table_sort_entries(gain_schedule_table_t *table) {
    if (!table || table->num_entries < 2) return;
    for (uint32_t i = 1; i < table->num_entries; i++) {
        schedule_entry_t key = table->entries[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && table->entries[j].scheduling_value >
               key.scheduling_value) {
            table->entries[j+1] = table->entries[j];
            j--;
        }
        table->entries[j+1] = key;
    }
    if (table->num_entries > 0) {
        table->sched_min = table->entries[0].scheduling_value;
        table->sched_max = table->entries[table->num_entries-1].scheduling_value;
    }
}

bool gs_table_find_bracket(const gain_schedule_table_t *table,
                           double sched_val,
                           uint32_t *idx_low,
                           uint32_t *idx_high) {
    if (!table || !idx_low || !idx_high || table->num_entries < 2) {
        return false;
    }

    if (sched_val < table->entries[0].scheduling_value ||
        sched_val > table->entries[table->num_entries-1].scheduling_value) {
        return false;
    }

    uint32_t lo = 0, hi = table->num_entries - 1;
    while (hi - lo > 1) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (table->entries[mid].scheduling_value <= sched_val) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    *idx_low = lo;
    *idx_high = hi;
    return true;
}

bool gs_table_validate(const gain_schedule_table_t *table,
                       char *errmsg, size_t errmsg_size) {
    if (!table) {
        if (errmsg && errmsg_size > 0) {
            snprintf(errmsg, errmsg_size, "Null table pointer");
        }
        return false;
    }
    if (table->num_entries < 2) {
        if (errmsg && errmsg_size > 0) {
            snprintf(errmsg, errmsg_size,
                     "Table has %u entries, need at least 2",
                     table->num_entries);
        }
        return false;
    }
    for (uint32_t i = 1; i < table->num_entries; i++) {
        if (table->entries[i].scheduling_value <=
            table->entries[i-1].scheduling_value) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Non-monotonic at index %u: %g then %g",
                         i, table->entries[i-1].scheduling_value,
                         table->entries[i].scheduling_value);
            }
            return false;
        }
    }
    for (uint32_t i = 0; i < table->num_entries; i++) {
        const pid_gain_set_t *g = &table->entries[i].gains;
        if (g->Kp <= 0.0) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Non-positive Kp=%g at entry %u",
                         g->Kp, i);
            }
            return false;
        }
        if (g->Ki < 0.0 || g->Kd < 0.0) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Negative Ki or Kd at entry %u", i);
            }
            return false;
        }
    }
    return true;
}

bool gs_table_remove_entry(gain_schedule_table_t *table, uint32_t index) {
    if (!table || index >= table->num_entries) return false;
    for (uint32_t i = index; i < table->num_entries - 1; i++) {
        table->entries[i] = table->entries[i+1];
    }
    table->num_entries--;
    if (table->num_entries == 0) {
        table->sched_min = +1e300;
        table->sched_max = -1e300;
    }
    return true;
}

void gs_table_clear(gain_schedule_table_t *table) {
    if (!table) return;
    memset(table->entries, 0, sizeof(table->entries));
    table->num_entries = 0;
    table->sched_min = +1e300;
    table->sched_max = -1e300;
}

uint32_t gs_table_count(const gain_schedule_table_t *table) {
    if (!table) return 0;
    return table->num_entries;
}

/**
 * gs_table_clone - Deep copy a schedule table.
 * Complexity: O(n)
 */
bool gs_table_clone(const gain_schedule_table_t *src, gain_schedule_table_t *dst) {
    if (!src || !dst) return false;
    memcpy(dst, src, sizeof(gain_schedule_table_t));
    return true;
}

/**
 * gs_table_get_entry - Get read-only pointer to a specific entry.
 * Complexity: O(1)
 */
const schedule_entry_t *gs_table_get_entry(const gain_schedule_table_t *table,
                                             uint32_t index) {
    if (!table || index >= table->num_entries) return NULL;
    return &table->entries[index];
}

/**
 * gs_table_find_nearest - Find the entry closest to a scheduling value.
 * Uses binary search then compares two candidates. O(log n).
 */
int32_t gs_table_find_nearest(const gain_schedule_table_t *table, double sched_val) {
    if (!table || table->num_entries == 0) return -1;
    if (table->num_entries == 1) return 0;
    if (sched_val <= table->entries[0].scheduling_value) return 0;
    if (sched_val >= table->entries[table->num_entries-1].scheduling_value)
        return (int32_t)(table->num_entries - 1);

    uint32_t lo = 0, hi = table->num_entries - 1;
    while (hi - lo > 1) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (table->entries[mid].scheduling_value <= sched_val) lo = mid;
        else hi = mid;
    }
    double d_lo = fabs(sched_val - table->entries[lo].scheduling_value);
    double d_hi = fabs(sched_val - table->entries[hi].scheduling_value);
    return (d_lo <= d_hi) ? (int32_t)lo : (int32_t)hi;
}

/**
 * gs_table_set_defaults - Configure default (fallback) gain values.
 * Used when scheduling variable is outside the table range.
 * Complexity: O(1)
 */
void gs_table_set_defaults(gain_schedule_table_t *table,
                           double Kp, double Ki, double Kd) {
    if (!table) return;
    if (Kp > 0.0) table->default_Kp = Kp;
    if (Ki >= 0.0) table->default_Ki = Ki;
    if (Kd >= 0.0) table->default_Kd = Kd;
}

/**
 * gs_table_set_extrapolation - Configure extrapolation behavior.
 * Complexity: O(1)
 */
void gs_table_set_extrapolation(gain_schedule_table_t *table,
                                 bool extrap_low, bool extrap_high) {
    if (!table) return;
    table->extrapolate_low = extrap_low;
    table->extrapolate_high = extrap_high;
}

/**
 * gs_table_set_interp_method - Set the interpolation method.
 * Complexity: O(1)
 */
void gs_table_set_interp_method(gain_schedule_table_t *table,
                                 interp_method_t method) {
    if (!table) return;
    table->interp_method = method;
}
