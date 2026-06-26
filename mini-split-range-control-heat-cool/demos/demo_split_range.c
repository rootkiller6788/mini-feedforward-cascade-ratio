/**
 * @file demo_split_range.c
 * @brief Interactive demo of split-range valve sequencing
 *
 * Visualizes how controller output maps to valve positions for
 * different split-range schemes (heat/cool, pH, three-way).
 *
 * Compile: gcc -I../include -o demo_split_range demo_split_range.c ../src/*.c -lm
 */

#include <stdio.h>
#include <string.h>
#include "split_range_control.h"

static void print_bar(const char *label, double val, int width) {
    printf("%-12s [", label);
    int filled = (int)(val / 100.0 * width);
    for (int i = 0; i < width; i++) {
        printf("%c", (i < filled) ? '#' : '.');
    }
    printf("] %5.1f%%\n", val);
}

static void demo_scheme(const char *name, split_range_scheme_t *scheme) {
    printf("\n=== %s ===\n", name);
    printf("Split point: %.1f%%, Deadband: %.1f%%, Channels: %u\n",
           scheme->split_point, scheme->deadband_width, scheme->num_channels);

    printf("\nCO%%  ");
    for (uint32_t c = 0; c < scheme->num_channels; c++) {
        printf("Ch%u(%%open)  ", c);
    }
    printf("  Visualization\n");
    printf("----  ");
    for (uint32_t c = 0; c < scheme->num_channels; c++) {
        printf("-----------  ");
    }
    printf("  -------------\n");

    double positions[6];
    for (int co = 0; co <= 100; co += 10) {
        split_distribute_output(scheme, (double)co, positions);
        printf("%3d   ", co);
        for (uint32_t c = 0; c < scheme->num_channels; c++) {
            printf("%8.1f     ", positions[c]);
        }
        /* Visual bar */
        printf(" ");
        print_bar("CO", (double)co, 20);
    }
}

int main(void) {
    printf("============================================\n");
    printf("  Split-Range Valve Sequencing Demo\n");
    printf("============================================\n");

    /* Demo 1: Heat/Cool */
    split_range_scheme_t hc;
    split_init_heat_cool_scheme(&hc);
    demo_scheme("Heat/Cool Split-Range (Reactor Temperature)", &hc);

    /* Demo 2: pH Neutralization */
    split_range_scheme_t ph;
    split_init_ph_scheme(&ph);
    demo_scheme("Acid/Base Split-Range (pH Neutralization)", &ph);

    /* Demo 3: Three-Way */
    split_range_scheme_t tw;
    split_init_three_way_scheme(&tw);
    demo_scheme("Three-Way Split-Range (Heat/Bypass/Cool)", &tw);

    /* Demo 4: Custom configuration */
    printf("\n=== Custom: Asymmetric Heat/Cool with Different Valve Ranges ===\n");
    split_range_scheme_t custom;
    memset(&custom, 0, sizeof(custom));
    custom.mode = SPLIT_MODE_COMPLEMENTARY;
    custom.split_point = 60.0;  /* shift split point to favor cooling */
    custom.deadband_width = 3.0;
    custom.transition_type = SPLIT_TRANSITION_CUBIC_SPLINE;

    split_add_channel(&custom, 0.0, 57.0, 0.0, 100.0, SPLIT_ACTION_INCREASING, SPLIT_VALVE_EQUAL_PCT);
    split_add_channel(&custom, 63.0, 100.0, 0.0, 100.0, SPLIT_ACTION_INCREASING, SPLIT_VALVE_LINEAR);

    demo_scheme("Custom Asymmetric (60%% split, cubic spline)", &custom);

    /* Valve characteristic comparison */
    printf("\n=== Valve Characteristic Comparison ===\n");
    printf("Stem%%  Linear    Equal%%   Quick     ModPara\n");
    printf("-----  ------    ------    -----     -------\n");
    for (int x = 0; x <= 100; x += 10) {
        double s = x / 100.0;
        printf("%4d   %6.3f   %6.3f   %6.3f   %6.3f\n", x,
               split_valve_characteristic_forward(s, SPLIT_VALVE_LINEAR, 50.0),
               split_valve_characteristic_forward(s, SPLIT_VALVE_EQUAL_PCT, 50.0),
               split_valve_characteristic_forward(s, SPLIT_VALVE_QUICK_OPENING, 50.0),
               split_valve_characteristic_forward(s, SPLIT_VALVE_MODIFIED_PARABOLIC, 50.0));
    }

    printf("\nDemo complete.\n");
    return 0;
}
