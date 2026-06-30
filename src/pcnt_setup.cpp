#include "pcnt_setup.h"
#include "core_logic.h"
// Define pins for the 4 motors (Phase A and Phase B)
const int PCNT_PINS[4][2] = {
    {34, 35}, // Motor 1
    {36, 39}, // Motor 2
    {4,  18}, // Motor 3
    {19, 23}  // Motor 4
};

static int16_t last_pcnt_val[4] = {0, 0, 0, 0};

void setup_pcnt() {
    for (int i = 0; i < 4; i++) {
        pcnt_config_t pcnt_config = {};
        // Setup channel 0
        pcnt_config.pulse_gpio_num = PCNT_PINS[i][0];
        pcnt_config.ctrl_gpio_num = PCNT_PINS[i][1];
        pcnt_config.channel = PCNT_CHANNEL_0;
        pcnt_config.unit = (pcnt_unit_t)i;
        pcnt_config.pos_mode = PCNT_COUNT_INC;
        pcnt_config.neg_mode = PCNT_COUNT_DEC;
        pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config.counter_h_lim = 32767;
        pcnt_config.counter_l_lim = -32768;

        pcnt_unit_config(&pcnt_config);
        
        // Setup channel 1 for full quadrature (4x resolution)
        pcnt_config.pulse_gpio_num = PCNT_PINS[i][1];
        pcnt_config.ctrl_gpio_num = PCNT_PINS[i][0];
        pcnt_config.channel = PCNT_CHANNEL_1;
        pcnt_config.pos_mode = PCNT_COUNT_INC;
        pcnt_config.neg_mode = PCNT_COUNT_DEC;
        pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
        
        pcnt_unit_config(&pcnt_config);

        // Filter glitchy pulses (100 is a safe glitch filter value for slow actuators)
        pcnt_set_filter_value((pcnt_unit_t)i, 100);
        pcnt_filter_enable((pcnt_unit_t)i);

        pcnt_counter_pause((pcnt_unit_t)i);
        pcnt_counter_clear((pcnt_unit_t)i);
        pcnt_counter_resume((pcnt_unit_t)i);
    }
}

void update_pcnt_counts(int32_t currentPositions[4]) {
    for (int i = 0; i < 4; i++) {
        int16_t current_val;
        pcnt_get_counter_value((pcnt_unit_t)i, &current_val);
        
        CoreLogic::updateAccumulatedPosition(current_val, last_pcnt_val[i], currentPositions[i]);
    }
}
