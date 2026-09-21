#ifndef INPUT_CALIBRATION_H
#define INPUT_CALIBRATION_H

struct input_calibration_private_vars_init_s
{
    //parameters
    uint16_t n_pts_per_turn;
    uint16_t n_turns;
};

void input_calibration_init();
void read_adcs_calibration();

#endif //INPUT_CALIBRATION_H