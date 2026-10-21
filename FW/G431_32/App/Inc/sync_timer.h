#ifndef SYNC_TIMER_H
#define SYNC_TIMER_H

__attribute__((section(".ccm_code")))
void TIM2_irq_handler();
void sync_timer_init(uint8_t _irq_en);
void sync_timer_start(uint8_t _calibration);
void sync_timer_stop();

#endif //SYNC_TIMER_H