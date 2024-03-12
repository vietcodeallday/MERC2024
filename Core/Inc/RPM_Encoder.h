
#ifndef RPM_ENCODER_H_
#define RPM_ENCODER_H_

void Encoder_init(uint16_t max_count);
extern float get_rpm();

extern volatile uint64_t first, second, dir;
extern float frequency;
extern float time;
extern float rpm_out;

#endif /* RPM_ENCODER_H_ */
