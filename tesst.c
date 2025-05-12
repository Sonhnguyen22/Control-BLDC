#define PWM_PERIOD 7200
#define MAX_MOD_INDEX 65535

extern uint16_t s_lookup[1024]; // bảng sin đã được scale sẵn
extern uint16_t modulation_index; // giá trị từ 0-65535
extern uint16_t angle_index;      // index từ 0–1023
extern uint8_t current_sector;    // giá trị từ 0–5

void SVM_Calc(float U_alpha, float U_beta, float Theta,
              uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3) {

    uint32_t mi = modulation_index;
    uint32_t sin_phi = s_lookup[angle_index];
    uint32_t sin_60_minus_phi = s_lookup[1023 - angle_index];

    uint32_t Tcw = (mi * sin_60_minus_phi) >> 9; // chia 512
    uint32_t Tccw = (mi * sin_phi) >> 9;
    uint32_t Tz = (PWM_PERIOD - Tcw - Tccw) >> 1;

    switch (current_sector) {
        case 0:
            *ccr1 = Tz;
            *ccr2 = Tz + Tcw;
            *ccr3 = PWM_PERIOD - Tz;
            break;
        case 1:
            *ccr1 = Tz + Tccw;
            *ccr2 = Tz;
            *ccr3 = PWM_PERIOD - Tz;
            break;
        case 2:
            *ccr1 = PWM_PERIOD - Tz;
            *ccr2 = Tz;
            *ccr3 = Tz + Tcw;
            break;
        case 3:
            *ccr1 = PWM_PERIOD - Tz;
            *ccr2 = Tz + Tccw;
            *ccr3 = Tz;
            break;
        case 4:
            *ccr1 = Tz + Tcw;
            *ccr2 = PWM_PERIOD - Tz;
            *ccr3 = Tz;
            break;
        case 5:
            *ccr1 = Tz;
            *ccr2 = PWM_PERIOD - Tz;
            *ccr3 = Tz + Tccw;
            break;
    }
}


uint16_t s_lookup[1024];
void init_sin_table(void) {
    for (int i = 0; i < 1024; i++) {
        s_lookup[i] = (uint16_t)(57600 * sinf((float)i * M_PI / 3069.0f)); // M_PI / (6 × 1024)
    }
}

void update_sector_and_index(float Theta) {
    if (Theta >= 2.0f * M_PI) Theta -= 2.0f * M_PI;
    current_sector = (uint8_t)(Theta / (M_PI / 3.0f));  // mỗi sector chiếm 60 độ
    float phi = Theta - current_sector * (M_PI / 3.0f); // góc nội bộ trong sector
    angle_index = (uint16_t)(phi * 3069.0f / M_PI);     // scale theo công thức bảng
}

uint16_t ccr1, ccr2, ccr3;

update_sector_and_index(Theta);
SVM_Calc(U_alpha, U_beta, Theta, &ccr1, &ccr2, &ccr3);

TIM1->CCR1 = ccr1;
TIM1->CCR2 = ccr2;
TIM1->CCR3 = ccr3;
