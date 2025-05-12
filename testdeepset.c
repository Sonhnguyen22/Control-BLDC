#define LUT_SIZE 1024
uint16_t sin_LUT[LUT_SIZE];

void Generate_SVM_LUT(void) {
    for (int i = 0; i < LUT_SIZE; i++) {
        float angle = (i * M_PI) / (3 * LUT_SIZE); // Góc từ 0 đến π/3
        sin_LUT[i] = (uint16_t)(57600 * sin(angle)); // Upscale 57600 lần (theo tài liệu)
    }
}

void Compute_SVM_DutyCycles(float V_ref_amplitude, float angle) {
    // Xác định sector (0-5)
    uint8_t sector = (uint8_t)(angle / (M_PI / 3)) % 6;
    
    // Tính góc trong sector hiện tại (0 đến π/3)
    float sector_angle = fmod(angle, M_PI / 3);
    
    // Lấy index trong LUT (0-1023)
    uint16_t idx = (uint16_t)(sector_angle * (3 * LUT_SIZE / M_PI));
    
    // Tính T_cw và T_ccw từ LUT (công thức (17) trong tài liệu)
    uint32_t T_cw = (V_ref_amplitude * sin_LUT[idx]) >> 19;      // = (m' * LUT[i]) / 524288
    uint32_t T_ccw = (V_ref_amplitude * sin_LUT[LUT_SIZE-1-idx]) >> 19;
    
    // Tính T_z (thời gian zero vector)
    uint32_t T_z = (7200 - T_cw - T_ccw) / 2;
    
    // Gán duty cycle cho từng kênh theo sector (Bảng 2 trong tài liệu)
    switch (sector) {
        case 0:
            htim1.Instance->CCR1 = T_z;
            htim1.Instance->CCR2 = T_z + T_cw;
            htim1.Instance->CCR3 = 7200 - T_z;
            break;
        case 1:
            htim1.Instance->CCR1 = T_z + T_ccw;
            htim1.Instance->CCR2 = T_z;
            htim1.Instance->CCR3 = 7200 - T_z;
            break;
        // Thêm các case cho sector 2-5...
    }
}


uint32_t gear_counter = 0; // Biến bánh răng lớn (30-bit)
#define GEAR_RATIO (1 << 30) // 2^30

void Update_SVM_Vector(float frequency, float V_amplitude) {
    // Cập nhật góc quay dựa trên coaxial gears (công thức (18))
    uint32_t step = (uint32_t)(frequency * 6144 / 5000 * GEAR_RATIO); // f = 5kHz
    gear_counter += step;
    uint16_t angle_step = gear_counter >> 30; // Lấy 14 bit cao
    gear_counter &= 0x3FFFFFFF; // Giữ lại 30 bit thấp
    
    // Tính góc tổng (0 đến 2π)
    float angle = (angle_step % 6144) * (2 * M_PI / 6144);
    
    // Gọi hàm tính duty cycle
    Compute_SVM_DutyCycles(V_amplitude, angle);
}









