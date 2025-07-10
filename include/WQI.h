#ifndef WQI_H
#define WQI_H

#include <Arduino.h>

inline float calculateWQI(float turbidity, float DO, float EC, float pH, float temp) {
    const char* parameters[] = {"turbidity", "DO", "EC", "pH"};
    float Si[4];
    float Expect[4] = {0, 4, 0, 7.00001};
    float dif_Si[4];
    float dif_Real[4];
    float Qi[4];
    float Wi[4];

    // Set Si
    Si[0] = 5;                        // turbidity
    Si[1] = -0.28f * temp + 14.6f;    // DO
    Si[2] = 2000;                     // EC
    Si[3] = (pH > 8.5f) ? 8.5f : ((pH < 6.5f) ? 6.5f : pH);  // pH

    // dif_Si and dif_Real
    dif_Si[0] = abs(Si[0] - Expect[0]);
    dif_Si[1] = abs(Si[1] - Expect[1]);
    dif_Si[2] = abs(Si[2] - Expect[2]);
    dif_Si[3] = abs(Si[3] - Expect[3]);

    dif_Real[0] = turbidity - Expect[0];
    dif_Real[1] = DO - Expect[1];
    dif_Real[2] = EC - Expect[2];
    dif_Real[3] = abs(pH - Expect[3]);

    // K calculation
    float K = 0;
    for (int i = 0; i < 4; ++i)
        K += 1.0f / Si[i];
    K = 1.0f / K;

    // Wi
    for (int i = 0; i < 4; ++i)
        Wi[i] = K / Si[i];
    Wi[2] = 0.1f; // EC special case

    // Qi
    for (int i = 0; i < 4; ++i) {
        Qi[i] = dif_Real[i] / dif_Si[i];
        if (Qi[i] < 0 && i == 1) { // Only fix DO
            Qi[i] = (Si[i] - DO) / (Si[i] - Expect[i]);
        }
        Qi[i] *= 100;
    }

    // WQI calculation
    float S_Wi = 0;
    float S_WQi = 0;
    for (int i = 0; i < 4; ++i) {
        S_Wi += Wi[i];
        S_WQi += Wi[i] * Qi[i];
    }

    float WQI = S_WQi / S_Wi;

    return WQI;
}

String evaluateWQI(float wqiValue) {
    String WQI_index = "";

    if (wqiValue < 25) {
        WQI_index = "Excellent water quality";
    } else if (wqiValue < 50) {
        WQI_index = "Good water quality";
    } else if (wqiValue < 75) {
        WQI_index = "Poor water quality";
    } else if (wqiValue < 100) {
        WQI_index = "Very poor water quality";
    } else {
        WQI_index = "Unsuitable for drink";
    }

    return WQI_index;
}



#endif
