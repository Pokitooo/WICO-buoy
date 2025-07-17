#ifndef DF_SENSOR_H
#define DF_SENSOR_H

#include <Arduino.h>

// Calibration settings
#define TWO_POINT_CALIBRATION 0

#define CAL1_V (1600)
#define CAL1_T (25)
#define CAL2_V (1300)
#define CAL2_T (15)

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

// Safe inline function definition
inline int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

struct CalibrationPoint
{
  float voltageRatio;
  float ntu;
};

CalibrationPoint calibrationPoints[] = {
    {3200, 0.5},
    {1900.0, 50.0},
    {116, 300.0}};

const int numPoints = sizeof(calibrationPoints) / sizeof(calibrationPoints[0]);

float ntu;

// inline float read_turbidity(float voltageRatio)
// {
//   // voltageRatio = (float)rawValue / ADC_MAX;

//   for (int i = 0; i < numPoints - 1; i++)
//   {
//     CalibrationPoint p1 = calibrationPoints[i];
//     CalibrationPoint p2 = calibrationPoints[i + 1];

//     if (voltageRatio >= p2.voltageRatio && voltageRatio <= p1.voltageRatio)
//     {
//       ntu = p1.ntu + (voltageRatio - p1.voltageRatio) * (p2.ntu - p1.ntu) / (p2.voltageRatio - p1.voltageRatio);
//       return ntu;
//     }
//   }

//   if (voltageRatio > calibrationPoints[0].voltageRatio)
//     return calibrationPoints[0].ntu;
//   if (voltageRatio < calibrationPoints[numPoints - 1].voltageRatio)
//     return calibrationPoints[numPoints - 1].ntu;

//   return 0;
// }

#endif // DF_SENSOR_H
