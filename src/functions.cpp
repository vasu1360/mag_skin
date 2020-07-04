// void toggle_CS_pins(bool LOW_HIGH) {
//   digitalWriteFast(cs_pins[0], LOW_HIGH);
//   digitalWriteFast(cs_pins[1], LOW_HIGH);
//   digitalWriteFast(cs_pins[2], LOW_HIGH);
//   digitalWriteFast(cs_pins[3], LOW_HIGH);
//   digitalWriteFast(cs_pins[4], LOW_HIGH);
// }

// // void calibration(int chip_iterator){
// //  int cal_count = 1000;
// //  float avg_cal[3] = {};
// //  for (int ii = 0; ii <= cal_count; ii++) {
// //    if (digitalRead(RDY_pins[0]) == HIGH) { //&& digitalRead(RDY_pins[1]) == HIGH && digitalRead(RDY_pins[2]) == HIGH) {
// //      digitalWriteFast(cs_pins[chip_iterator], LOW);
// //      SPI.transfer(0xC0 | 0x28); //10101000: 0x80 represents read mode, 0x28 represents OUT_X_L register
// //      data_reg[0] = SPI.transfer(0); //OUT_X_L register
// //      data_reg[1] = SPI.transfer(0); //OUT_X_H register
// //      data_reg[2] = SPI.transfer(0); //OUT_Y_L register
// //      data_reg[3] = SPI.transfer(0); //OUT_Y_H register
// //      data_reg[4] = SPI.transfer(0); //OUT_Z_L register
// //      data_reg[5] = SPI.transfer(0); //OUT_Z_H register
// //      digitalWrite(cs_pins[chip_iterator], HIGH);

// //      /* Get 18bits data, raw data unit is "count or LSB" */
// //      data16bit[0] = (int)(data_reg[1] << 8 | data_reg[0]);
// //      data16bit[1] = (int)(data_reg[3] << 8 | data_reg[2]);
// //      data16bit[2] = (int)(data_reg[5] << 8 | data_reg[4]);
// // //      for (int ii = 0; ii <= cal_count; ii++){
// //        avg_cal[0] = avg_cal[0] + (data16bit[0]);
// //        avg_cal[1] = avg_cal[1] + (data16bit[1]);
// //        avg_cal[2] = avg_cal[2] + (data16bit[2]);
// // //      }
// //     /* Find the baseline average for the sensor */
// //   }
// //  }
// //     avg_chip_mG[0][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G)*(avg_cal[0]/cal_count);
// //     avg_chip_mG[1][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G)*(avg_cal[1]/cal_count);
// //     avg_chip_mG[2][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G)*(avg_cal[2]/cal_count);
// // }