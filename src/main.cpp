#include <Arduino.h>
#include <SPI.h>

//SPI1 pins for Teensy 3.6
// #define SPI1_MISO 1
// #define SPI1_MOSI 0
// #define SPI1_SCK 32

#define MAG_SENSOR_NUM 1  // quantity of sensors
#define PROBEPIN 23      // For timing or debugging only
#define SPISPEED 10000000 // Arduino Mega
#define LOOP_FREQUENCY 1000
#define LIS3MDL_FROM_FS_4G_TO_G (float)(6842.0)
#define LIS3MDL_FROM_FS_8G_TO_G (float)(3421.0)
#define LIS3MDL_FROM_FS_12G_TO_G (float)(2281.0)
#define LIS3MDL_FROM_FS_16G_TO_G (float)(1711.0)

static const uint8_t cs_pins[MAG_SENSOR_NUM] = {28};  //, 27}; // chip select
static const uint8_t RDY_pins[MAG_SENSOR_NUM] = {33}; //, 34};
static const uint8_t disp_pin = 39;                   // reading laser disp sensor A20

//// This is for Teensy 4.0
//static const uint8_t cs_pins[MAG_SENSOR_NUM] = {8}; // chip select
//static const uint8_t RDY_pins[MAG_SENSOR_NUM] = {14}; // Data ready

byte Chip_ID;

uint8_t data_reg[6] = {0};
int data16bit[3] = {0};
float magnetic_mG[3][MAG_SENSOR_NUM] = {};
unsigned long loop_timing = 0;
unsigned long loop_control_period;
//float displacement_laser = 0;

int upper_range = 100;
int lower_range = 0;

int mytime = 0;

float avg_chip_mG[3][MAG_SENSOR_NUM] = {};

void toggle_CS_pins(bool LOW_HIGH); //declare the function before it's used

void setup()
{
    Serial.begin(115200);
    delay(3000);
    loop_control_period = round(1000000 * (1.0 / LOOP_FREQUENCY)); //This is for loop control If you use micros() for timing, change 1000 to 1000000
    pinMode(disp_pin, INPUT);
    for (int i = 0; i < MAG_SENSOR_NUM; i++)
    {
        pinMode(cs_pins[i], OUTPUT);
        digitalWriteFast(cs_pins[i], HIGH);
        pinMode(RDY_pins[i], INPUT);
    }

    pinMode(PROBEPIN, OUTPUT);
    digitalWriteFast(PROBEPIN, LOW);
    // pinMode(SPI1_SCK, OUTPUT);
    // pinMode(SPI1_MOSI, OUTPUT);
    // pinMode(SPI1_MISO, INPUT);

    // SPI1.setSCK(SPI1_SCK);
    // SPI1.setMOSI(SPI1_MOSI);
    // SPI1.setMISO(SPI1_MISO);
    SPI1.begin(); //start the spi-bus
    delay(100);

    SPI1.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0)); // start SPI
    delay(1000);

    for (int i = 0; i < MAG_SENSOR_NUM; i++)
    {
        digitalWriteFast(cs_pins[i], LOW);
        SPI1.transfer(0x0F | 0xC0);    // 0b10001111: 0x80 represents write mode, 0x21 represents Register: Control 2
        Chip_ID = SPI1.transfer(0x00); // b00000000
        digitalWriteFast(cs_pins[i], HIGH);
        delayMicroseconds(100);
        if (Chip_ID == 61)
        {
            Serial.print("Identity of LIS3MDL# ");
            Serial.print(i + 1);
            Serial.println(" checked!");
        }
        Chip_ID = 0;
    }
    /******************************************************************************************************************
    Register: Control 2
    Address: 0x21
    BIT 7  |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |  BIT 1 |   BIT 0
      0    |  FS1    |   FS0   |     0    |  REBOOT  |  SOFT_RST |   0    |     0
    + FS0 & FS0: (Control the full scale)
        FS1   FS0     Full- Scale [G]    LSB/G
         0     0        +-4              6842
         0     1        +-8              3421
         1     0        +-12             2281
         1     1        +-16             1711
  **********************************************************************************************************************/
    toggle_CS_pins(LOW);       // RESET
    SPI1.transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    SPI1.transfer(0x08);        // b00001000
    toggle_CS_pins(HIGH);
    delay(10);

    toggle_CS_pins(LOW);       // Set FS = 16
    SPI1.transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    SPI1.transfer(0x60);        //
    toggle_CS_pins(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 1
    Address: 0x20
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    TEMP_EN   |  OM1    |   OM0   |    DO2   |   DO1    |    DO0    |   FAST_ODR   |    ST
  **********************************************************************************************************************/
    toggle_CS_pins(LOW);
    SPI1.transfer(0x20 | 0x40); // 0b00100000: 0x00 represents write mode, 0x20 represents Register: Control 1
    SPI1.transfer(0x02);        // b00000010
    toggle_CS_pins(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 4
    Address: 0x23
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    0    |     0    |   OMZ1   |    OMZ0   |      BLE     |    0
  **********************************************************************************************************************/
    toggle_CS_pins(LOW);
    SPI1.transfer(0x23 | 0x40); // 0b00100011 : 0x00 represents write mode, 0x23 represents Register: Control 4
    SPI1.transfer(0x00);        // b00000000 - low power, 5.3 mgauss RMS noise, start up time 1.2ms, TON duration 0.9ms
    toggle_CS_pins(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 3
    Address: 0x22
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    LP   |     0    |    0     |    SIM    |      MD1     |    MD0
  **********************************************************************************************************************/
    toggle_CS_pins(LOW);
    SPI1.transfer(0x22 | 0x40); // 0b00100010: 0x00 represents write mode, 0x22 represents Register: Control 3
    SPI1.transfer(0x00);        // b00000000 - continuous measurement mode
    toggle_CS_pins(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 5
    Address: 0x24
      BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    FAST_READ   |   BDU   |    0    |     0    |   0      |     0     |      0       |    0
  **********************************************************************************************************************/
    toggle_CS_pins(LOW);
    SPI1.transfer(0x24 | 0x40); // 0b00100100: 0x00 represents write mode, 0x24 represents Register: Control 5
    SPI1.transfer(0x00);        // b00000000
    toggle_CS_pins(HIGH);
    delay(100);

    Serial.println("Initialization complete");
    delay(100);

    //    Ending the SPI transaction
    SPI1.endTransaction();

    while (!Serial && (millis() <= 5000))
        ; // WAIT UP TO 5000 MILLISECONDS FOR SERIAL OUTPUT CONSOLE
    Serial.println("configured, starting");
    Serial.println("");
    delay(100);
    // Serial.println("Time(millis) Disp(mm) X-axis     Y-axis      Z-axis");
    Serial.println("Time(millis)     X-axis     Y-axis      Z-axis");

    delay(2000);
    SPI1.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0)); // start SPI

    //  //find the baseline readings before starting the code
    //  for (int chip_iterator = 0; chip_iterator < MAG_SENSOR_NUM; chip_iterator++) {
    //    calibration(chip_iterator);
    //    for (int j = 0; j <= 2; j++) {
    //      Serial.print(avg_chip_mG[j][chip_iterator]);
    //      Serial.print(" ");
    //    }
    //  }

    // Serial.println("Calibration Over!");
}

void loop()
{
    // float displacement_laser = analogRead(disp_pin);
    // displacement_laser = map(displacement_laser,10,1023,0,3.3); //in mm

    if (digitalRead(RDY_pins[0]) == HIGH)
    { //&& digitalRead(RDY_pins[1]) == HIGH && digitalRead(RDY_pins[2]) == HIGH) {
        for (int chip_iterator = 0; chip_iterator < MAG_SENSOR_NUM; chip_iterator++)
        {
            digitalWriteFast(cs_pins[chip_iterator], LOW);
            SPI1.transfer(0xC0 | 0x28);     //10101000: 0x80 represents read mode, 0x28 represents OUT_X_L register
            data_reg[0] = SPI1.transfer(0); //OUT_X_L register
            data_reg[1] = SPI1.transfer(0); //OUT_X_H register
            data_reg[2] = SPI1.transfer(0); //OUT_Y_L register
            data_reg[3] = SPI1.transfer(0); //OUT_Y_H register
            data_reg[4] = SPI1.transfer(0); //OUT_Z_L register
            data_reg[5] = SPI1.transfer(0); //OUT_Z_H register
            digitalWrite(cs_pins[chip_iterator], HIGH);

            /* Get 18bits data, raw data unit is "count or LSB" */
            data16bit[0] = (int)(data_reg[1] << 8 | data_reg[0]);
            data16bit[1] = (int)(data_reg[3] << 8 | data_reg[2]);
            data16bit[2] = (int)(data_reg[5] << 8 | data_reg[4]);

            magnetic_mG[0][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[0]); //- avg_chip_mG[0][chip_iterator];
            magnetic_mG[1][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[1]); //- avg_chip_mG[1][chip_iterator];
            magnetic_mG[2][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[2]); //- avg_chip_mG[2][chip_iterator];

            for (int j = 0; j <= 2; j++)
            { //unwrapping of mag data >> check if the baseline changes when this is commented out
                if (magnetic_mG[j][chip_iterator] > 19150)
                {
                    magnetic_mG[j][chip_iterator] -= 38300;
                }
            }

            Serial.print(millis());
            Serial.print(" ");
            // Serial.print(displacement_laser);
            // Serial.print(" ");
            for (int j = 0; j <= 2; j++)
            {
                Serial.print(magnetic_mG[j][chip_iterator]);
                Serial.print(" ");
            }
        }

        Serial.println(" ");
        digitalWriteFast(PROBEPIN, !digitalRead(PROBEPIN));
        while (micros() < (loop_timing + loop_control_period))
        {
            //wait...
        }
        loop_timing = micros();
    }
}

void toggle_CS_pins(bool LOW_HIGH)
{
    // 10 sensors max per sensor MCU
    digitalWriteFast(cs_pins[0], LOW_HIGH);
    digitalWriteFast(cs_pins[1], LOW_HIGH);
    digitalWriteFast(cs_pins[2], LOW_HIGH);
    digitalWriteFast(cs_pins[3], LOW_HIGH);
    digitalWriteFast(cs_pins[4], LOW_HIGH);
    digitalWriteFast(cs_pins[5], LOW_HIGH);
    digitalWriteFast(cs_pins[6], LOW_HIGH);
    digitalWriteFast(cs_pins[7], LOW_HIGH);
    digitalWriteFast(cs_pins[8], LOW_HIGH);
    digitalWriteFast(cs_pins[9], LOW_HIGH);
}
