#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MS5837.h>
#include <ModbusSlave.h>

// gpio
#define RE_PIN 2
#define DE_PIN 3

// default values
#define DEFAULT_SLAVE_ID 0
#define DEFAULT_BAUD_RATE 9600
#define FRESHWATER_DENSITY 977.0

// SI unit values
#define PASCAL_UNIT 0x5061
#define CELSIUS_UNIT 0x43
#define METER_UNIT 0x6D
#define KILOGRAM_UNIT 0x6B67

// Imperial unit values
#define PSI_UNIT_WORD_1 0x7073
#define PSI_UNIT_WORD_2 0x69
#define FAHRENHEIT_UNIT 0x46
#define FEET_UNIT 0x6674
#define POUND_UNIT 0x6C62

// Shared characters among units of measurement
#define CUBIC_UNIT 0x5E33
#define PER_SYMBOL 0x2F

// Conversion values
#define MILLIBAR_TO_PASCAL 100
#define MILLIBAR_TO_PSI 0.0145037738
#define METER_TO_FEET 3.280839895
#define KGPM3_TO_LBPFT3 0.062427960576145
#define LBPFT3_TO_KGPM3 16.01846337396

// baud rates
enum {
  BAUD_RATE_1 = 2400,
  BAUD_RATE_2 = 4800,
  BAUD_RATE_3 = 9600,
  BAUD_RATE_4 = 14400,
  BAUD_RATE_5 = 19200,
  BAUD_RATE_6 = 28800,
  BAUD_RATE_7 = 38400,
  BAUD_RATE_8 = 57600,
  BAUD_RATE_9 = 76800,
  BAUD_RATE_10 = 115200,
  BAUD_RATE_11 = 230400,
  BAUD_RATE_12 = 250000,
  BAUD_RATE_13 = 500000,
  BAUD_RATE_14 = 1000000,
  BAUD_RATE_15 = 2000000,
};

// coil registers
enum {
  RESET_COIL = 0x00,
  LOW_POWER_COIL = 0x01,
  CONVERT_FLUID_DENSITY = 0x02,
  SET_FLUID_DENSITY = 0x03,
  SAVE_FLUID_DENSITY = 0x04,
  UNIT_OF_MEASUREMENT_COIL = 0x05,
  SAVE_UNIT_OF_MEASUREMENT = 0x06,
  SAVE_BAUD_RATE = 0x07,
  SAVE_SLAVE_ID = 0x08,
};

// input registers
enum {
  PRESSURE_WORD_1_IREG  = 0x00,
  PRESSURE_WORD_2_IREG = 0x01,
  TEMPERATURE_WORD_1_IREG = 0x02,
  TEMPERATURE_WORD_2_IREG = 0x03,
  DEPTH_WORD_1_IREG = 0x04,
  DEPTH_WORD_2_IREG = 0x05,
  ALTITUDE_WORD_1_IREG = 0x06,
  ALTITUDE_WORD_2_IREG = 0x07,
  PRESSURE_UNIT_WORD_1_IREG = 0x08,
  PRESSURE_UNIT_WORD_2_IREG = 0x09,
  TEMPERATURE_UNIT_IREG = 0x0A,
  DEPTH_UNIT_IREG = 0x0B,
  ALTITUDE_UNIT_IREG = 0x0C,
  FLUID_DENSITY_UNIT_WORD_1_IREG = 0x0D,
  FLUID_DENSITY_UNIT_WORD_2_IREG = 0x0E,
  FLUID_DENSITY_UNIT_WORD_3_IREG = 0x0F,
  FLUID_DENSITY_UNIT_WORD_4_IREG = 0x10,
};

// holding registers
enum {
  BAUD_RATE_HREG = 0x00,
  SLAVE_ID_HREG = 0x01,
  FLUID_DENSITY_WORD_1_HREG = 0x02,
  FLUID_DENSITY_WORD_2_HREG = 0x03,
};

union UInt16FloatUnion {
  uint16_t dataUInt16[2];
  float dataFloat;
};

union UInt16UInt32Union {
  uint16_t dataUInt16[2];
  uint32_t dataUInt32;
};

UInt16FloatUnion pressure;
UInt16FloatUnion depth;
UInt16FloatUnion temperature;
UInt16FloatUnion altitude;

uint16_t baudRate;
uint16_t slaveId;
UInt16FloatUnion fluidDensity;
bool unitOfMeasurement = 0; 

MS5837 sensor;
Modbus slave(Serial, DEFAULT_SLAVE_ID, RE_PIN, DE_PIN);

void (*reset) (void) = 0;

void transmitRS485() {
  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, HIGH);
}

void receiveRS485() {
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}

void lowPowerRS485() {
  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, LOW);
  delayMicroseconds(1);
  receiveRS485();
}

// serial print through RS485
size_t logRS485(const char *message) {
  transmitRS485();
  size_t n = Serial.println(message);
  receiveRS485();
  return n;
}

// read data from the coils
uint8_t readCoil(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_READ_COILS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case UNIT_OF_MEASUREMENT_COIL:
          slave.writeCoilToBuffer(i, unitOfMeasurement);
          break;
      }  
    }
  }

  return STATUS_OK;
}

// write data to the coils.
uint8_t writeCoil(uint8_t fc, uint16_t address, uint16_t status) {
  if (fc == FC_WRITE_COIL) {
    switch (address) {
      case UNIT_OF_MEASUREMENT_COIL:
        unitOfMeasurement = status;
        break;
      case SAVE_UNIT_OF_MEASUREMENT:
        // WARNING: USES EEPROM
        break;
      case RESET_COIL:
        if (status == 1) reset();
        break;
      case LOW_POWER_COIL:
        if (status == 1) lowPowerRS485();
        break;
      case CONVERT_FLUID_DENSITY:
        fluidDensity.dataFloat = (status == 0) ? fluidDensity.dataFloat * LBPFT3_TO_KGPM3 : fluidDensity.dataFloat * KGPM3_TO_LBPFT3;
        break;
      case SET_FLUID_DENSITY:
        if (status == 1) sensor.setFluidDensity(fluidDensity.dataFloat);
        break;
      case SAVE_FLUID_DENSITY:
        // WARNING: USES EEPROM
        break;
      case SAVE_BAUD_RATE:
        // WARNING: USES EEPROM
        break;
      case SAVE_SLAVE_ID:
        // WARNING: USES EEPROM
        break;
    }
  }

  return STATUS_OK;
}

// read data from the registers
uint8_t readRegister(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_READ_INPUT_REGISTERS) {
    sensor.read();
    pressure.dataFloat = (unitOfMeasurement == 0) ? sensor.pressure(MILLIBAR_TO_PASCAL) : sensor.pressure(MILLIBAR_TO_PSI);
    temperature.dataFloat = (unitOfMeasurement == 0) ? sensor.temperature() : sensor.temperature() * 1.8 + 32;
    depth.dataFloat = (unitOfMeasurement == 0) ? sensor.depth() : sensor.depth() * METER_TO_FEET;
    altitude.dataFloat = (unitOfMeasurement == 0) ? sensor.altitude() : sensor.altitude() * METER_TO_FEET;
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case PRESSURE_WORD_1_IREG:
          slave.writeRegisterToBuffer(PRESSURE_WORD_1_IREG, pressure.dataUInt16[1]);
          break;
        case PRESSURE_WORD_2_IREG:
          slave.writeRegisterToBuffer(PRESSURE_WORD_2_IREG, pressure.dataUInt16[0]);
          break;
        case TEMPERATURE_WORD_1_IREG:
          slave.writeRegisterToBuffer(TEMPERATURE_WORD_1_IREG, temperature.dataUInt16[1]);
          break;
        case TEMPERATURE_WORD_2_IREG:
          slave.writeRegisterToBuffer(TEMPERATURE_WORD_2_IREG, temperature.dataUInt16[0]);
          break;
        case DEPTH_WORD_1_IREG:
          slave.writeRegisterToBuffer(DEPTH_WORD_1_IREG, depth.dataUInt16[1]);
          break;
        case DEPTH_WORD_2_IREG:
          slave.writeRegisterToBuffer(DEPTH_WORD_2_IREG, depth.dataUInt16[0]);
          break;
        case ALTITUDE_WORD_1_IREG:
          slave.writeRegisterToBuffer(ALTITUDE_WORD_1_IREG, altitude.dataUInt16[1]);
          break;
        case ALTITUDE_WORD_2_IREG:
          slave.writeRegisterToBuffer(ALTITUDE_WORD_2_IREG, altitude.dataUInt16[0]);
          break;
        case PRESSURE_UNIT_WORD_1_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(PRESSURE_UNIT_WORD_1_IREG, PASCAL_UNIT);
          else slave.writeRegisterToBuffer(PRESSURE_UNIT_WORD_1_IREG, PSI_UNIT_WORD_1);
          break;
        case PRESSURE_UNIT_WORD_2_IREG:
          if (unitOfMeasurement == 1) slave.writeRegisterToBuffer(PRESSURE_UNIT_WORD_2_IREG, PSI_UNIT_WORD_2);
          break;
        case TEMPERATURE_UNIT_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(TEMPERATURE_UNIT_IREG, CELSIUS_UNIT);
          else slave.writeRegisterToBuffer(TEMPERATURE_UNIT_IREG, FAHRENHEIT_UNIT);
          break;
        case DEPTH_UNIT_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(DEPTH_UNIT_IREG, METER_UNIT);
          else slave.writeRegisterToBuffer(DEPTH_UNIT_IREG, FEET_UNIT);
          break;
        case ALTITUDE_UNIT_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(ALTITUDE_UNIT_IREG, METER_UNIT);
          else slave.writeRegisterToBuffer(ALTITUDE_UNIT_IREG, FEET_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_1_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_1_IREG, KILOGRAM_UNIT);
          else slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_1_IREG, POUND_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_2_IREG:
          slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_2_IREG, PER_SYMBOL);
          break;
        case FLUID_DENSITY_UNIT_WORD_3_IREG:
          if (unitOfMeasurement == 0) slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_3_IREG, METER_UNIT);
          else slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_3_IREG, FEET_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_4_IREG:
          slave.writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_4_IREG, CUBIC_UNIT);
          break;
      } 
    }
  }
  
  if (fc == FC_READ_HOLDING_REGISTERS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case BAUD_RATE_HREG:
          slave.writeRegisterToBuffer(BAUD_RATE_HREG, baudRate);
          break;
        case SLAVE_ID_HREG:
          slave.writeRegisterToBuffer(SLAVE_ID_HREG, slaveId);
          break;
        case FLUID_DENSITY_WORD_1_HREG:
          slave.writeRegisterToBuffer(FLUID_DENSITY_WORD_1_HREG, fluidDensity.dataUInt16[1]);
          break;
        case FLUID_DENSITY_WORD_2_HREG:
          slave.writeRegisterToBuffer(FLUID_DENSITY_WORD_2_HREG, fluidDensity.dataUInt16[0]);
          break;
      }
    }
  }

  return STATUS_OK;
}

// write data to the registers.
uint8_t writeRegister(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_WRITE_MULTIPLE_REGISTERS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case BAUD_RATE_HREG:
          baudRate = slave.readRegisterFromBuffer(BAUD_RATE_HREG);
          break;
        case SLAVE_ID_HREG:
          slaveId = slave.readRegisterFromBuffer(SLAVE_ID_HREG);
          break;
        case FLUID_DENSITY_WORD_1_HREG:
          // fluidDensity.dataUInt16[1] = slave.readRegisterFromBuffer(FLUID_DENSITY_WORD_1_HREG);
          uint16_t eepromData = 0;
          EEPROM.get(FLUID_DENSITY_WORD_1_HREG, eepromData);
          if (eepromData == 0) fluidDensity.dataUInt16[1] = 0;
          break;
        case FLUID_DENSITY_WORD_2_HREG:
          // fluidDensity.dataUInt16[0] = slave.readRegisterFromBuffer(FLUID_DENSITY_WORD_2_HREG);
          // EEPROM.get(FLUID_DENSITY_WORD_2_HREG, fluidDensity.dataUInt16[0]);
          // if (fluidDensity.dataUInt16[0] == 0) fluidDensity.dataUInt16[0] = 420;
          break;
      }
    }
  }

  return STATUS_OK;
}

void setup() {
  // setting pins
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  // set baud rate
  Serial.begin(DEFAULT_BAUD_RATE);
  slave.begin(DEFAULT_BAUD_RATE);
  // initialize
  Wire.begin();
  while(!sensor.init()) {
    logRS485("Error: Sensor initialization failed.");
    delay(1000);
  };
  receiveRS485();
  // sensor config
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(FRESHWATER_DENSITY);
  // setting registers
  slave.cbVector[CB_READ_COILS] = readCoil;
  slave.cbVector[CB_WRITE_COIL] = writeCoil;
  slave.cbVector[CB_READ_REGISTERS] = readRegister;
  slave.cbVector[CB_WRITE_MULTIPLE_REGISTERS] = writeRegister;
}

void loop() {  
  slave.poll();
}