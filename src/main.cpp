/*
  Name: AHole-PT50 Sensor Firmware
  Description: A RS485 Modbus firmware for MS5837-30BA pressure 
               and temperature sensor IC.
  Author: Kyle Aranas
  License: Commercial
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MS5837.h>
#include <ModbusSlave.h>
#include <Adafruit_SleepyDog.h>
#include <avr/power.h>

// RS485 control gpios
#define RE_PIN 2
#define DE_PIN 3

// Serial number of the device.
#define SERIAL_NUMBER "69-420"

// Default values
#define DEFAULT_SLAVE_ID 0
#define DEFAULT_BAUD_RATE 2
#define DEFAULT_FRESHWATER_DENSITY_WORD_1 0x4474
#define DEFAULT_FRESHWATER_DENSITY_WORD_2 0x4000
#define DEFAULT_UNIT_OF_MEASUREMENT 0
#define DEFAULT_PRESSURE_DRIFT_WORD_1 0x0000
#define DEFAULT_PRESSURE_DRIFT_WORD_2 0x0000
#define DEFAULT_WAKE_UP_TIME_WINDOW 2000

// SI unit values
#define PASCAL_UNIT 0x5061
#define CELSIUS_UNIT 0x0043
#define METER_UNIT 0x006D
#define KILOGRAM_UNIT 0x6B67

// Imperial unit values
#define PSI_UNIT_WORD_1 0x7073
#define PSI_UNIT_WORD_2 0x0069
#define FAHRENHEIT_UNIT 0x0046
#define FEET_UNIT 0x6674
#define POUND_UNIT 0x6C62

// Shared characters among units of measurement
#define CUBIC_UNIT 0x5E33
#define PER_SYMBOL 0x002F

// Conversion macros
#define MILLIBAR_TO_PASCAL(pressure) (pressure * 100)
#define MILLIBAR_TO_PSI(pressure) (pressure * 0.0145037738)
#define PASCAL_TO_PSI(pressure) (pressure * 0.0001450377)
#define PSI_TO_PASCAL(pressure) (pressure * 6894.7572932)
#define METER_TO_FEET(distance) (distance * 3.280839895)
#define KGPM3_TO_LBPFT3(density) (density * 0.062427960576145)
#define LBPFT3_TO_KGPM3(density) (density * 16.01846337396)
#define CELSIUS_TO_FAHRENHEIT(temperature) (temperature * 1.8 + 32)

// Real value of the baud rates
enum {
  BAUD_RATE_0 = 2400,
  BAUD_RATE_1 = 4800,
  BAUD_RATE_2 = 9600,
  BAUD_RATE_3 = 14400,
  BAUD_RATE_4 = 19200,
  BAUD_RATE_5 = 28800,
  BAUD_RATE_6 = 38400,
  BAUD_RATE_7 = 57600,
  BAUD_RATE_8 = 76800,
  BAUD_RATE_9 = 115200,
  BAUD_RATE_10 = 230400,
  BAUD_RATE_11 = 250000,
  BAUD_RATE_12 = 500000,
  BAUD_RATE_13 = 1000000,
  BAUD_RATE_14 = 2000000,
};

// EEPROM registers
enum {
  BAUD_RATE_EREG = 0x00,
  SLAVE_ID_EREG = 0x01,
  UNIT_OF_MEASUREMENT_ECOIL = 0x02,
  FLUID_DENSITY_WORD_1_EREG = 0x03,
  FLUID_DENSITY_WORD_2_EREG = 0x05,
  PRESSURE_DRIFT_WORD_1_EREG = 0x07,
  PRESSURE_DRIFT_WORD_2_EREG = 0x09,
  WAKE_UP_TIME_WINDOW_EREG = 0x0B,
};

// Coil registers
enum {
  RESET_COIL = 0x00,
  POWER_MODE_COIL = 0x01,
  SAVE_BAUD_RATE_COIL = 0x02,
  SAVE_SLAVE_ID_COIL = 0x03,
  UNIT_OF_MEASUREMENT_COIL = 0x04,
  SAVED_UNIT_OF_MEASUREMENT_COIL = 0x05,
  SAVE_UNIT_OF_MEASUREMENT_COIL = 0x06,
  CONVERT_FLUID_DENSITY_COIL = 0x07,
  SAVE_FLUID_DENSITY_COIL = 0x08,
  CONVERT_PRESSURE_DRIFT_COIL = 0x09,
  SAVE_PRESSURE_DRIFT_COIL = 0x0A,
  SAVE_WAKE_UP_TIME_WINDOW_COIL = 0x0B,
  SET_DEFAULT_VALUES_COIL = 0x0C,
  SAVE_DEFAULT_VALUES_COIL = 0x0D,
};

// Input registers
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
  PRESSURE_DRIFT_UNIT_WORD_1_IREG = 0x11,
  PRESSURE_DRIFT_UNIT_WORD_2_IREG = 0x12,
  SAVED_BAUD_RATE_IREG = 0x13,
  SAVED_SLAVE_ID_IREG = 0x14,
  SAVED_FLUID_DENSITY_WORD_1_IREG = 0x15,
  SAVED_FLUID_DENSITY_WORD_2_IREG = 0x16,
  SAVED_PRESSURE_DRIFT_WORD_1_IREG = 0x17,
  SAVED_PRESSURE_DRIFT_WORD_2_IREG = 0x18,
  SAVED_WAKE_UP_TIME_WINDOW_IREG = 0x19,
  SERIAL_NUMBER_IREG = 0x1A,
};

// Holding registers
enum {
  BAUD_RATE_HREG = 0x00,
  SLAVE_ID_HREG = 0x01,
  FLUID_DENSITY_WORD_1_HREG = 0x02,
  FLUID_DENSITY_WORD_2_HREG = 0x03,
  PRESSURE_DRIFT_WORD_1_HREG = 0x04,
  PRESSURE_DRIFT_WORD_2_HREG = 0x05,
  WAKE_UP_TIME_WINDOW_HREG = 0x06,
};

union UInt16FloatUnion {
  uint16_t dataUInt16[2];
  float dataFloat;
};

UInt16FloatUnion pressure;
UInt16FloatUnion depth;
UInt16FloatUnion temperature;
UInt16FloatUnion altitude;
UInt16FloatUnion fluidDensity;
UInt16FloatUnion pressureDrift;

uint16_t wakeUpTimeWindow;
uint8_t baudRate;
uint8_t slaveId;

bool unitOfMeasurement = 0; 
bool powerMode = 0;

MS5837 sensor;
Modbus *slave;

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
}

// Get the corresponding baud rate.
uint32_t getActualBaudRate(uint8_t baudRateIdentifier) {
  uint32_t realBaudRate;

  switch (baudRateIdentifier) {
    case 0:
      realBaudRate = BAUD_RATE_0;
      break;
    case 1:
      realBaudRate = BAUD_RATE_1;
      break;
    case 2:
      realBaudRate = BAUD_RATE_2;
      break;
    case 3:
      realBaudRate = BAUD_RATE_3;
      break;
    case 4:
      realBaudRate = BAUD_RATE_4;
      break;
    case 5:
      realBaudRate = BAUD_RATE_5;
      break;
    case 6:
      realBaudRate = BAUD_RATE_6;
      break;
    case 7:
      realBaudRate = BAUD_RATE_7;
      break;
    case 8:
      realBaudRate = BAUD_RATE_8;
      break;
    case 9:
      realBaudRate = BAUD_RATE_9;
      break;
    case 10:
      realBaudRate = BAUD_RATE_10;
      break;
    case 11:
      realBaudRate = BAUD_RATE_11;
      break;
    case 12:
      realBaudRate = BAUD_RATE_12;
      break;
    case 13:
      realBaudRate = BAUD_RATE_13;
      break;
    case 14:
      realBaudRate = BAUD_RATE_14;
      break;
    default:
      realBaudRate = BAUD_RATE_2;
      break;
  }

  return realBaudRate;
}

// Serial print through RS485
size_t logRS485(const char *message) {
  transmitRS485();
  size_t n = Serial.println(message);
  receiveRS485();
  return n;
}

// Read data from the coils
uint8_t readCoil(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_READ_COILS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case POWER_MODE_COIL:
          slave->writeCoilToBuffer(POWER_MODE_COIL, powerMode);
          break;
        case UNIT_OF_MEASUREMENT_COIL:
          slave->writeCoilToBuffer(UNIT_OF_MEASUREMENT_COIL, unitOfMeasurement);
          break;
        case SAVED_UNIT_OF_MEASUREMENT_COIL:
          bool eepromUnitOfMeasurement;
          EEPROM.get(UNIT_OF_MEASUREMENT_ECOIL, eepromUnitOfMeasurement);
          slave->writeCoilToBuffer(SAVED_UNIT_OF_MEASUREMENT_COIL, eepromUnitOfMeasurement);
          break;
      }  
    }
  }

  return STATUS_OK;
}

// Write data to the coils.
uint8_t writeCoil(uint8_t fc, uint16_t address, uint16_t status) {
  if (fc == FC_WRITE_COIL) {
    switch (address) {
      case RESET_COIL:
        if (status == 1) reset();
        break;
      case POWER_MODE_COIL:
        if (status == 1 && powerMode == 0) {
          Wire.end();
          power_twi_disable();
          powerMode = 1;
        } else if (status == 0 && powerMode == 1) {
          power_twi_enable();
          Wire.begin();
          powerMode = 0;
        }
        break;
      case SAVE_BAUD_RATE_COIL:
        if (status == 1) EEPROM.put(BAUD_RATE_EREG, baudRate);
        break;
      case SAVE_SLAVE_ID_COIL:
        if (status == 1) EEPROM.put(SLAVE_ID_EREG, slaveId);
        break;
      case UNIT_OF_MEASUREMENT_COIL:
        unitOfMeasurement = status;
        break;
      case SAVE_UNIT_OF_MEASUREMENT_COIL:
        if (status == 1) EEPROM.put(UNIT_OF_MEASUREMENT_ECOIL, unitOfMeasurement);
        break;
      case CONVERT_FLUID_DENSITY_COIL:
        fluidDensity.dataFloat = (status == 0) ? LBPFT3_TO_KGPM3(fluidDensity.dataFloat) : KGPM3_TO_LBPFT3(fluidDensity.dataFloat);
        break;
      case SAVE_FLUID_DENSITY_COIL:
        if (status == 1) {
          EEPROM.put(FLUID_DENSITY_WORD_1_EREG, fluidDensity.dataUInt16[1]);
          EEPROM.put(FLUID_DENSITY_WORD_2_EREG, fluidDensity.dataUInt16[0]);
        }
        break;
      case CONVERT_PRESSURE_DRIFT_COIL:
        pressureDrift.dataFloat = (status == 0) ? PSI_TO_PASCAL(pressureDrift.dataFloat) : PASCAL_TO_PSI(pressureDrift.dataFloat);
        break; 
      case SAVE_PRESSURE_DRIFT_COIL:
        if (status == 1) {
          EEPROM.put(PRESSURE_DRIFT_WORD_1_EREG, pressureDrift.dataUInt16[1]);
          EEPROM.put(PRESSURE_DRIFT_WORD_2_EREG, pressureDrift.dataUInt16[0]);
        }
        break;
      case SAVE_WAKE_UP_TIME_WINDOW_COIL:
        if (status == 1) EEPROM.put(WAKE_UP_TIME_WINDOW_EREG, wakeUpTimeWindow);
        break;
      case SET_DEFAULT_VALUES_COIL:
        if (status == 1) {
          baudRate = DEFAULT_BAUD_RATE;
          slaveId = DEFAULT_SLAVE_ID;
          unitOfMeasurement = DEFAULT_UNIT_OF_MEASUREMENT;
          fluidDensity.dataUInt16[1] = DEFAULT_FRESHWATER_DENSITY_WORD_1;
          fluidDensity.dataUInt16[0] = DEFAULT_FRESHWATER_DENSITY_WORD_2;
          pressureDrift.dataUInt16[1] = DEFAULT_PRESSURE_DRIFT_WORD_1;
          pressureDrift.dataUInt16[0] = DEFAULT_PRESSURE_DRIFT_WORD_2;
          wakeUpTimeWindow = DEFAULT_WAKE_UP_TIME_WINDOW;
        }
        break;
      case SAVE_DEFAULT_VALUES_COIL:
        if (status == 1) {
          EEPROM.put(BAUD_RATE_EREG, DEFAULT_BAUD_RATE);
          EEPROM.put(SLAVE_ID_EREG, DEFAULT_SLAVE_ID);
          EEPROM.put(UNIT_OF_MEASUREMENT_ECOIL, DEFAULT_UNIT_OF_MEASUREMENT);
          EEPROM.put(FLUID_DENSITY_WORD_1_EREG, DEFAULT_FRESHWATER_DENSITY_WORD_1);
          EEPROM.put(FLUID_DENSITY_WORD_2_EREG, DEFAULT_FRESHWATER_DENSITY_WORD_2);
          EEPROM.put(PRESSURE_DRIFT_WORD_1_EREG, DEFAULT_PRESSURE_DRIFT_WORD_1);
          EEPROM.put(PRESSURE_DRIFT_WORD_2_EREG, DEFAULT_PRESSURE_DRIFT_WORD_2);
          EEPROM.put(WAKE_UP_TIME_WINDOW_EREG, DEFAULT_WAKE_UP_TIME_WINDOW);
        }
        break;
    }
  }

  return STATUS_OK;
}

// Read data from the registers
uint8_t readRegister(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_READ_INPUT_REGISTERS && powerMode == 0) {
    sensor.read();
    pressure.dataFloat = (unitOfMeasurement == 0) ? MILLIBAR_TO_PASCAL(sensor.pressure()) + pressureDrift.dataFloat : MILLIBAR_TO_PSI(sensor.pressure()) + pressureDrift.dataFloat;
    temperature.dataFloat = (unitOfMeasurement == 0) ? sensor.temperature() : CELSIUS_TO_FAHRENHEIT(sensor.temperature());
    depth.dataFloat = (unitOfMeasurement == 0) ? sensor.depth(fluidDensity.dataFloat) : METER_TO_FEET(sensor.depth(LBPFT3_TO_KGPM3(fluidDensity.dataFloat)));
    altitude.dataFloat = (unitOfMeasurement == 0) ? sensor.altitude() : METER_TO_FEET(sensor.altitude());
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case PRESSURE_WORD_1_IREG:
          slave->writeRegisterToBuffer(PRESSURE_WORD_1_IREG, pressure.dataUInt16[1]);
          break;
        case PRESSURE_WORD_2_IREG:
          slave->writeRegisterToBuffer(PRESSURE_WORD_2_IREG, pressure.dataUInt16[0]);
          break;
        case TEMPERATURE_WORD_1_IREG:
          slave->writeRegisterToBuffer(TEMPERATURE_WORD_1_IREG, temperature.dataUInt16[1]);
          break;
        case TEMPERATURE_WORD_2_IREG:
          slave->writeRegisterToBuffer(TEMPERATURE_WORD_2_IREG, temperature.dataUInt16[0]);
          break;
        case DEPTH_WORD_1_IREG:
          slave->writeRegisterToBuffer(DEPTH_WORD_1_IREG, depth.dataUInt16[1]);
          break;
        case DEPTH_WORD_2_IREG:
          slave->writeRegisterToBuffer(DEPTH_WORD_2_IREG, depth.dataUInt16[0]);
          break;
        case ALTITUDE_WORD_1_IREG:
          slave->writeRegisterToBuffer(ALTITUDE_WORD_1_IREG, altitude.dataUInt16[1]);
          break;
        case ALTITUDE_WORD_2_IREG:
          slave->writeRegisterToBuffer(ALTITUDE_WORD_2_IREG, altitude.dataUInt16[0]);
          break;
        case PRESSURE_UNIT_WORD_1_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(PRESSURE_UNIT_WORD_1_IREG, PASCAL_UNIT);
          else slave->writeRegisterToBuffer(PRESSURE_UNIT_WORD_1_IREG, PSI_UNIT_WORD_1);
          break;
        case PRESSURE_UNIT_WORD_2_IREG:
          if (unitOfMeasurement == 1) slave->writeRegisterToBuffer(PRESSURE_UNIT_WORD_2_IREG, PSI_UNIT_WORD_2);
          break;
        case TEMPERATURE_UNIT_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(TEMPERATURE_UNIT_IREG, CELSIUS_UNIT);
          else slave->writeRegisterToBuffer(TEMPERATURE_UNIT_IREG, FAHRENHEIT_UNIT);
          break;
        case DEPTH_UNIT_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(DEPTH_UNIT_IREG, METER_UNIT);
          else slave->writeRegisterToBuffer(DEPTH_UNIT_IREG, FEET_UNIT);
          break;
        case ALTITUDE_UNIT_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(ALTITUDE_UNIT_IREG, METER_UNIT);
          else slave->writeRegisterToBuffer(ALTITUDE_UNIT_IREG, FEET_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_1_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_1_IREG, KILOGRAM_UNIT);
          else slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_1_IREG, POUND_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_2_IREG:
          slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_2_IREG, PER_SYMBOL);
          break;
        case FLUID_DENSITY_UNIT_WORD_3_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_3_IREG, METER_UNIT);
          else slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_3_IREG, FEET_UNIT);
          break;
        case FLUID_DENSITY_UNIT_WORD_4_IREG:
          slave->writeRegisterToBuffer(FLUID_DENSITY_UNIT_WORD_4_IREG, CUBIC_UNIT);
          break;
        case PRESSURE_DRIFT_UNIT_WORD_1_IREG:
          if (unitOfMeasurement == 0) slave->writeRegisterToBuffer(PRESSURE_DRIFT_UNIT_WORD_1_IREG, PASCAL_UNIT);
          else slave->writeRegisterToBuffer(PRESSURE_DRIFT_UNIT_WORD_1_IREG, PSI_UNIT_WORD_1);
          break;
        case PRESSURE_DRIFT_UNIT_WORD_2_IREG:
          if (unitOfMeasurement == 1) slave->writeRegisterToBuffer(PRESSURE_DRIFT_UNIT_WORD_2_IREG, PSI_UNIT_WORD_2);
          break;
        case SAVED_BAUD_RATE_IREG:
          uint8_t eepromBaudRate;
          EEPROM.get(BAUD_RATE_EREG, eepromBaudRate);
          slave->writeRegisterToBuffer(SAVED_BAUD_RATE_IREG, eepromBaudRate);
          break;
        case SAVED_SLAVE_ID_IREG:
          uint8_t eepromSlaveId;
          EEPROM.get(SLAVE_ID_EREG, eepromSlaveId);
          slave->writeRegisterToBuffer(SAVED_SLAVE_ID_IREG, eepromSlaveId);
          break;
        case SAVED_FLUID_DENSITY_WORD_1_IREG:
          uint16_t eepromFluidDensityWord1;
          EEPROM.get(FLUID_DENSITY_WORD_1_EREG, eepromFluidDensityWord1);
          slave->writeRegisterToBuffer(SAVED_FLUID_DENSITY_WORD_1_IREG, eepromFluidDensityWord1);
          break;
        case SAVED_FLUID_DENSITY_WORD_2_IREG:
          uint16_t eepromFluidDensityWord2;
          EEPROM.get(FLUID_DENSITY_WORD_2_EREG, eepromFluidDensityWord2);
          slave->writeRegisterToBuffer(SAVED_FLUID_DENSITY_WORD_2_IREG, eepromFluidDensityWord2);
          break;
        case SAVED_PRESSURE_DRIFT_WORD_1_IREG:
          uint16_t eepromPressureDriftWord1;
          EEPROM.get(PRESSURE_DRIFT_WORD_1_EREG, eepromPressureDriftWord1);
          slave->writeRegisterToBuffer(SAVED_PRESSURE_DRIFT_WORD_1_IREG, eepromPressureDriftWord1);
          break;
        case SAVED_PRESSURE_DRIFT_WORD_2_IREG:
          uint16_t eepromPressureDriftWord2;
          EEPROM.get(PRESSURE_DRIFT_WORD_2_EREG, eepromPressureDriftWord2);
          slave->writeRegisterToBuffer(SAVED_PRESSURE_DRIFT_WORD_2_IREG, eepromPressureDriftWord2);
          break;
        case SAVED_WAKE_UP_TIME_WINDOW_IREG:
          uint16_t eepromWakeUpTimeWindow;
          EEPROM.get(WAKE_UP_TIME_WINDOW_EREG, eepromWakeUpTimeWindow);
          slave->writeRegisterToBuffer(SAVED_WAKE_UP_TIME_WINDOW_IREG, eepromWakeUpTimeWindow);
          break;
        case SERIAL_NUMBER_IREG:
          for (uint8_t i = 0; i < sizeof(SERIAL_NUMBER) - 1; i++) slave->writeRegisterToBuffer(SERIAL_NUMBER_IREG + i, SERIAL_NUMBER[i]);
          break;
      } 
    }
  }
  
  if (fc == FC_READ_HOLDING_REGISTERS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case BAUD_RATE_HREG:
          slave->writeRegisterToBuffer(BAUD_RATE_HREG, baudRate);
          break;
        case SLAVE_ID_HREG:
          slave->writeRegisterToBuffer(SLAVE_ID_HREG, slaveId);
          break;
        case FLUID_DENSITY_WORD_1_HREG:
          slave->writeRegisterToBuffer(FLUID_DENSITY_WORD_1_HREG, fluidDensity.dataUInt16[1]);
          break;
        case FLUID_DENSITY_WORD_2_HREG:
          slave->writeRegisterToBuffer(FLUID_DENSITY_WORD_2_HREG, fluidDensity.dataUInt16[0]);
          break;
        case PRESSURE_DRIFT_WORD_1_HREG:
          slave->writeRegisterToBuffer(PRESSURE_DRIFT_WORD_1_HREG, pressureDrift.dataUInt16[1]);
          break;
        case PRESSURE_DRIFT_WORD_2_HREG:
          slave->writeRegisterToBuffer(PRESSURE_DRIFT_WORD_2_HREG, pressureDrift.dataUInt16[0]);
          break;
        case WAKE_UP_TIME_WINDOW_HREG:
          slave->writeRegisterToBuffer(WAKE_UP_TIME_WINDOW_HREG, wakeUpTimeWindow);
          break;
      }
    }
  }

  return STATUS_OK;
}

// Write data to the registers.
uint8_t writeRegister(uint8_t fc, uint16_t address, uint16_t length) {
  if (fc == FC_WRITE_MULTIPLE_REGISTERS) {
    for (uint16_t i = 0; i < length; i++) {
      switch (address + i) {
        case BAUD_RATE_HREG:
          baudRate = slave->readRegisterFromBuffer(BAUD_RATE_HREG);
          break;
        case SLAVE_ID_HREG:
          slaveId = slave->readRegisterFromBuffer(SLAVE_ID_HREG);
          break;
        case FLUID_DENSITY_WORD_1_HREG:
          fluidDensity.dataUInt16[1] = slave->readRegisterFromBuffer(FLUID_DENSITY_WORD_1_HREG);
          break;
        case FLUID_DENSITY_WORD_2_HREG:
          fluidDensity.dataUInt16[0] = slave->readRegisterFromBuffer(FLUID_DENSITY_WORD_2_HREG);
          break;
        case PRESSURE_DRIFT_WORD_1_HREG:
          pressureDrift.dataUInt16[1] = slave->readRegisterFromBuffer(PRESSURE_DRIFT_WORD_1_HREG);
          break;
        case PRESSURE_DRIFT_WORD_2_HREG:
          pressureDrift.dataUInt16[0] = slave->readRegisterFromBuffer(PRESSURE_DRIFT_WORD_2_HREG);
          break;
        case WAKE_UP_TIME_WINDOW_HREG:
          wakeUpTimeWindow = slave->readRegisterFromBuffer(WAKE_UP_TIME_WINDOW_HREG);
          break;
      }
    }
  }

  return STATUS_OK;
}

void setup() {
  // Disable unused features
  ADCSRA = 0;
  power_adc_disable();
  power_spi_disable();
  power_timer1_disable();
  power_timer2_disable();
  // Get EEPROM data
  EEPROM.get(BAUD_RATE_EREG, baudRate);
  EEPROM.get(SLAVE_ID_EREG, slaveId);
  EEPROM.get(UNIT_OF_MEASUREMENT_ECOIL, unitOfMeasurement);
  EEPROM.get(FLUID_DENSITY_WORD_1_EREG, fluidDensity.dataUInt16[1]);
  EEPROM.get(FLUID_DENSITY_WORD_2_EREG, fluidDensity.dataUInt16[0]);
  EEPROM.get(PRESSURE_DRIFT_WORD_1_EREG, pressureDrift.dataUInt16[1]);
  EEPROM.get(PRESSURE_DRIFT_WORD_2_EREG, pressureDrift.dataUInt16[0]);
  EEPROM.get(WAKE_UP_TIME_WINDOW_EREG, wakeUpTimeWindow);
  // Setting pins
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  slave = new Modbus(Serial, slaveId, RE_PIN, DE_PIN);
  // Set baud rate
  Serial.begin(getActualBaudRate(baudRate));
  slave->begin(getActualBaudRate(baudRate));
  // Initialize
  Wire.begin();
  while(!sensor.init()) {
    logRS485("Error: Sensor initialization failed.");
    delay(3000);
  };
  receiveRS485();
  // Sensor config
  sensor.setModel(MS5837::MS5837_30BA);
  // Setting registers
  slave->cbVector[CB_READ_COILS] = readCoil;
  slave->cbVector[CB_WRITE_COIL] = writeCoil;
  slave->cbVector[CB_READ_REGISTERS] = readRegister;
  slave->cbVector[CB_WRITE_MULTIPLE_REGISTERS] = writeRegister;
}

void loop() {  
  slave->poll();
  if (powerMode == 1) {
    lowPowerRS485();
    Watchdog.sleep(8000);
    receiveRS485();
    for (uint16_t start = millis(); (uint16_t)(millis() - start) < wakeUpTimeWindow;) slave->poll();
  }
}