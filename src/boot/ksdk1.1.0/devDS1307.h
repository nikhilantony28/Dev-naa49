void		initDS1307(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterDS1307(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterDS1307(uint8_t deviceRegister, uint8_t payload); // check
WarpStatus setTimeDS1307(uint8_t seconds, uint8_t minutes, uint8_t hours);
uint8_t convertSexaHexa(uint8_t num);
uint8_t convertHexaSexa(uint8_t num);
uint8_t outputTimeDS1307(uint8_t reg);