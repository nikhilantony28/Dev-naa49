void		initDS1307(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterDS1307(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterDS1307(uint8_t deviceRegister, uint8_t payload); // check
WarpStatus	configureSensorDS1307(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1); // edit
void		printSensorDataDS1307(bool hexModeFlag);