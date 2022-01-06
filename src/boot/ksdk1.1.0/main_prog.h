uint8_t mins;
uint8_t hours;
uint8_t flashCol;
uint64_t lastReadTag;
bool alarmState;

void		main_printTime();
void		updateTime();
bool        timeChange();
void        showTime();
uint8_t     checkAlarm(uint8_t *hours, uint8_t *mins, bool alarmOn);
void readTag();
bool checkTag(uint64_t savedData);