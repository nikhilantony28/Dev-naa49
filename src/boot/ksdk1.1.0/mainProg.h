uint8_t mins;
uint8_t hours;
uint8_t flashCol;
uint64_t lastReadTag;
bool alarmState;

void		main_printTime();
void		updateTime();
bool        timeChange();
void        showTime();
int     checkAlarm(uint8_t *hours, uint8_t *mins);
void readTag();
bool checkTag(uint64_t savedData);
void enterPillName(char *name, uint8_t loc);