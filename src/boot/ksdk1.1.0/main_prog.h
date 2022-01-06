uint8_t mins;
uint8_t hours;
uint8_t flashCol;
uint8_t lastReadTag[5];

void		main_printTime();
void		updateTime();
bool        timeChange();
void        showTime();
uint8_t     checkAlarm(uint8_t *hours, uint8_t *mins);
void readTag();