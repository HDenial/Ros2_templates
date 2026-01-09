#naming convention: StartWithUpperCaseNoUnderscore.msg

float64 temperature
bool are_motors_read
string status

#define message type followed by variable name
#supported types: bool, byte, char, float32, float64, int8, int16, int32, int64, uint8, uint16, uint32, uint64, string, time, duration
#you can also use arrays by adding [] after the type (e.g. int32[] data)