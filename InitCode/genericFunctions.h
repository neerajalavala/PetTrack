#include<DW1000.h>
#include "ProtocolConsts.h"

void setCurrentConfigs()
{
//    DW1000.setDefaults();
DW1000.setDefaults_longMsg();
}

void any_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	int i;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void any_msg_get_ts(const uint8_t *ts_field, uint64_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}


void generic_send(uint8_t *buffer_to_send, int buffer_size, int ts_index, int delayed, int randMax=0) {
    DW1000.newTransmit();
    setCurrentConfigs();
    
    uint32_t txDelay;
    switch (delayed) {
        case SEND_DELAY_FIXED: {
            txDelay = FIXED_DELAY;
            break;
        }
        case SEND_DELAY_RANDOM: {
            txDelay = FIXED_DELAY + random(randMax)*RESP_DURATION;
            break;
        }
    }

    Serial.print("txDelay: :");
    Serial.println(txDelay);
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(txDelay, DW1000Time::MILLISECONDS);
    // delay sending the message for the given amount
    
    
    txTime = DW1000.setDelay(deltaTime);

    DW1000Time currTime;
    DW1000.getSystemTimestamp(currTime);

    uint64_t txTime64 = txTime.getTimestamp();
    any_msg_set_ts(&buffer_to_send[ts_index], txTime64);

    DW1000.setData(buffer_to_send, buffer_size);
    
    DW1000.startTransmit();
    
}

uint16_t float_to_fixed_point(float in)
{
  if(in>0)
    return (uint16_t) (in*256);
  else
    return (uint16_t) -in*256;  
}

float fixed_point_to_float(uint16_t in)
{
  if(in>0)
    return  -((float)in)/256;
  else
    return ((float)in)/256;
}
