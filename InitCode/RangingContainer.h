class Ranging {
  public:
    const uint64_t SPEED_OF_LIGHT = 299792458L;
    DW1000Time PollTxTime;
    DW1000Time PollRxTime;
    DW1000Time RespTxTime;
    DW1000Time RespRxTime;
    DW1000Time FinalTxTime;
    DW1000Time FinalRxTime;

    DW1000Time Ra;
    DW1000Time Rb;
    DW1000Time Da;
    DW1000Time Db;

    char buff[300];

    uint32_t calculateRange() {
     print_all_time_fast();

      Ra = (RespRxTime - PollTxTime).wrap();
      Rb = (FinalRxTime - RespTxTime).wrap();
      Da = (FinalTxTime - RespRxTime).wrap();
      Db = (RespTxTime - PollRxTime).wrap();
      /*
        Serial.print("Ra = ");
        Serial.println(Ra);
        Serial.print("Rb = ");
        Serial.println(Rb);

        Serial.print("Da = ");
        Serial.println(Da);
        Serial.print("Db = ");
        Serial.println(Db);
      */

      DW1000Time ToF;
      ToF = ((Ra * Rb) - (Da * Db)) / (Ra + Rb + Da + Db);

      //        float ToF_float = ((Ra.getAsMicroSeconds()*Rb.getAsMicroSeconds())-(Da.getAsMicroSeconds()*Db.getAsMicroSeconds()))/(Ra.getAsMicroSeconds()+Rb.getAsMicroSeconds()+Da.getAsMicroSeconds()+Db.getAsMicroSeconds());
      //        Serial.println(ToF_float*SPEED_OF_LIGHT/1000);

      //Serial.print("ToF: ");
      //Serial.println(ToF);
      float ToF_microseconds = ToF.getAsMicroSeconds();
      //        Serial.print("ToF_microseconds: ");
      //        Serial.println(ToF_microseconds);
      uint32_t out = (uint32_t)(ToF_microseconds * SPEED_OF_LIGHT / 1000);
      //Serial.print("Output: ");
      //Serial.println(output);
      return out;

    }

    void initialize() {
      PollTxTime.setTimestamp((int64_t)0);
      PollRxTime.setTimestamp((int64_t)0);
      RespTxTime.setTimestamp((int64_t)0);
      RespRxTime.setTimestamp((int64_t)0);
      FinalTxTime.setTimestamp((int64_t)0);
      FinalRxTime.setTimestamp((int64_t)0);
    }

    void printAll() {
      Serial.print("PollTxTime: ");
      Serial.println(PollTxTime);
      Serial.print("RespRxTime: ");
      Serial.println(RespRxTime);

      Serial.print("RespTxTime: ");
      Serial.println(RespTxTime);
      Serial.print("FinalRxTime: ");
      Serial.println(FinalRxTime);

      Serial.print("FinalTxTime: ");
      Serial.println(FinalTxTime);
      Serial.print("PollRxTime: ");
      Serial.println(PollRxTime);


    }

    void print_AB_fast() {
      sprintf(buff, "PollTxTime:%x RespRxTime:%x FinalTxTime:%x PollRxTime:%x RespTxTime:%x FinalRxTime:%x", PollTxTime.getTimestamp(), RespRxTime.getTimestamp(), FinalTxTime.getTimestamp(), PollRxTime.getTimestamp(), RespTxTime.getTimestamp(), FinalRxTime.getTimestamp());
      Serial.println(buff);
      sprintf(buff, "Ra:%x Da:%x Rb:%x Db:%x ", Ra.getTimestamp(), Da.getTimestamp(), Rb.getTimestamp(), Db.getTimestamp());
      Serial.println(buff);

    }

    void print_all_time_fast() {
      char buff[300];
      sprintf(buff, "Ra:%8f Da:%8f Rb:%8f Db:%8f ", Ra.getAsMicroSeconds(), Da.getAsMicroSeconds(), Rb.getAsMicroSeconds(), Db.getAsMicroSeconds());
      Serial.print("Buff: ");
      Serial.println(buff);

    }

};
