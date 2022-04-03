class Ranging {
  public:
    uint8_t device_ID;
    const uint64_t SPEED_OF_LIGHT = 299792458L;

    uint16_t prev_seq = 0;
    uint16_t seq = 0;
    DW1000Time PollTxTime;
    DW1000Time RespRxTime;
    DW1000Time FinalTxTime;


    DW1000Time PollRxTime_T;
    DW1000Time RespRxTime_T;
    DW1000Time FinalRxTime_T;

    DW1000Time Ra;
    DW1000Time Rb;
    DW1000Time Da;
    DW1000Time Db;

    DW1000Time prev_Db;
    DW1000Time prev_Rb;

    DW1000Time RT1;
    DW1000Time RT2;

    void calculate_TIME() {
      //print_all_time();

      Ra = (RespRxTime - PollTxTime).wrap();
      Da = (FinalTxTime - RespRxTime).wrap();

      RT1 = (RespRxTime_T - PollRxTime_T).wrap();
      RT2 = (FinalRxTime_T - RespRxTime_T).wrap();
    }

    int32_t calculateTDoARange() {

      //print_all_time_fast();
      DW1000Time TDoA;
      TDoA = (RT1 * prev_Rb - prev_Db * RT2 + Da * RT1 - RT2 * Ra) / 2 / (RT1 + RT2);
      //Serial.print("ToF: ");
      //Serial.println(ToF);
      float TDoA_microseconds = TDoA.getAsMicroSeconds();

      int32_t out = (int32_t)(TDoA_microseconds * SPEED_OF_LIGHT / 1000);
      //        Serial.print("Dist: ");
      //        Serial.println(out);
      return out;

    }


    int32_t calculateTDoARange2() {
      //print_all_time_fast2();
      //print_all_time_fast();
      DW1000Time TDoA;
      TDoA = (Da * RT1 - RT2 * Ra) / (Ra + Da);
      //Serial.print("ToF: ");
      //Serial.println(ToF);
      float TDoA_microseconds = TDoA.getAsMicroSeconds();

      int32_t out = (int32_t)(TDoA_microseconds * SPEED_OF_LIGHT / 1000);
      //        Serial.print("Dist: ");
      //        Serial.println(out);
      return out;

    }

    uint32_t calculateRange() {
//      print_all_time_fast();
      DW1000Time ToF;
      ToF = ((Ra * prev_Rb) - (Da * prev_Db)) / (Ra + prev_Rb + Da + prev_Db);

      float ToF_microseconds = ToF.getAsMicroSeconds();
      //        Serial.print("ToF_microseconds: ");
      //        Serial.println(ToF_microseconds);
      Serial.println(ToF_microseconds * SPEED_OF_LIGHT / 1000);

      uint32_t out = (uint32_t)(ToF_microseconds * SPEED_OF_LIGHT / 1000);
      //Serial.print("Output: ");
      //Serial.println(output);
      return out;

    }





    void initialize() {
      PollTxTime.setTimestamp((int64_t)0);
      PollRxTime_T.setTimestamp((int64_t)0);
      RespRxTime_T.setTimestamp((int64_t)0);
      RespRxTime.setTimestamp((int64_t)0);
      FinalTxTime.setTimestamp((int64_t)0);
      FinalRxTime_T.setTimestamp((int64_t)0);
    }


    void print_all_time() {
      Serial.print("Ra: ");
      Serial.println(Ra.getAsMicroSeconds());
      Serial.print("Db: ");
      Serial.println(prev_Db.getAsMicroSeconds());
      Serial.print("RT1: ");
      Serial.println(RT1.getAsMicroSeconds());

      Serial.print("Rb: ");
      Serial.println(prev_Rb.getAsMicroSeconds());
      Serial.print("Da: ");
      Serial.println(Da.getAsMicroSeconds());
      Serial.print("RT2: ");
      Serial.println(RT2.getAsMicroSeconds());
    }


    void print_all_time_fast() {
      char buff[300];
      sprintf(buff, "Ra:%8f Da:%8f Rb:%8f Db:%8f RT1:%8f RT2:%8f ", Ra.getAsMicroSeconds(), Da.getAsMicroSeconds(), prev_Rb.getAsMicroSeconds(), prev_Db.getAsMicroSeconds(), RT1.getAsMicroSeconds(), RT2.getAsMicroSeconds());
      Serial.println(buff);

    }

    void print_all_time_fast2() {
      char buff[300];
      sprintf(buff, "Ra:%6f Da:%6f RT1:%6f RT2:%6f ", Ra.getAsMicroSeconds(), Da.getAsMicroSeconds(), RT1.getAsMicroSeconds(), RT2.getAsMicroSeconds());
      Serial.println(buff);

    }

    //    void printAll() {
    //      Serial.print("PollTxTime: ");
    //      Serial.println(PollTxTime);
    //      Serial.print("RespRxTime: ");
    //      Serial.println(RespRxTime);
    //
    //      Serial.print("RespTxTime: ");
    //      Serial.println(RespTxTime);
    //      Serial.print("FinalRxTime: ");
    //      Serial.println(FinalRxTime);
    //
    //      Serial.print("FinalTxTime: ");
    //      Serial.println(FinalTxTime);
    //      Serial.print("PollRxTime: ");
    //      Serial.println(PollRxTime);
    //
    //
    //    }

};
