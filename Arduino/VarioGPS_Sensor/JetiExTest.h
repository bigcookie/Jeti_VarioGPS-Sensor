typedef struct /*_JetiSensorConst*/
{
  uint8_t id;
  char    text[20];
  char    unit[7];
  uint8_t dataType;
  uint8_t precision;
}
JetiSensorConst;
typedef const JetiSensorConst JETISENSOR_CONST;


class JetiExProtocol;
class JetiSensor
{
public:
  // Jeti data types
  enum enDataType
  {
    TYPE_6b   = 0, // int6_t  Data type 6b (-31 ¸31)
    TYPE_14b  = 1, // int14_t Data type 14b (-8191 ¸8191)
    TYPE_22b  = 4, // int22_t Data type 22b (-2097151 ¸2097151)
    TYPE_DT   = 5, // int22_t Special data type <96> time and date
    TYPE_30b  = 8, // int30_t Data type 30b (-536870911 ¸536870911) 
    TYPE_GPS  = 9, // int30_t Special data type <96> GPS coordinates:  lo/hi minute - lo/hi degree. 
  }
  EN_DATA_TYPE;

  class JetiValue {
    public:
      JetiValue() : m_value( -1 ) {}
    protected:
      // value
      int32_t m_value;
  };


  JetiSensor( int arrIdx, JetiExProtocol * pProtocol );

  // sensor id
  uint8_t m_id;

  // value
  int32_t m_value;

  // value
  uint8_t m_bActive;

  // label/description of value
  uint8_t m_label[ 20 ];
  uint8_t m_textLen;
  uint8_t m_unitLen;

  // format
  uint8_t  m_dataType;
  uint8_t  m_precision; 
  uint8_t  m_bufLen;

  // helpers
  void    copyLabel( const uint8_t * text, const uint8_t * unit,  uint8_t * label, int label_size, uint8_t * textLen, uint8_t * unitLen );
  uint8_t jetiCopyLabel( uint8_t * exbuf, uint8_t n );
  uint8_t jetiEncodeValue( uint8_t * exbuf, uint8_t n );
};


class JetiExProtocol {
  private:
    int32_t * myValues = NULL;
    JETISENSOR_CONST *mySensors;
    uint32_t mySensorCount = 0;
    JetiSensorConst * getSensor(uint8_t id) {
      JetiSensorConst sens;
      for (int i = 0 ; i < mySensorCount; i++) {
        memcpy_P( &sens, &mySensors[i], sizeof(sens));
	if (id == sens.id) {
          return &sens;
	}
      }
      return NULL;
    };
  public :
     enum enLineNo
  {
    LINE1 = 0,
    LINE2 = 1
  };

  enum enJetiboxKey
  {
    DOWN  = 0xb0,
    UP    = 0xd0,
    LEFT  = 0x70,
    RIGHT = 0xe0,
  };

    enum enComPort {
     DEFAULTPORT = 0x00,
     SERIAL1     = 0x01,
     SERIAL2     = 0x02,
     SERIAL3     = 0x03,
    };
    JetiExProtocol() {};
    uint8_t DoJetiSend() {
      Serial.println("DoJetiSend:");
      JetiSensorConst *  sensorConst;
      for (int i = 0 ; i < mySensorCount; i++) {
       	sensorConst = getSensor(i+1);
        // Serial.print(" -> ");
        // Serial.print(sensorConst->id);
        // Serial.print(" : ");
        // Serial.print(sensorConst->text);
        // Serial.print(" = ");
        // Serial.println(myValues[sensorConst->id]);
      }
    };
    void SetSensorActive( uint8_t id, bool bEnable, JETISENSOR_CONST * pSensorArray  ) {};
    void SetDeviceId( uint8_t idLo, uint8_t idHi ) {};
    void    Start( const char * name,  JETISENSOR_CONST * pSensorArray, enComPort comPort = DEFAULTPORT ) {
      mySensors = pSensorArray;
      JetiSensorConst sensorConst;

      for (int i = 0 ; i < 100; i++) {
        memcpy_P( &sensorConst, &pSensorArray[i], sizeof(sensorConst) );
	if (sensorConst.id == 0) {
          mySensorCount = i;
          break;
	}
      }
      myValues = new int32_t[mySensorCount];
      for (int i = 0 ; i < mySensorCount; i++) {
        memcpy_P( &sensorConst, &pSensorArray[i], sizeof(sensorConst) );
         Serial.print(sensorConst.id);
         Serial.print(":");
         Serial.print(sensorConst.text);
         Serial.println(":");
      }
    };
    void SetSensorValue( uint8_t id, int32_t value ) {
      myValues[id] = value;
      // JetiSensorConst * sens = getSensor(id);
      // Serial.print(F("SetSensorValue["));
      // Serial.print(id);
      // Serial.print(F("] :"));
      // Serial.print(sens->text);
      // Serial.print(myValues[id]);
      // Serial.println(sens->unit);
    };
    void SetSensorValueGPS( uint8_t id, bool bLongitude, float value ) {
    };
    uint8_t GetJetiboxKey() {};
    void SetJetiboxText( enLineNo lineNo, const char* text ){};
    // void SetSensorValueDate( uint8_t id, uint8_t day, uint8_t month, uint16_t year ) {};
    // void SetSensorValueTime( uint8_t id, uint8_t hour, uint8_t minute, uint8_t second ) {};
};	

