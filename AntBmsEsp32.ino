//#include <Arduino.h>
#include "BluetoothSerial.h"
// #include "ArduinoJson.h"

BluetoothSerial SerialBT;
byte bms_addresses[3][6] = {
  {0xAA, 0xBB, 0xCC, 0xA0, 0x00, 0x11},
  {0xAA, 0xBB, 0xCC, 0xA0, 0x00, 0x22},
  {0xAA, 0xBB, 0xCC, 0xA0, 0x00, 0x33}
};

const char *pin = "22220000"; 

String addr_to_str(byte* address){
  String res = "";
  int length = 6;
	for (int i=0; i < length; i++){
             String s = String(address[i], HEX);
             if (s == "0") {
                     s = "00";
                 }
	     res = res + s ;
             if (i < length-1) {
               res = res + ":";
             }
	   }
       res.toUpperCase();
       return res;
}

String byte_to_str(byte* val,int length){
  String res = "";
	for (int i=0; i < length; i++){
             res = res + " " + String(val[i], HEX);
	   }
       return res;
}

static const uint8_t CHARGE_MOSFET_STATUS_SIZE = 16;
static const char *const CHARGE_MOSFET_STATUS[CHARGE_MOSFET_STATUS_SIZE] = {
    "Off",                           // 0x00
    "On",                            // 0x01
    "Overcharge protection",         // 0x02
    "Over current protection",       // 0x03
    "Battery full",                  // 0x04
    "Total overpressure",            // 0x05
    "Battery over temperature",      // 0x06
    "MOSFET over temperature",       // 0x07
    "Abnormal current",              // 0x08
    "Balanced line dropped string",  // 0x09
    "Motherboard over temperature",  // 0x0A
    "Unknown",                       // 0x0B
    "Unknown",                       // 0x0C
    "Discharge MOSFET abnormality",  // 0x0D
    "Unknown",                       // 0x0E
    "Manually turned off",           // 0x0F
};

static const uint8_t DISCHARGE_MOSFET_STATUS_SIZE = 16;
static const char *const DISCHARGE_MOSFET_STATUS[DISCHARGE_MOSFET_STATUS_SIZE] = {
    "Off",                           // 0x00
    "On",                            // 0x01
    "Overdischarge protection",      // 0x02
    "Over current protection",       // 0x03
    "Unknown",                       // 0x04
    "Total pressure undervoltage",   // 0x05
    "Battery over temperature",      // 0x06
    "MOSFET over temperature",       // 0x07
    "Abnormal current",              // 0x08
    "Balanced line dropped string",  // 0x09
    "Motherboard over temperature",  // 0x0A
    "Charge MOSFET on",              // 0x0B
    "Short circuit protection",      // 0x0C
    "Discharge MOSFET abnormality",  // 0x0D
    "Start exception",               // 0x0E
    "Manually turned off",           // 0x0F
};

static const uint8_t BALANCER_STATUS_SIZE = 11;
static const char *const BALANCER_STATUS[BALANCER_STATUS_SIZE] = {
    "Off",                                   // 0x00
    "Exceeds the limit equilibrium",         // 0x01
    "Charge differential pressure balance",  // 0x02
    "Balanced over temperature",             // 0x03
    "Automatic equalization",                // 0x04
    "Unknown",                               // 0x05
    "Unknown",                               // 0x06
    "Unknown",                               // 0x07
    "Unknown",                               // 0x08
    "Unknown",                               // 0x09
    "Motherboard over temperature",          // 0x0A
};

bool MyBluetoothConnect(byte* address,const char *pin) {
  byte addr[6];
  memcpy(addr,address,6);
  Serial.println("Connecting to BT: " + String(addr_to_str(addr)));
  
  //SerialBT.setPin(pin);
  SerialBT.begin("ESP32", true); 
  //SerialBT.begin("BMS-ANT16S-026", true); 
  
  bool connected = SerialBT.connect(addr);

  
	if(connected) {
		Serial.println("Blutooth connected Succesfully!");
	  }else {
    int retry = 0;   
		while(!SerialBT.connected(5000) && (retry < 5)) {
      retry++;
		  Serial.println("Failed to connect. Retry " + String(retry)); 
      delay(1000);
		}
	  }
  
  return connected;
}    

void MyBluetoothDisconnect() {
    Serial.println("BT stopping");
    SerialBT.flush();  
    SerialBT.disconnect();
    SerialBT.end();
}

bool DecodeData(byte* address, byte* data, int length){

    if (length == 140) {
      // DynamicJsonDocument doc(1024);
      auto ant_get_16bit = [&](size_t i) -> uint16_t {
          return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
        };
      auto ant_get_32bit = [&](size_t i) -> uint32_t {
          return (uint32_t(ant_get_16bit(i + 0)) << 16) | (uint32_t(ant_get_16bit(i + 2)) << 0);
        };
      //sensor.clearFields();
      //sensor.addField("addr", addr_to_str(address));
      //sensor.addField("connect_type", "esp32");
      //sensor.addField("rssi", WiFi.RSSI());
      Serial.println("Starting decode data "  + byte_to_str(data,140));
      
      float bms_v = ant_get_16bit(4) * 0.1f;
      Serial.print("TotalVoltage = ") ;
      Serial.print(bms_v) ;
      Serial.print("\n") ;

      // doc["bms_v"] = bms_v;
      //sensor.addField("bms_v", bms_v);

        //   6    0x10 0x2A: Cell voltage 1        4138 * 0.001 = 4.138V                    0.001 V
        //   8    0x10 0x2A: Cell voltage 2                4138 * 0.001 = 4.138V            0.001 V
        //  10    0x10 0x27: Cell voltage 3                4135 * 0.001 = 4.135V            0.001 V
        //  12    0x10 0x2A: Cell voltage 4                                                 0.001 V
        //  ...
        //  ...
        //  66    0x00 0x00: Cell voltage 31                                                0.001 V
        //  68    0x00 0x00: Cell voltage 32                                                0.001 V
        uint8_t cell_count = data[123];
        // doc["cell_count"] = cell_count;
        // sensor.addField("cell_count", cell_count);
        for (uint8_t i = 0; i < cell_count; i++) {
          
          float cell_v = (float) ant_get_16bit(i * 2 + 6) * 0.001f;

          // doc["cell_amps_"+String(i)] = cell_v;
          // sensor.addField("cell_amps_"+String(i), cell_v);

        }
        //  70    0x00 0x00 0x00 0x00: Current               0.0 A                          0.1 A
        
        /*uint8_t bms_current_raw = ((uint8_t) ant_get_32bit(70));
        float bms_current = 0;
        if (bms_current_raw >= 2147483648) {
            bms_current = (float) (bms_current_raw-2 * 2147483648) / 10;
        }
        else {
          bms_current = (float) bms_current_raw / 10;
        }
        
        doc["bms_current"]  = bms_current;
        sensor.addField("bms_current", bms_current);*/

          //  74    0x64: SOC                                  100 %                          1.0 %
        uint8_t soc = (uint8_t) data[74];
        // doc["soc"] = soc;
        // sensor.addField("soc", soc);

          //  75    0x02 0x53 0x17 0xC0: Total Battery Capacity Setting   39000000            0.000001 Ah
        
        
        float total_battery_capacity = (float) ant_get_32bit(75) * 0.000001f;
        // doc["total_battery_capacity"]  = total_battery_capacity;
        // sensor.addField("total_battery_capacity", total_battery_capacity);

          //  79    0x02 0x53 0x06 0x11: Battery Capacity Remaining                           0.000001 Ah
        
        float remain_ah = (float) ant_get_32bit(79) * 0.000001f;
        // doc["remain_ah"] = remain_ah;
        // sensor.addField("remain_ah", remain_ah);

          //  83    0x00 0x08 0xC7 0x8E: Battery Cycle Capacity                               0.001 Ah
        float battery_cycle_capacity = (float) ant_get_32bit(83) * 0.001f;
        // doc["battery_cycle_capacity"] = battery_cycle_capacity;
        // sensor.addField("battery_cycle_capacity", battery_cycle_capacity);
          //  87    0x00 0x08 0x57 0x20: Uptime in seconds     546.592s                       1.0 s
        
        float total_runtime = (float) ant_get_32bit(87);
        // doc["total_runtime"] = total_runtime;
        // sensor.addField("total_runtime", total_runtime);


        for (uint8_t i = 0; i < 6; i++) {
          uint8_t sensor_temp = (uint8_t) ant_get_16bit(i * 2 + 91);
          // doc["sensor_temp_"+String(i)] = sensor_temp;
          // sensor.addField("sensor_temp_"+String(i),sensor_temp);
        }

        /*float mosfet_temp = (float) ant_get_16bit(92);
        doc["mosfet_temp"] = mosfet_temp;
        sensor.addField("mosfet_temp", mosfet_temp);
        
        float balance_temp = (float) ant_get_16bit(94);
        doc["balance_temp"] = mosfet_temp;
        sensor.addField("balance_temp", balance_temp);*/

        //  103   0x01: Charge MOSFET Status
        uint8_t raw_charge_mosfet_status = data[103];
        float charge_mosfet_status_code_sensor = (float) raw_charge_mosfet_status;
        String charge_mosfet_status_text_sensor;
        if (raw_charge_mosfet_status < CHARGE_MOSFET_STATUS_SIZE) {
          charge_mosfet_status_text_sensor = CHARGE_MOSFET_STATUS[raw_charge_mosfet_status];
        } else {
          charge_mosfet_status_text_sensor = "Unknown";
        }
        byte charge_status =  (byte) (raw_charge_mosfet_status == 0x01);
        // doc["charge_status"] = charge_status;
        // sensor.addField("charge_status", charge_status);

        //  104   0x01: Discharge MOSFET Status
        uint8_t raw_discharge_mosfet_status = data[104];
        String discharge_mosfet_status_text_sensor;
        float discharge_mosfet_status_code_sensor = (float) raw_discharge_mosfet_status;
        if (raw_discharge_mosfet_status < DISCHARGE_MOSFET_STATUS_SIZE) {
          discharge_mosfet_status_text_sensor = DISCHARGE_MOSFET_STATUS[raw_discharge_mosfet_status];
        } else {
          discharge_mosfet_status_text_sensor =  "Unknown";
        }
        byte discharge_status = (byte) (raw_discharge_mosfet_status == 0x01);
        // doc["discharge_status"] = discharge_status;
        // sensor.addField("discharge_status", discharge_status);

        //  105   0x00: Balancer Status
        uint8_t raw_balancer_status = data[105];
        float balancer_status_code_sensor = (float) raw_balancer_status;
        float balancer_switch = (bool) (raw_balancer_status == 0x04);
        String balancer_status_text_sensor;
        if (raw_balancer_status < BALANCER_STATUS_SIZE) {
          balancer_status_text_sensor = BALANCER_STATUS[raw_balancer_status];
        } else {
          balancer_status_text_sensor = "Unknown";
        }
        byte balance_status = (byte) (raw_balancer_status == 0x01);
        // doc["balance_status"] = balance_status ;
        // sensor.addField("balance_status", balance_status);


        //  106   0x03 0xE8: Tire length                                                    mm
        //  108   0x00 0x17: Number of pulses per week
        //  110   0x01: Relay switch
        //  111   0x00 0x00 0x00 0x00: Current power         0W                             1.0 W
        

          float power =  (float) (int32_t) ant_get_32bit(111);
          // doc["power"] = power;
          // sensor.addField("power", power);

          float bms_current = (float) power/bms_v;
          // doc["bms_current"]  = bms_current;
          // sensor.addField("bms_current", bms_current);
          //float power_sensor = total_voltage * current;
          
          float tire_length = (float) ant_get_16bit(106);
          // doc["tire_length"] = tire_length;
          // sensor.addField("tire_length", tire_length);


          float pulses_week = (float) ant_get_16bit(108);
          // doc["pulses_week"] = pulses_week;
          // sensor.addField("pulses_week", pulses_week);


          //  115   0x0D: Cell with the highest voltage        Cell 13
        float max_voltage_cell = (float) data[115];
        // doc["max_voltage_cell"] = max_voltage_cell;
        // sensor.addField("max_voltage_cell", max_voltage_cell);

        //  116   0x10 0x2C: Maximum cell voltage            4140 * 0.001 = 4.140V          0.001 V
        float cell_max = (float) ant_get_16bit(116) * 0.001f;
        // doc["cell_max"] = cell_max;
        // sensor.addField("cell_max", cell_max);
        //  118   0x09: Cell with the lowest voltage         Cell 9
        float min_voltage_cell = (float) data[118];
        // doc["min_voltage_cell"] = min_voltage_cell;
        // sensor.addField("min_voltage_cell", min_voltage_cell);
        //  119   0x10 0x26: Minimum cell voltage            4134 * 0.001 = 4.134V          0.001 V
        float cell_min = (float) ant_get_16bit(119) * 0.001f;
        // doc["cell_min"] = cell_min;
        // sensor.addField("cell_min", cell_min);
        //  121   0x10 0x28: Average cell voltage            4136 * 0.001 = 4.136V          0.001 V
        float cell_avg = (float) ant_get_16bit(121) * 0.001f;
        // doc["cell_avg"] = cell_avg;
        // sensor.addField("cell_avg",cell_avg );

        //  123   0x0D: Battery strings                      13
        float battery_strings_sensor = (float) data[123];
        //  124   0x00 0x00: Discharge MOSFET, voltage between D-S                          0.1 V
        //  126   0x00 0x73: Drive voltage (discharge MOSFET)                               0.1 V
        //  128   0x00 0x6F: Drive voltage (charge MOSFET)                                  0.1 V
        //  130   0x02 0xA7: When the detected current is 0, the initial value of the comparator
        //  132   0x00 0x00 0x00 0x00: Battery is in balance bitmask (Bit 1 = Cell 1, Bit 2 = Cell 2, ...)
        //  136   0x11 0x62: System log / overall status bitmask?
        //  138   0x0B 0x00: CRC
   
      Serial.println("battery_strings_sensor = "+ String(battery_strings_sensor));
      // serializeJsonPretty(doc, Serial);
      return true;
    }
    else {
      return false;
      };
}


void BTReadAndDecodeData(byte* address) {
  byte start_word[6]  = {0xDB, 0xDB, 0x00, 0x00, 0x00, 0x00};
  byte data[140];
  memset(data, 0, sizeof(data));

  Serial.println("Sending test word"  + addr_to_str(start_word)); 
  
  
    SerialBT.write(start_word,6);
    delay(1000);
    int bytes_available = SerialBT.available();
    Serial.println("Bytes available: " + String(bytes_available) );
    if (bytes_available == 140) {
      int i = 0;
      while (SerialBT.available() > 0) {
          data[i] = SerialBT.read();
          i++;
      }
      Serial.println("Read bytes done! Red " + String(i));
    } else {
      Serial.println("Bytes available must be 140!");
    }
    
    if (DecodeData(address,data,sizeof(data))) {
       //MyInfluxSendData();
    }
    else {
      Serial.println("Bytes available must be 140!");
    }
    
}


void setup(void) {

  Serial.begin(115200);
  
}

void loop(void) {

  for ( int i=0; i < 3; i++ ) {
    
    Serial.println("Processing BMS address "+addr_to_str(bms_addresses[i]));
    
    if (MyBluetoothConnect(bms_addresses[i],pin)) {
        //WifiReconnect();
        BTReadAndDecodeData(bms_addresses[i]);
        MyBluetoothDisconnect();
        
    }
    delay(3000);
    
  }
  
}             
