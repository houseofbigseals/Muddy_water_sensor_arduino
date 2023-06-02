//#include <Arduino.h>
#include <ArduinoJson.h>
#include "AS7265X.h"
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

AS7265X sensor(8);  // 8 is interrupt pin, that we didnt connect
float    calData[18];
//int16_t rawData[18];


const uint8_t ds_pin = 5; 
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ds_pin);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
uint8_t ds1818b20_resolution = 12; //max resolution
//String hex_ds1818b20_addr;
// Found device 0 with address: 28CB9F79A20003C1
String hexnames = "0x";


//uint16_t freq[18] = {610, 680, 730, 760, 810, 860, 560, 585, 645, 705, 900,
// 940, 410, 435, 460, 485, 510, 535}; // latest datasheet

const int test_led_pin = 13;      // the number of the LED pin
const int white_led_pin = 2;
const int uv_led_pin = 3;
const int ir_led_pin = 4;

// rs485
uint8_t tx_pin = 8;
uint8_t rx_pin = 6;
SoftwareSerial ss(rx_pin, tx_pin); // RX, TX
uint8_t tr_pin = 7;
uint8_t rec_pin = 9;

typedef struct
{
    String status;
    float data;
    int length;
} one_result;

typedef struct
{
    String status;
    float data [18];
    int length;
} array_18_result;


//one_result one_r;
//array_18_result a18r;
//result res;

/*
typedef struct
{
    String status;
    uint16_t* data;
    int length;
} status;
*/

//float fake_spectrum_data [18] = {2.07791,33.87891,3.20188,73.92497,14.93329,7.24725,13.54624,56.10091,
//90.47527,2.53375,64.89537,3.21863,63.70171,84.50841,66.55,42.64173,14.597,10.78074};  // for debug

    /*
    example structure of master message:
    {
    "from_addr": 100,
    "to_addr": 200,
    "time": 1351824127,
    "command": "do_something_big",
    "args": {
        "arg_num_1": "some_string",
        "arg_num_2": 12312312
    }

    example of slave response:

    {
    "from_addr": 200,
    "to_addr": 100,
    "time": 1351824120,
    "command": "response",
    "args": {
        "status": "some_string",
        "data": [12.12, 13.13, …. ]
    }
    }


    */ 


// info about AS7265X device 

uint8_t as_dev_type;
uint8_t as_hv_version;
uint16_t as_fw_major_ver;
uint16_t as_fw_build_ver;
uint16_t as_fw_patch_ver; 
uint16_t as_status;
float as_temp_0;
float as_temp_1;
float as_temp_2;

void setup() 
{
    // setup leds
    pinMode(test_led_pin, OUTPUT);
    digitalWrite(test_led_pin, LOW);
    pinMode(white_led_pin, OUTPUT);
    digitalWrite(white_led_pin, LOW);
    pinMode(uv_led_pin, OUTPUT);
    digitalWrite(uv_led_pin, LOW);
    pinMode(ir_led_pin, OUTPUT);
    digitalWrite(ir_led_pin, LOW);

    // setup software serial pins
    pinMode(tr_pin, OUTPUT);
    pinMode(rx_pin, INPUT);
    pinMode(tx_pin, OUTPUT);
    pinMode(rec_pin, OUTPUT);
    digitalWrite(rec_pin, LOW);


    // setup ds18b20
    sensors.begin();
    //printAddress(tempDeviceAddress)
    // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.getAddress(tempDeviceAddress, 0);

    
    // try to copy ID of ds18b20 device into String and convert it to HEX
    for (uint8_t i = 0; i < 8; i++)
    {
        char hexname[2];
        sprintf(hexname, "%X", tempDeviceAddress[i]);
        if (tempDeviceAddress[i] < 16){
            hexnames+="0";}
        hexnames+=hexname[0];
        hexnames+=hexname[1];
    }

    //char* dsaddr_;
    //sprintf(dsaddr_, "%x", tempDeviceAddress); // convert it to hex
    //hex_ds1818b20_addr = String(dsaddr_);
    // we have knly one ds18b20 device connected, so its index on onewire bus 0 and it is constant
    // so we can use it as its address
    sensors.setResolution(tempDeviceAddress, ds1818b20_resolution);




    // tr pin is high for transmit and low for receive
    // by default we are in receive mode
    digitalWrite(tr_pin, LOW);
    ss.begin(9600);
    // set timeout for softaware serial
    ss.setTimeout(1500);


    //setup AS7265X spectrometer

    /* choices are:
    *  gain = gain_1x, gain_4x, gain_16x, gain_64x (default 16x)
    *  mode = mode0, mode1, mode2, mode3 (default mode2)
    *  intTime 1 - 255 (default 20) 
    *   integration time = intTime * 2.8 milliseconds, so 20 * 2.8 ms == 56 ms default
    *   maximum integration time = 714 ms
    */
    uint8_t gain = gain_16x, mode = mode2, intTime = 26; //original = 36

    /* choices are: 
    *  ledIndCurrent led_ind_1_mA, led_ind_2_mA, led_ind_4_mA, led_ind_8_mA
    *  ledDrvCurrent led_drv_12_5_mA, led_drv_25_mA, led_drv_50_mA, led_drv_100_mA
    */
    
    uint8_t ledIndCurrent0 = led_ind_1_mA, ledDrvCurrent0 = led_drv_12_5_mA;
    uint8_t ledIndCurrent1 = led_ind_1_mA, ledDrvCurrent1 = led_drv_12_5_mA;
    uint8_t ledIndCurrent2 = led_ind_1_mA, ledDrvCurrent2 = led_drv_12_5_mA;
    

    Wire.begin(A4); //SDA
    Wire.begin(A5); //SCL
    Wire.setClock(400000);      // set I2C frequency - 400 kHz 
    delay(1000);
    sensor.I2Cscan();

    sensor.init(gain, mode, intTime);
    //Serial.print(sensor.getTemperature(i), 0); 
    //Serial.println(" C");
    
    // disable all leds on sensor plate
    sensor.configureLed(ledIndCurrent0, ledDrvCurrent0, 0);
    sensor.disableIndLed(0);
    sensor.disableDrvLed(0);
    sensor.configureLed(ledIndCurrent1, ledDrvCurrent1, 1);
    sensor.disableIndLed(1);
    sensor.disableDrvLed(1);
    sensor.configureLed(ledIndCurrent2, ledDrvCurrent2, 2);
    sensor.disableIndLed(2);
    sensor.disableDrvLed(2);

    //Serial.println(sensor.getStatus());  // must be 106 if all is good
    //Serial.flush();
}


// https://docs.arduino.cc/learn/programming/memory-guide
// __heap_start : the beginning of the heap section. 
// __brkval : the last memory address pointer used by the heap


int freeRam() 
{
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  ? (int)&__heap_start : (int) __brkval);  
}

/*
status get_as7265x_info()
{
    status st;
    st.status = "ok";
    uint16_t free_memory = freeRam();  // diff between stack and heap in sram
    as_status = sensor.getStatus();  // must be 106 if all is good
    // get some information about the device hardware and firmware
    as_dev_type = sensor.getDevType();
    //Serial.print("AS72651 "); Serial.print("Device Type = 0x"); Serial.print(c, HEX);  Serial.println(" should be 0x40");
    as_hv_version = sensor.getHWVersion();
    //Serial.print("AS72651 "); Serial.print("HW Version = 0x"); Serial.print(d, HEX); Serial.println(" should be 0x41"); 
    as_fw_major_ver = sensor.getFWMajorVersion();
    //Serial.print("AS72651 "); Serial.print("FW Major Version = 0x"); Serial.print(e, HEX);
    as_fw_patch_ver = sensor.getFWPatchVersion();
    //Serial.print("AS72651 "); Serial.print("FW Patch Version = 0x"); Serial.print(f, HEX);
    as_fw_build_ver = sensor.getFWBuildVersion();
    //Serial.print("AS72651 "); Serial.print("FW Build Version = 0x"); Serial.print(g, HEX);
    //as_temp_0 = sensor.getTemperature(0);
    //as_temp_1 = sensor.getTemperature(1);
    //as_temp_2 = sensor.getTemperature(2);
    
    uint16_t info_arr [7] = {free_memory, as_status, as_dev_type, as_hv_version, as_fw_major_ver, as_fw_patch_ver, 
                                as_fw_build_ver};
    st.data = info_arr; //as_status, as_dev_type;
    st.length = 7;
    return st;
}
*/

one_result led_handler(const char* led_name, int state)
{
    one_result res;
    res.length = 0;
    res.data = 0;
    if(strcmp(led_name, "test") == 0)  // command for builtin led blink
    {
        if(state == 1)
        {
            digitalWrite(test_led_pin, HIGH);
            //Serial.println(F("Turning test led on")); //
            res.status = F("ok");
            return res;
        }
        else
        {
            digitalWrite(test_led_pin, LOW); 
            //Serial.println(F("Turning test led off"));  //
            res.status = F("ok");
            return res;
        }
    }
    else if(strcmp(led_name, "white") == 0)  // command for white led blink
    {
        if(state == 1)
        {
            digitalWrite(white_led_pin, HIGH);
            //Serial.println(F("Turning white led on")); //
            res.status = F("ok");
            return res;
        }
        else
        {
            digitalWrite(white_led_pin, LOW); 
            //Serial.println(F("Turning white led off"));  //
            res.status = F("ok");
            return res;
        }
    }
    else if(strcmp(led_name, "uv") == 0)  // command for uv led blink
    {
        if(state == 1)
        {
            digitalWrite(uv_led_pin, HIGH);
            //Serial.println(F("Turning uv led on")); //
            res.status = F("ok");
            return res;
        }
        else
        {
            digitalWrite(uv_led_pin, LOW); 
            //Serial.println(F("Turning uv led off"));  //
            res.status = F("ok");
            return res;
        }
    }
    else if(strcmp(led_name, "ir") == 0)  // command for ir led blink
    {
        if(state == 1)
        {
            digitalWrite(ir_led_pin, HIGH);
            //Serial.println(F("Turning ir led on")); //
            res.status = F("ok");
            return res;
        }
        else
        {
            digitalWrite(ir_led_pin, LOW); 
            //Serial.println(F("Turning ir led off"));  //
            res.status = F("ok");
            return res;
        }
    }
    else
    {
        res.status = F("error, no such led");
        return res;
    }
}

one_result get_temp()
{
    one_result res;
    sensors.requestTemperaturesByAddress(tempDeviceAddress);  // request temp
    float temp_c = sensors.getTempC(tempDeviceAddress);  // get temp from device with addr stored in tempDeviceAddress
    //sensors.getAddress(tempDeviceAddress, 0);
    // Check if reading was successful
    
    if (temp_c != DEVICE_DISCONNECTED_C)
    {
        res.data = temp_c;
        res.status = F("ok");
        res.length = 1;
    }
    else
    {
        res.status = F("DS18B20 Error");
        res.length = 0;
        res.data = 0;
    }
    return res;
}

array_18_result get_spectrum()
{
    array_18_result res;

    //float* data = fake_spectrum_data;
    sensor.readCalData(res.data);
    //res.data = calData;
    res.status = F("ok");
    res.length = 18;
    return res;
}




void loop()
{
    if(ss.available())
    {
        //one_result one_r;
        
        unsigned long time_ = 0;
        StaticJsonDocument<192> doc;  // 192 bytes was calculated by https://arduinojson.org/v6/assistant

        //r.status = "default";
        //status s;

        // reading data package
        digitalWrite(rec_pin, HIGH);
        String msg = ss.readStringUntil('\n');

        // deserialize

        DeserializationError error = deserializeJson(doc, msg);

        digitalWrite(rec_pin, LOW);

        if (error)
        {
            // clear old json msg
            doc.clear();
            // create new msg, with only error message
            StaticJsonDocument<400> ans;  // same way
            JsonObject args = ans.createNestedObject("args");
            JsonArray args_data = args.createNestedArray("data");
            // create json with answer
            ans["from"] = 2;  // only for future
            ans["to"] = 1;  // only for future
            ans["time"] = time_;  // we will send same time as a UID of answer
            ans["command"] = F("resp");
            args["status"] = error.f_str();


            // and send it back
            digitalWrite(tr_pin, HIGH);
            delay(10);
            serializeJson(ans, ss);
            digitalWrite(tr_pin, LOW);

            // clear old json msg
            //doc.garbageCollect();
            //ans.garbageCollect();
            args.clear();
            args_data.clear();
            ans.clear();
        }
        else
        {
            // parse deserialized json

            // this device is very simple, so we dont need any address comparisons or time calculating
            // it is just stub for imaginary future development of this project

            //int from_addr = doc["from"]; // 100
            //int to_addr = doc["to"]; // 200
            time_ = doc["time"]; // 1351824120  - absolute linux time in secs - we use it as UID of message

            const char* command = doc["command"]; // "do_something_big"

            const char* args_arg_num_1 = doc["args"]["arg1"]; // "some_string"
            long args_arg_num_2 = doc["args"]["arg2"]; // 12312312

            // clear old json msg
            doc.clear();

            //doc.garbageCollect();
            StaticJsonDocument<400> ans;  // same way
            JsonObject args = ans.createNestedObject("args");
            JsonArray args_data = args.createNestedArray("data");
            // create json with answer
            ans["from"] = 2;  // only for future
            ans["to"] = 1;  // only for future
            ans["time"] = time_;  // we will send same time as a UID of answer
            ans["command"] = F("resp");

            // parse command and handle it
            if(strcmp(command, "set_led") == 0)
            {
                one_result r = led_handler(args_arg_num_1, args_arg_num_2);
                args["status"] = r.status;
                args_data[0] = r.data;
            }
            else if(strcmp(command, "get_temp") == 0)
            {
                one_result r = get_temp();
                args["status"] = r.status;
                args_data[0] = r.data;
            }
            else if(strcmp(command, "get_spectrum") == 0)
            {
                array_18_result r18 = get_spectrum();
                args["status"] = r18.status;
                for(int i = 0; i<r18.length; i++)
                {
                    args_data[i] = r18.data[i];
                }
            }
            else if(strcmp(command, "get_status") == 0)
            {
                
                args["status"] = F("ok");
                args["mem"] = freeRam();
                args["asst"] = sensor.getStatus();  // must be 106 if all is good
                args["devt"] = sensor.getDevType();
                args["hvv"] = sensor.getHWVersion();
                args["fwmv"] = sensor.getFWMajorVersion();
                args["fwpv"] = sensor.getFWPatchVersion();
                args["fwbv"] = sensor.getFWBuildVersion();
                args["temp0"] = sensor.getTemperature(0);
                args["temp1"] = sensor.getTemperature(1);
                args["temp2"] = sensor.getTemperature(2);
                args["dsaddr"] = hexnames; // address of ds18b20 device 
            }
            else
            {
                args["status"] = F("no such command");
            }
        

            // send it back to master
            digitalWrite(tr_pin, HIGH);
            delay(10);
            serializeJson(ans, ss);
            digitalWrite(tr_pin, LOW);

            // clear old json msg
            //doc.garbageCollect();
            //ans.garbageCollect();
            args.clear();
            args_data.clear();
            ans.clear();
        }
    }
}
