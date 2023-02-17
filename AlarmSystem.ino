/**************************************************************************************************************************************************
* File name     : AlarmSystem.ino
* Compiler      : 
* Autor         : Vigasan   
* Created       : 13/02/2023
* Modified      : 
* Last modified :
*
*
* Description   : 
*
* Other info    : Alarm System Application for ESP32
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <WiFi.h>
#include <PubSubClient.h>     // Remember to edit this file and change MQTT_MAX_PACKET_SIZE from default 256 to 600
#include <ArduinoJson.h>

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Local definitions------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

#define PERIOD_MILLSEC_1000    1000
#define PERIOD_MILLSEC_500     500
#define PERIOD_MILLSEC_250     250

#define DELTA_VOLTAGE          0.05

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             COMMUNICATION PROTOCOL DEFINES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LODWORD(l)  (*( (unsigned long*)(&l)))
#define HIDWORD(l)  (*( ( (unsigned long*) (&l) ) + 1 ))
#define MAKEDWORD(hi,lo) ((unsigned long)(((unsigned long)(lo)) | (((unsigned long)(hi))<<16)))
//Word manipulation
#define LOWORD(l)   (*( (word*)(&l)))
#define HIWORD(l)   (*( ( (word*) (&l) ) + 1 ))
#define MAKEWORD(hi,lo) ((word)(((word)(lo)) | (((word)(hi))<<8)))
//Byte manipulation
#define LOBYTE(w)   (*( (byte*) (&w)))
#define HIBYTE(w)   (*( ( (byte*)  (&w) ) + 1 ))
#define UPBYTE(w)   (*( ( (byte*)  (&w) ) + 2 ))
#define MSBYTE(w)   (*( ( (byte*)  (&w) ) + 3 ))


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             PROTOCOL FRAME DEFINES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GPL_ST_IDLE                     0
#define GPL_ST_CMD                      1
#define GPL_ST_NUM_BYTE                 2
#define GPL_ST_DATA                     3
#define GPL_ST_CHECKSUM                 4

#define GPL_START_FRAME                 0x8A
#define GPL_ESCAPE_CHAR                 0x8B
#define GPL_XOR_CHAR                    0x20

#define GPL_CMD_TIMEOUT                 1000        // Frame Timeout

#define GPL_BYTE_SOF                    0
#define GPL_BYTE_CMD                    1
#define GPL_BYTE_LENGTH                 2
#define GPL_BYTE_FIRST_DATA             3

#define GPL_NUM_EXTRA_BYTES             4           

#define LEN_IN_Serial2_BUFFER           100
#define LEN_OUT_Serial2_BUFFER          30
#define LEN_OUT_FRAME_BUFFER            30


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                     CMD ID FOR PROTOCOL DATA EXCHANGE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------ Global CMDs ---------------------------------------------------
#define CMD_CONFIRM                             0
#define CMD_IS_BOOTLOADER                       1
#define CMD_PROGRAM_VERSION                     2
#define CMD_GOTO_BOOTLOADER                     3
#define CMD_DEVICE_ID                           4
#define CMD_DELAYED_OPERATION                   5
#define CMD_HW_VERSION                          6
#define CMD_QUIT_BOOTLOADER                     7
#define CMD_FLASH_DATA                          8
#define CMD_EEPROM_DATA                         9

//------------------------------------------------------ Application Specific CMD s--------------------------------------
#define CMD_GET_AIN1                            10
#define CMD_GET_AIN2                            11
#define CMD_GET_AIN3                            12
#define CMD_GET_AIN4                            13
#define CMD_GET_AIN5                            14
#define CMD_GET_AIN6                            15
#define CMD_GET_AIN7                            16
#define CMD_GET_AIN8                            17
#define CMD_GET_DIN1                            18
#define CMD_GET_DIN2                            19
#define CMD_GET_DIN3                            20
#define CMD_GET_DIN4                            21
#define CMD_SET_THR_AIN1                        22
#define CMD_GET_THR_AIN1                        23
#define CMD_SET_THR_AIN2                        24
#define CMD_GET_THR_AIN2                        25
#define CMD_SET_THR_AIN3                        26
#define CMD_GET_THR_AIN3                        27
#define CMD_SET_THR_AIN4                        28
#define CMD_GET_THR_AIN4                        29
#define CMD_SET_THR_AIN5                        30
#define CMD_GET_THR_AIN5                        31
#define CMD_SET_THR_AIN6                        32
#define CMD_GET_THR_AIN6                        33
#define CMD_SET_THR_AIN7                        34
#define CMD_GET_THR_AIN7                        35
#define CMD_SET_THR_AIN8                        36
#define CMD_GET_THR_AIN8                        37
#define CMD_GET_INPUTS                          38
#define CMD_SET_AUX_POWER                       39
#define CMD_GET_AUX_POWER                       40
#define CMD_SET_TEST_BATTERY                    41
#define CMD_GET_TEST_BATTERY                    42
#define CMD_SET_RELAY2                          43
#define CMD_GET_RELAY2                          44
#define CMD_SET_RELAY3                          45
#define CMD_GET_RELAY3                          46
#define CMD_SET_RELAY4                          47
#define CMD_GET_RELAY4                          48
#define CMD_SET_RELAY5                          49
#define CMD_GET_RELAY5                          50
#define CMD_SET_SYSTEM_STATUS                   51
#define CMD_GET_SYSTEM_STATUS                   52
#define CMD_SET_GUI_STATUS                      53
#define CMD_GET_V_EXT                           54
#define CMD_GET_V_BATT                          55
#define CMD_GET_V_BOARD                         56
#define CMD_SET_BUS_POWER                       57
#define CMD_GET_BUS_POWER                       58
#define CMD_SET_DATE_TIME                       59
#define CMD_GET_DATE_TIME                       60
#define CMD_INIT_SYSTEM                         61
#define CMD_EXT_POWER_STATUS                    62
#define CMD_OUTPUTS                             63


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT DISCOVERY PARAMETERS----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*         g_ssid = "Wifi Name";                           // Wifi SSID Name
const char*         g_password = "Wifi Passoword";                  // Wifi Password
const char*         g_mqtt_server = "192.168.1.25";                 // IP Address of MQTT Broker (Maybe same of Home Assistant)
const char*         g_mqttUser = "mosquitto";                       // MQTT Broker User
const char*         g_mqttPsw = "psw";                              // MQTT Broker Password
int                 g_mqttPort = 1883;                              // MQTT Broker Port

const char*         g_mqtt_DeviceName = "IOTMB_Alarm";              // Your Device Name

String              g_TopicState_ain1 = "IOTMB_Alarm/ain1/state";   // State Topic for entity input ain1
String              g_TopicState_ain2 = "IOTMB_Alarm/ain2/state";   // State Topic for entity input ain2
String              g_TopicState_ain3 = "IOTMB_Alarm/ain3/state";   // State Topic for entity input ain3
String              g_TopicState_ain4 = "IOTMB_Alarm/ain4/state";   // State Topic for entity input ain4
String              g_TopicState_ain5 = "IOTMB_Alarm/ain5/state";   // State Topic for entity input ain5
String              g_TopicState_ain6 = "IOTMB_Alarm/ain6/state";   // State Topic for entity input ain6
String              g_TopicState_ain7 = "IOTMB_Alarm/ain7/state";   // State Topic for entity input ain7
String              g_TopicState_ain8 = "IOTMB_Alarm/ain8/state";   // State Topic for entity input ain8

String              g_TopicState_din1 = "IOTMB_Alarm/din1/state";   // State Topic for entity input din1
String              g_TopicState_din2 = "IOTMB_Alarm/din2/state";   // State Topic for entity input din2
String              g_TopicState_din3 = "IOTMB_Alarm/din3/state";   // State Topic for entity input din3
String              g_TopicState_din4 = "IOTMB_Alarm/din4/state";   // State Topic for entity input din4

String              g_TopicRelay1 = "IOTMB_Alarm/relay1";           // Topic for entity relay1
String              g_TopicRelay2 = "IOTMB_Alarm/relay2";           // Topic for entity relay2
String              g_TopicRelay3 = "IOTMB_Alarm/relay3";           // Topic for entity relay3
String              g_TopicRelay4 = "IOTMB_Alarm/relay4";           // Topic for entity relay4
String              g_TopicRelayBell = "IOTMB_Alarm/relayBell";     // Topic for entity relayBell (Alarm Siren)
String              g_TopicSwitchBus = "IOTMB_Alarm/switchBus";     // Topic for entity switch Power Bus
String              g_TopicSwitchTestBattery = "IOTMB_Alarm/switchTestBatt";  // Topic for entity switch Test Battery

String              g_TopicPowerState = "IOTMB_Alarm/power/state";  // State Topic for entity Power (Check for blackout)
String              g_TopicVoltBattery = "IOTMB_Alarm/battery/state"; // State Topic for entity Battery

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Generic global variables-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
int                 g_mqttCounterConn = 0;
String              g_UniqueId;
bool                g_InitSystem = true;
byte                g_InputBuffer[LEN_IN_Serial2_BUFFER];
byte                g_OutputBuffer[LEN_OUT_Serial2_BUFFER];
byte                g_FrameBuffer[LEN_OUT_FRAME_BUFFER];
byte                dataReceived = 0;
byte                gplStatus = GPL_ST_IDLE;
byte                xored = 0x00;
byte                checkSum;
int                 numByte = 0, i = 0;
double              g_VoltagePowerSupply = 0.0;
double              g_VoltageBoard = 0.0;
double              g_VoltageBattery = 0.0;
word                g_Inputs = 0;
byte                g_Outputs = 0;
double              g_oldVoltagePowerSupply = 1.0;
double              g_oldVoltageBoard = 1.0;
double              g_oldVoltageBattery = 1.0;
word                g_oldInputs = 1;
byte                g_oldOutputs = 1;
int                 g_canPublish = 0;
double              delta = 0.0;
unsigned long       g_Time = 0;
word                g_aIn1 = 0;
word                g_old_aIn1 = 0xFFFF;
word                g_aIn2 = 0;
word                g_old_aIn2 = 0xFFFF;
word                g_aIn3 = 0;
word                g_old_aIn3 = 0xFFFF;
word                g_aIn4 = 0;
word                g_old_aIn4 = 0xFFFF;
word                g_aIn5 = 0;
word                g_old_aIn5 = 0xFFFF;
word                g_aIn6 = 0;
word                g_old_aIn6 = 0xFFFF;
word                g_aIn7 = 0;
word                g_old_aIn7 = 0xFFFF;
word                g_aIn8 = 0;
word                g_old_aIn8 = 0xFFFF;
word                g_dIn1 = 0;
word                g_old_dIn1 = 0xFFFF;
word                g_dIn2 = 0;
word                g_old_dIn2 = 0xFFFF;
word                g_dIn3 = 0;
word                g_old_dIn3 = 0xFFFF;
word                g_dIn4 = 0;
word                g_old_dIn4 = 0xFFFF;
byte                g_Relay1 = 0;
byte                g_oldRelay1 = 0xFF;
byte                g_Relay2 = 0;
byte                g_oldRelay2 = 0xFF;
byte                g_Relay3 = 0;
byte                g_oldRelay3 = 0xFF;
byte                g_Relay4 = 0;
byte                g_oldRelay4 = 0xFF;
byte                g_RelayBell = 0;
byte                g_oldRelayBell = 0xFF;
byte                g_SwitchBus = 0;
byte                g_oldSwitchBus = 0xFF;
byte                g_SwitchTestBattery = 0;
byte                g_oldSwitchTestBattery = 0xFF;
String              g_PowerStatus = "OFF";
String              g_oldPowerStatus = "ON";

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() 
{
    Serial.begin(115200);
    Serial2.begin(9600);
    delay(100); //Take some time to open up the Serial2 Monitor

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Configurazione I/O
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("");
    Serial.println("");
    Serial.println("-------- IOTMB ALARM ----------");
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Wifi Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    } else
    {
        Serial.println("WiFi NOT connected!!!");
        setup_wifi();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Discovery Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(g_InitSystem)
    {
        delay(1000);
        g_InitSystem = false;

        MqttHomeAssistantDiscovery();

        delay(1000);
        Serial.println("INIT BOARD...");
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_INIT_SYSTEM, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Serial2 - Communication with ATMega2560
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (Serial2.available() > 0)
    {
        dataReceived = (byte)Serial2.read();
        if(dataReceived == GPL_ESCAPE_CHAR)
        {
            xored = GPL_XOR_CHAR;
        } else
        {
            dataReceived ^= xored;
            xored = 0x00;

            switch(gplStatus)
            {
                case GPL_ST_IDLE:
                {
                    if(dataReceived == GPL_START_FRAME)
                    {   
                        i = 0;
                        g_InputBuffer[i++] = dataReceived;
                        checkSum = dataReceived;
                        gplStatus = GPL_ST_CMD;
                    }
                } break;
    
                case GPL_ST_CMD:
                {
                    g_InputBuffer[i++] = dataReceived;
                    checkSum += dataReceived;
                    gplStatus = GPL_ST_NUM_BYTE;
                } break;
    
                case GPL_ST_NUM_BYTE:
                {
                    numByte = dataReceived;
                    if(numByte > 0)
                    {
                        g_InputBuffer[i++] = dataReceived;
                        checkSum += dataReceived;
                        gplStatus = GPL_ST_DATA;
                    } else
                    {   
                        gplStatus = GPL_ST_IDLE;
                    }
                } break;
    
                case GPL_ST_DATA:
                {
                    g_InputBuffer[i++] = dataReceived;
                    checkSum += dataReceived;
                    if(--numByte == 0)
                        gplStatus = GPL_ST_CHECKSUM;
                } break;
    
                case GPL_ST_CHECKSUM:
                {
                    if(dataReceived == checkSum)
                    {   
                        gplStatus = GPL_ST_IDLE;
                        g_InputBuffer[i++] = dataReceived;

                        switch(g_InputBuffer[GPL_BYTE_CMD])
                        {
                            case CMD_GET_INPUTS:
                            {
                                Serial.print("Received Inputs: ");
                                g_Inputs = GPL_GetWord(g_InputBuffer);
                                Serial.println(g_Inputs);

                                g_aIn1 = ((g_Inputs & 0x0001) > 0) ? 1 : 0;
                                if(g_aIn1 != g_old_aIn1)
                                {
                                    g_old_aIn1 = g_aIn1;
                                    MqttPublishStatus_aIn1();
                                }

                                g_aIn2 = ((g_Inputs & 0x0002) > 0) ? 1 : 0;
                                if(g_aIn2 != g_old_aIn2)
                                {
                                    g_old_aIn2 = g_aIn2;
                                    MqttPublishStatus_aIn2();
                                }

                                g_aIn3 = ((g_Inputs & 0x0004) > 0) ? 1 : 0;
                                if(g_aIn3 != g_old_aIn3)
                                {
                                    g_old_aIn3 = g_aIn3;
                                    MqttPublishStatus_aIn3();
                                }

                                g_aIn4 = ((g_Inputs & 0x0008) > 0) ? 1 : 0;
                                if(g_aIn4 != g_old_aIn4)
                                {
                                    g_old_aIn4 = g_aIn4;
                                    MqttPublishStatus_aIn4();
                                }

                                g_aIn5 = ((g_Inputs & 0x0010) > 0) ? 1 : 0;
                                if(g_aIn5 != g_old_aIn5)
                                {
                                    g_old_aIn5 = g_aIn5;
                                    MqttPublishStatus_aIn5();
                                }

                                g_aIn6 = ((g_Inputs & 0x0020) > 0) ? 1 : 0;
                                if(g_aIn6 != g_old_aIn6)
                                {
                                    g_old_aIn6 = g_aIn6;
                                    MqttPublishStatus_aIn6();
                                }

                                g_aIn7 = ((g_Inputs & 0x0040) > 0) ? 1 : 0;
                                if(g_aIn7 != g_old_aIn7)
                                {
                                    g_old_aIn7 = g_aIn7;
                                    MqttPublishStatus_aIn7();
                                }

                                g_aIn8 = ((g_Inputs & 0x0080) > 0) ? 1 : 0;
                                if(g_aIn8 != g_old_aIn8)
                                {
                                    g_old_aIn8 = g_aIn8;
                                    MqttPublishStatus_aIn8();
                                }

                                g_dIn1 = ((g_Inputs & 0x0100) > 0) ? 1 : 0;
                                if(g_dIn1 != g_old_dIn1)
                                {
                                    g_old_dIn1 = g_dIn1;
                                    MqttPublishStatus_dIn1();
                                }

                                g_dIn2 = ((g_Inputs & 0x0200) > 0) ? 1 : 0;
                                if(g_dIn2 != g_old_dIn2)
                                {
                                    g_old_dIn2 = g_dIn2;
                                    MqttPublishStatus_dIn2();
                                }

                                g_dIn3 = ((g_Inputs & 0x0400) > 0) ? 1 : 0;
                                if(g_dIn3 != g_old_dIn3)
                                {
                                    g_old_dIn3 = g_dIn3;
                                    MqttPublishStatus_dIn3();
                                }

                                g_dIn4 = ((g_Inputs & 0x0800) > 0) ? 1 : 0;
                                if(g_dIn4 != g_old_dIn4)
                                {
                                    g_old_dIn4 = g_dIn4;
                                    MqttPublishStatus_dIn4();
                                }
                            } break;

                            case CMD_OUTPUTS:
                            {
                                Serial.print("Received Outputs: ");
                                g_Outputs = GPL_GetByte(g_InputBuffer);
                                Serial.println(g_Outputs);

                                g_Relay1 = ((g_Outputs & 0x01) > 0) ? 1 : 0;
                                if(g_Relay1 != g_oldRelay1)
                                {
                                    g_oldRelay1 = g_Relay1;
                                    MqttPublishStatus_Relay1();
                                }

                                g_Relay2 = ((g_Outputs & 0x02) > 0) ? 1 : 0;
                                if(g_Relay2 != g_oldRelay2)
                                {
                                    g_oldRelay2 = g_Relay2;
                                    MqttPublishStatus_Relay2();
                                }

                                g_Relay3 = ((g_Outputs & 0x04) > 0) ? 1 : 0;
                                if(g_Relay3 != g_oldRelay3)
                                {
                                    g_oldRelay3 = g_Relay3;
                                    MqttPublishStatus_Relay3();
                                }

                                g_Relay4 = ((g_Outputs & 0x08) > 0) ? 1 : 0;
                                if(g_Relay4 != g_oldRelay4)
                                {
                                    g_oldRelay4 = g_Relay4;
                                    MqttPublishStatus_Relay4();
                                }

                                g_RelayBell = ((g_Outputs & 0x10) > 0) ? 1 : 0;
                                if(g_RelayBell != g_oldRelayBell)
                                {
                                    g_oldRelayBell = g_RelayBell;
                                    MqttPublishStatus_RelayBell();
                                }

                                g_SwitchBus = ((g_Outputs & 0x40) > 0) ? 1 : 0;
                                if(g_SwitchBus != g_oldSwitchBus)
                                {
                                    g_oldSwitchBus = g_SwitchBus;
                                    MqttPublishStatus_SwitchBus();
                                }

                                g_SwitchTestBattery = ((g_Outputs & 0x20) > 0) ? 1 : 0;
                                if(g_SwitchTestBattery != g_oldSwitchTestBattery)
                                {
                                    g_oldSwitchTestBattery = g_SwitchTestBattery;
                                    MqttPublishStatus_SwitchTestBattery();
                                }
                            } break;

                            case CMD_EXT_POWER_STATUS:
                            {
                                Serial.print("Received Power Status: ");
                                if(GPL_GetByte(g_InputBuffer) == 1)
                                    g_PowerStatus = "ON";
                                else
                                    g_PowerStatus = "OFF";
                                Serial.println(g_PowerStatus);
                                MqttPublishStatus_Power();
                            } break;

                            case CMD_GET_V_BATT:
                            {
                                Serial.print("Received V Batt: ");
                                g_VoltageBattery = (double)(15 * GPL_GetWord(g_InputBuffer)) / 1024;
                                Serial.println(g_VoltageBattery);
                                MqttPublishStatus_Battery();
                            } break;
                        }
                    }
                    
                } break;
            }
        }
    }
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(g_ssid);

    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);

    Serial.print("Unique ID: ");
    Serial.println(g_UniqueId); 
    
    while(WiFi.status() != WL_CONNECTED && counter++ < 5) 
    {
        delay(500);
        Serial.print(".");
    }

    if(WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else
    {
        Serial.println("WiFi NOT connected!!!");
    }
}

void MqttReconnect() 
{
    // Loop until we're MqttReconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        if (g_mqttPubSub.connect(g_mqtt_DeviceName, g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");

            // ESP32 Subscribe following topics
            // Home assistant status
            g_mqttPubSub.subscribe("homeassistant/status");

            g_mqttPubSub.subscribe((g_TopicRelay1 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicRelay2 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicRelay3 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicRelay4 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicRelayBell + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicSwitchBus + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicSwitchTestBattery + "/set").c_str());
            
            delay(500);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 3 seconds");
            delay(3000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    byte state = 0;
    String payload;
    String topicMsg;
    
    for (int i = 0; i < length; i++) 
    {
        Serial.print((char)inFrame[i]);
        payload += (char)inFrame[i];
    }
    Serial.println();

    if(payload == "ON") 
        state = 1;
    else if(payload == "OFF") 
        state = 0;


    if(String(topic) == String("homeassistant/status")) 
    {
        if(payload == "online")
        {
            MqttHomeAssistantDiscovery();
            delay(1000);
            Serial.println("INIT BOARD...");
            GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_INIT_SYSTEM, 0);
        }
    }

    if(String(topic) == String(g_TopicRelay1 + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_RELAY2, state);
    }

    if(String(topic) == String(g_TopicRelay2 + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_RELAY3, state);
    }

    if(String(topic) == String(g_TopicRelay3 + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_RELAY4, state);
    }

    if(String(topic) == String(g_TopicRelay4 + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_RELAY5, state);
    }

    if(String(topic) == String(g_TopicRelayBell + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_AUX_POWER, state);
    }

    if(String(topic) == String(g_TopicSwitchBus + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_BUS_POWER, state);
    }
    
    if(String(topic) == String(g_TopicSwitchTestBattery + "/set")) 
    {
        GPL_SendByte(ID_ESP32, ID_MOTHER_BOARD, CMD_SET_TEST_BATTERY, state);
    }
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;

    if(g_mqttPubSub.connected())
    {
        Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");
        StaticJsonDocument<600> payload;
        JsonObject device;
        JsonArray identifiers;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain1/config";     // Discovery Topic for entity ain1

        payload["name"] = "IOTMB_Alarm.ain1";      
        payload["uniq_id"] = g_UniqueId + "_ain1";
        payload["stat_t"] = g_TopicState_ain1;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain2/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain2";      
        payload["uniq_id"] = g_UniqueId + "_ain2";
        payload["stat_t"] = g_TopicState_ain2;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 3
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain3/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain3";      
        payload["uniq_id"] = g_UniqueId + "_ain3";
        payload["stat_t"] = g_TopicState_ain3;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 4
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain4/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain4";      
        payload["uniq_id"] = g_UniqueId + "_ain4";
        payload["stat_t"] = g_TopicState_ain4;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 5
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain5/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain5";      
        payload["uniq_id"] = g_UniqueId + "_ain5";
        payload["stat_t"] = g_TopicState_ain5;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 6
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain6/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain6";      
        payload["uniq_id"] = g_UniqueId + "_ain6";
        payload["stat_t"] = g_TopicState_ain6;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 7
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain7/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain7";      
        payload["uniq_id"] = g_UniqueId + "_ain7";
        payload["stat_t"] = g_TopicState_ain7;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Input 8
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/ain8/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.ain8";      
        payload["uniq_id"] = g_UniqueId + "_ain8";
        payload["stat_t"] = g_TopicState_ain8;
        payload["dev_cla"] = "door";
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Digital Input 1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/din1/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.din1";      
        payload["uniq_id"] = g_UniqueId + "_din1";
        payload["stat_t"] = g_TopicState_din1;
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Digital Input 2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/din2/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.din2";      
        payload["uniq_id"] = g_UniqueId + "_din2";
        payload["stat_t"] = g_TopicState_din2;
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Digital Input 3
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/din3/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.din3";      
        payload["uniq_id"] = g_UniqueId + "_din3";
        payload["stat_t"] = g_TopicState_din3;
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Digital Input 4
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/binary_sensor/din4/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.din4";      
        payload["uniq_id"] = g_UniqueId + "_din4";
        payload["stat_t"] = g_TopicState_din4;
        device = payload.createNestedObject("device");
        device["name"] = g_mqtt_DeviceName;
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay Sirena
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/relayBell/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.Bell";                 
        payload["uniq_id"] = g_UniqueId + "_bell";                                                       
        payload["stat_t"] = g_TopicRelayBell + "/state";    
        payload["cmd_t"] = g_TopicRelayBell + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay OUT1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/relay1/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.relay1";                 
        payload["uniq_id"] = g_UniqueId + "_relay1";                                                          
        payload["stat_t"] = g_TopicRelay1 + "/state";    
        payload["cmd_t"] = g_TopicRelay1 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay OUT2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/relay2/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.relay2";                 
        payload["uniq_id"] = g_UniqueId + "_relay2";                                                            
        payload["stat_t"] = g_TopicRelay2 + "/state";    
        payload["cmd_t"] = g_TopicRelay2 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay OUT3
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/relay3/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.relay3";                 
        payload["uniq_id"] = g_UniqueId + "_relay3";                                                           
        payload["stat_t"] = g_TopicRelay3 + "/state";    
        payload["cmd_t"] = g_TopicRelay3 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay OUT4
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/relay4/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.relay4";                 
        payload["uniq_id"] = g_UniqueId + "_relay4";                                                           
        payload["stat_t"] = g_TopicRelay4 + "/state";    
        payload["cmd_t"] = g_TopicRelay4 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Switch Power Bus
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/switchBus/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.Bus";                 
        payload["uniq_id"] = g_UniqueId + "_switchBus";                                                          
        payload["stat_t"] = g_TopicSwitchBus + "/state";    
        payload["cmd_t"] = g_TopicSwitchBus + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Switch Test Battery
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/switchTestBatt/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.TestBattery";                 
        payload["uniq_id"] = g_UniqueId + "_switcheTestBatt";                                                          
        payload["stat_t"] = g_TopicSwitchTestBattery + "/state";    
        payload["cmd_t"] = g_TopicSwitchTestBattery + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Binary Power Status
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/binary_sensor/powerState/config";
        
        payload["name"] = "IOTMB_Alarm.powerstate";
        payload["uniq_id"] = g_UniqueId + "_binary";
        payload["stat_t"] = g_TopicPowerState;
        //payload["val_tpl"] = "{{ value_json.pwrStatus | is_defined }}";
        payload["dev_cla"] = "power";
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Sensor Battery Voltage
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/battVoltage/config";
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = "IOTMB_Alarm.voltBatt";                 
        payload["uniq_id"] = g_UniqueId + "_battVolt";                                                          
        payload["stat_t"] = g_TopicVoltBattery;
        payload["unit_of_meas"] = "V";
        //payload["val_tpl"] = "{{ value_json.battVolt | is_defined }}";
        device = payload.createNestedObject("device");
        device["name"] = "IOTMB_Alarm";
        device["model"] = "IOTMB_RevA";
        device["manufacturer"] = "VLC";
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);
    }
}

void MqttPublishStatus_aIn1()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn1 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain1.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn2()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn2 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain2.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn3()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn3 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain3.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn4()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn4 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain4.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn5()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn5 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain5.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn6()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn6 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain6.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn7()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn7 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain7.c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_aIn8()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_aIn8 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_ain8.c_str(), strPayload.c_str());
    }  
}

void MqttPublishStatus_dIn1()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_dIn1 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_din1.c_str(), strPayload.c_str());
    }   
}

void MqttPublishStatus_dIn2()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_dIn2 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_din2.c_str(), strPayload.c_str());
    }    
}

void MqttPublishStatus_dIn3()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_dIn3 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_din3.c_str(), strPayload.c_str());
    }    
}

void MqttPublishStatus_dIn4()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_dIn4 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish(g_TopicState_din4.c_str(), strPayload.c_str());
    }   
}

void MqttPublishStatus_Relay1()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_Relay1 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay1 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_Relay2()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_Relay2 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay2 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_Relay3()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_Relay3 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay3 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_Relay4()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_Relay4 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay4 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_RelayBell()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_RelayBell == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelayBell + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_SwitchBus()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_SwitchBus == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicSwitchBus + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_SwitchTestBattery()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_SwitchTestBattery == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicSwitchTestBattery + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_Power()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        g_mqttPubSub.publish(g_TopicPowerState.c_str(), g_PowerStatus.c_str());
    }
}

void MqttPublishStatus_Battery()
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        g_mqttPubSub.publish(g_TopicVoltBattery.c_str(), String(g_VoltageBattery).c_str());
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial Communication Protocol Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte CalculateChecksum(byte numByte)
{
    byte rv = 0, index;
    for(index = 0; index < numByte; index++)
    {
        rv += g_FrameBuffer[index];
    }
    return rv;
}

void GplSendData(byte* pBuff, int length)
{
    int i;
    byte dataToSend = 0;

    g_OutputBuffer[dataToSend++] = GPL_START_FRAME;

    for(i = 1; i < length; i++)
    {
        if(pBuff[i] == GPL_START_FRAME || pBuff[i] == GPL_ESCAPE_CHAR)
        {
            g_OutputBuffer[dataToSend++] = GPL_ESCAPE_CHAR;
            g_OutputBuffer[dataToSend++] = pBuff[i] ^ GPL_XOR_CHAR;
        } else
            g_OutputBuffer[dataToSend++] = pBuff[i];
    }
    
    Serial2.write(g_OutputBuffer, dataToSend);
}

/*************************************************************************************************************************************************/
//NAME:         GPL_SendByte
//DESCRPTION:   Send a byte
//              (byte) Command
//              (byte) Data
//RETURN:       void
//NOTE:
/*************************************************************************************************************************************************/
void GPL_SendByte(byte idSender, byte idReceiver, byte cmd, byte data)
{
    g_FrameBuffer[0] = GPL_START_FRAME;                         // Start Frame
    g_FrameBuffer[1] = cmd;                                     // Comando
    g_FrameBuffer[2] = 0x01;                                    // Lunghezza campo dati
    g_FrameBuffer[3] = data;                                    // Data
    g_FrameBuffer[4] = CalculateChecksum(6);                    // Checksum

    GplSendData(g_FrameBuffer, 5);
}

/*************************************************************************************************************************************************/
//NAME:         GPL_SendWord
//DESCRPTION:   Send a 2 byte word
//              (byte) Command
//              (word) Data
//RETURN:       void
//NOTE:
/*************************************************************************************************************************************************/
void GPL_SendWord(byte idSender, byte idReceiver, byte cmd, word data)
{
    g_FrameBuffer[0] = GPL_START_FRAME;                            // Start Frame
    g_FrameBuffer[1] = cmd;                                     // Comando
    g_FrameBuffer[2] = 0x02;                                    // Lunghezza campo dati
    g_FrameBuffer[3] = HIBYTE(data);                            // Data
    g_FrameBuffer[4] = LOBYTE(data);
    g_FrameBuffer[5] = CalculateChecksum(7);                    // Checksum

    GplSendData(g_FrameBuffer, 6);
}

/*************************************************************************************************************************************************/
//NAME:         GPL_SendDWord
//DESCRPTION:   Send a 4 byte word
//              (byte) Command
//              (unsigned long) Data
//RETURN:       void
//NOTE:
/*************************************************************************************************************************************************/
void GPL_SendDWord(byte idSender, byte idReceiver, byte cmd, unsigned long data)
{
    g_FrameBuffer[0] = GPL_START_FRAME;                            // Start Frame
    g_FrameBuffer[1] = cmd;                                     // Comando
    g_FrameBuffer[2] = 0x04;                                    // Lunghezza campo dati
    g_FrameBuffer[3] = MSBYTE(data);                            // Data
    g_FrameBuffer[4] = UPBYTE(data);
    g_FrameBuffer[5] = HIBYTE(data);
    g_FrameBuffer[6] = LOBYTE(data);
    g_FrameBuffer[7] = CalculateChecksum(9);                    // Checksum

    GplSendData(g_FrameBuffer, 8);
}

/*************************************************************************************************************************************************/
//NAME:         GPL_SendMessage
//DESCRPTION:   Send a buffer data
//              (byte) Command
//              (byte*) Buffer
//              (int) Buffer length
//RETURN:       void
//NOTE:
/*************************************************************************************************************************************************/
void GPL_SendMessage(byte idSender, byte idReceiver, byte cmd, byte* pBuff, int length)
{
    int i = 0;
    g_FrameBuffer[0] = GPL_START_FRAME;                         // Start Frame
    g_FrameBuffer[1] = cmd;                                     // Comando
    g_FrameBuffer[2] = length;                                  // Lunghezza campo dati
    for(i = 0; i < length; i++)
        g_FrameBuffer[i + 3] = pBuff[i];
    g_FrameBuffer[length + 3] = CalculateChecksum(length + 3);  // Checksum

    GplSendData(g_FrameBuffer, length + 4);
}

/*************************************************************************************************************************************************/
//NAME:         GPL_SendFrame
//DESCRPTION:   Send a Protocol Frame 
//INPUT:        (byte*) Pointer to Frame buffer
//RETURN:       void
//NOTE:
/*************************************************************************************************************************************************/
void GPL_SendFrame(byte* pBuff)
{
    int i = 0, length;
    length = pBuff[GPL_BYTE_LENGTH] + GPL_NUM_EXTRA_BYTES;
    for(i = 0; i < length; i++)
        g_FrameBuffer[i] = pBuff[i];

    GplSendData(g_FrameBuffer, length);
}

/*************************************************************************************************************************************************/
//NAME:         GPL0_GetByte
//DESCRPTION:   Get Frame Data Byte
//INPUT:        (byte*) Pointer to Frame buffer
//RETURN:       (INT8U) Byte in the frame
//NOTE:         
/*************************************************************************************************************************************************/
byte GPL_GetByte(byte* frame)
{
    byte rv = 0;
    if(frame)
    {
        rv = frame[GPL_BYTE_FIRST_DATA];
    }
    return rv;
}

/*************************************************************************************************************************************************/
//NAME:         GPL0_GetWord
//DESCRPTION:   Get Frame Data Word (2 Byte)
//INPUT:        (byte*) Pointer to Frame buffer
//RETURN:       (INT16U) Word in the frame
//NOTE:         
/*************************************************************************************************************************************************/
word GPL_GetWord(byte* frame)
{
    word rv = 0;
    if(frame)
    {
        rv = MAKEWORD(frame[GPL_BYTE_FIRST_DATA], frame[GPL_BYTE_FIRST_DATA + 1]);
    }
    return rv;
}

/*************************************************************************************************************************************************/
//NAME:         GPL0_GetDWord
//DESCRPTION:   Get Frame Data Unsigned Long (4 Byte)
//INPUT:        (byte*) Puntatore al buffer del frame
//RETURN:       (unsigned long) Unsigned Long in the frame
//NOTE:         
/*************************************************************************************************************************************************/
unsigned long GPL_GetDWord(byte* frame)
{
    unsigned long rv = 0;
    word hiWord = 0, loWord = 0;
    if(frame)
    {
        hiWord = MAKEWORD(frame[GPL_BYTE_FIRST_DATA], frame[GPL_BYTE_FIRST_DATA + 1]);
        loWord = MAKEWORD(frame[GPL_BYTE_FIRST_DATA + 2], frame[GPL_BYTE_FIRST_DATA + 3]);
        rv = (unsigned long)(((unsigned long)(loWord)) | (((unsigned long)(hiWord))<<16));
    }
    return rv;
}
