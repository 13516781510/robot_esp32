#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <../lib/0_96_OLED/oled.hpp>
# include <../lib/motor/motor.hpp>
#include <../lib/robotgeo/robotgeo.hpp>
#include <WiFi.h>
// #include <../lib/0_96_OLED/oledfont.hpp>
// #define angle
#define bufferSize 13  // 3个代表直接接受角度，15个接收3*[1+4]，方向➕脉冲
const byte frameStart = 0xFF;//数据包帧头
const byte frameEnd = 0xFE;//数据包帧尾
#define frameStartsize 2  //针头的个数，在数据处理回调函数中，默认了帧尾的个数等于帧头个数
unsigned char num_label[4];
unsigned char receivedBuffer2[12];//
struct Location {
    /* data */
    float x;
    float y;
} location;
int num_yunda, num_shunfeng, num_yuantong;
int labeled = 0;

bool motor_is_reached[260] = {0};
#if (en_motor1)
Motor *motor1 = new Motor(0x01);
#endif
#if (en_motor2)
Motor *motor2 = new Motor(0x02);
#endif
#if (en_motor3)
Motor *motor3 = new Motor(0x03);
#endif
// put function declarations here:
/*mqtt回调函数*/
void onMqttMessage(int messageSize);

/*串口0回调函数*/
void Callback1();

/*串口2回调函数*/
void Callback2();

String Payload = "{\"params\":{\"robot_power\":1},\"veision\":\"1.0.0\"}";
bool willRetain = true;
int willQos = 1;
int subscribeQos = 1;
/*huojia::到六个货架的，每个电机需要给的脉冲数量，256细分，1.8°步长电机，传动比4.6带轮结构的前提下，由robotgeo计算得到；*/
robotgeo Robot;
bool trace = 0;
unsigned char K = 0;
static float pipeline_speed = 0;
String ssid = "88888888";    // your network SSID (name)
String pass = "88888888";    // your network password (use for WPA, or use as key for WEP)
// String ssid = "Xiaomi_F30C";    // your network SSID (name)
// String pass = "2945194465";    // your network password (use for WPA, or use as key for WEP)
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
unsigned char res;
float x, y, z;
bool needchange;
const char broker[] = "iot-06z00jl8tcjx5tf.mqtt.iothub.aliyuncs.com";
int port = 1883;
const char willTopic[] = "/sys/k0eudLzkYR8/esp32/thing/event/property/post";
const char inTopic[] = "/sys/k0eudLzkYR8/esp32/thing/service/property/set";
const char outTopic[] = "/sys/k0eudLzkYR8/esp32/thing/event/property/post";
const char replytopic[] = "/sys/k0eudLzkYR8/esp32/thing/event/property/post_reply";
String payload;
const char clientId[] = "k0eudLzkYR8.esp32|securemode=2,signmethod=hmacsha256,timestamp=1710758471671|";
const char username[] = "esp32&k0eudLzkYR8";
const char password[] = "162971c18505085332ba9267ffe99da9c62457151c4555396edc76f4a0e708d6";
String inputString = "";
bool fangzhifinish = 1;
int count = 0;
JsonDocument json_msg;
JsonDocument json_data;
unsigned char label;
extern unsigned char st[4];

void clear_reached(void) {
    motor_is_reached[1] = 0;
    motor_is_reached[2] = 0;
    motor_is_reached[3] = 0;
}

/**
 * not_reached 返回1，不达到返回0
*/
int check_not_reached(void) {
    if ((motor_is_reached[1] == 0) or (motor_is_reached[2] == 0) or (motor_is_reached[3] == 0)) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * reached 返回1，不达到返回0
*/
int check_reached(void) {
    if ((motor_is_reached[1] == 1) and (motor_is_reached[2] == 1) and (motor_is_reached[3] == 1)) {
        return 1;
    } else {
        return 0;
    }
}

void uCharsToFloat(unsigned char *receivedBuffer2, float &x, float &y, float &z) {
    int x1;
    x1 = float((receivedBuffer2[3] + receivedBuffer2[2] * 256 + receivedBuffer2[1] * 256 * 256)) / 20000;
    if (receivedBuffer2[0] == 0) {
        x1 = -1 * (x1);
    }
    x = x1-30;
    labeled = label;
    y = float((receivedBuffer2[7] + receivedBuffer2[6] * 256 + receivedBuffer2[5] * 256 * 256)) / 20000;
    z = float((receivedBuffer2[11] + receivedBuffer2[10] * 256 + receivedBuffer2[9] * 256 * 256)) / 20000;
    if (receivedBuffer2[4] == 0) {
        y = -1 * (y);
    }
    y = y+133;
    if (receivedBuffer2[8] == 0) {
        z = -1 * (z);
    }
    z = z+20;
    
}

void onMqttMessage(int messageSize) {
    while (mqttClient.available()) {
        char inchar = (char) mqttClient.read();
        inputString = inputString + inchar;

        if (inputString.length() == messageSize) {

            JsonDocument json_msg;
            JsonDocument json_item;
            JsonDocument json_location;
            JsonDocument json_pipeline_speed;
            deserializeJson(json_msg, inputString);
            String items = json_msg["items"];
            deserializeJson(json_item, items);

            String location = json_item["location"];
            String spipeline_speed = json_item["pipeline_speed"];

            if (location != "null") {
                deserializeJson(json_location, location);
                int x = json_location["value"][0];
                int y = json_location["value"][1];
                int z = json_location["value"][2];
            }


            if (spipeline_speed != "null") {
                deserializeJson(json_pipeline_speed, spipeline_speed);
                float a = json_pipeline_speed["value"];
                if (a != pipeline_speed) {
                    pipeline_speed = a;
                    needchange = true;
                }
            }
            inputString = "";
        }
    }
}


void Callback1() {
    static byte receivedIndex1 = 0;
    while (Serial.available()) {
        byte incomingByte = Serial.read();
        // recieve:  命令返回：地址 + 0xFD + 命令状态 + 校验字节
        // 命令示例：发送 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B，正确返回 01 FD 02 6B，
        // 条件不满足返回 01 FD E2 6B，错误命令返回 01 00 EE 6B
        //Reached： 只在发送位置模式命令时返回到位命令（地址 + FD + 9F + 6B）；
        if (receivedIndex1 == 0) {
            motor_is_reached[incomingByte] = 1;
        }
        receivedIndex1++;
    }
    receivedIndex1 = 0;
}

void Callback2() {
    static byte receivedIndex2 = 0;
    unsigned char j = 0;
    while (Serial2.available()) {
        byte incomingByte = Serial2.read();
        if (incomingByte == frameStart && receivedIndex2 < frameStartsize) {
            //帧头两个0,1----1,2；
            receivedIndex2++;
            //
        } else if (receivedIndex2 >= frameStartsize && receivedIndex2 < (bufferSize + frameStartsize)) {

            //数据,2 3 4 5 6 7 8 9 10 11 12 13 14----3 4 5 6 7 8 9 10 11 12 13 14 15
            if (receivedIndex2 == 2) {
                label = incomingByte;
            } else {
                //0 1 2 3 4 5 6 7 8 9 10 11 
                receivedBuffer2[j] = incomingByte;
                j++;
            }
            receivedIndex2++;
        } else if (incomingByte == frameEnd && receivedIndex2 >= bufferSize + frameStartsize) {
            //15 16 进并且在这里达到17 另  16，0
            receivedIndex2++;
            receivedIndex2 = 0;
            trace = 1;
            j = 0;

        } else {
            receivedIndex2 = 0;
            j = 0;
        }
    }
}

void setup() {
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    Serial.begin(115200);
    Serial.onReceive(Callback1);
    Serial2.begin(115200);
    Serial2.onReceive(Callback2);
    /* 串口初始化后直接开始电机的多圈无限位回零，在运动过程中可以进行wifi和mqtt的连接，节约时间 */
    motor1->Emm_V5_home();
    delay(15);
    motor2->Emm_V5_home();
    delay(15);
    motor3->Emm_V5_home();
    // OLED_init();
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    // OLED_init1();
    mqttClient.setId(clientId);
    mqttClient.setUsernamePassword(username, password);
    String willPayload = "{\"params\":{\"robot_power\":0,\"num_yuantong\":0,\"num_shunfeng\":0,\"num_yunda\":0},\"veision\":\"1.0.0\"}";
    bool willRetain = true;
    int willQos = 1;
    int subscribeQos = 1;
    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();
    if (!mqttClient.connect(broker, port)) {
        while (1);
    }
    mqttClient.onMessage(onMqttMessage);
    mqttClient.subscribe(inTopic, subscribeQos);
    mqttClient.unsubscribe(replytopic);
    Payload = "{\"params\":{\"robot_power\":0,\"num_yuantong\":0,\"num_shunfeng\":0,\"num_yunda\":0},\"veision\":\"1.0.0\"}";

    mqttClient.beginMessage(outTopic, Payload.length(), willRetain, willQos);
    mqttClient.print(Payload);
    mqttClient.endMessage();
/*
* 连上wifi和mqtt服务器后，初始化电机到建模预设的初始状态，等待电机全部到位，进行位置清零和误差清零
*/
    clear_reached();
    motor1->Emm_V5_Pos_Control(0, 40, 10, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    delay(15);
    motor2->Emm_V5_Pos_Control(0, 40, 50, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    delay(100);
    motor3->Emm_V5_Pos_Control(0, 40, 25, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    delay(15);
    while (check_not_reached()) {
        delay(10);
    }

    clear_reached();
    while ((motor_is_reached[1] == 0)) {
        motor1->Emm_V5_0set();
        delay(15);
    }
    while ((motor_is_reached[2] == 0)) {
        delay(15);
        motor2->Emm_V5_0set();
    }
    while ((motor_is_reached[3] == 0)) {
        delay(15);
        motor3->Emm_V5_0set();
    }

    clear_reached();
    motor1->Emm_V5_Pos_Control(0, 40, 90, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    delay(15);
    motor1->Emm_V5_Pos_Control(0, 40, 90, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    delay(15);
    motor1->Emm_V5_Pos_Control(0, 40, 90, 25, 1, 0);//200 * 16 = 3200 = 1圈，800 = 1/4圈 。传动比1:4.6
    while ((motor_is_reached[1] == 0)) {
        delay(15);
    }
    clear_reached();
    delay(20);
}


void loop() {
    if (trace == 1) {
            uCharsToFloat(receivedBuffer2, x, y, z);
            res = Robot.robot_inversegeo(x+7, y, z);
            if (res == 1) {
                Robot.robot_angle_clk();
                motor1->Emm_V5_Pos_Control(70, Robot.clk[0], 0, 1, 0);
                delay(12);
                motor2->Emm_V5_Pos_Control(70, Robot.clk[1], 0, 1, 0);
                delay(12);
                motor3->Emm_V5_Pos_Control(70, Robot.clk[2], 0, 1, 0);
                delay(30);
                if (check_reached()) {
                    // delay(1200);
                    clear_reached();
                    digitalWrite(21, HIGH);
                    res = Robot.robot_inversegeo(x+13, y, z - 6);
                    Robot.robot_angle_clk();
                    motor1->Emm_V5_Pos_Control(50, Robot.clk[0], 0, 1, 0);
                    delay(12);
                    motor2->Emm_V5_Pos_Control(50, Robot.clk[1], 0, 1, 0);
                    delay(12);
                    motor3->Emm_V5_Pos_Control(50, Robot.clk[2], 0, 1, 0);
                    fangzhifinish = 0;
                    delay(1500);
                    clear_reached();
                }
            }
        
        while (fangzhifinish == 0) {
            if (labeled == 1) {
                location.x = 152;
                location.y = 4.5;
                num_yunda = num_yunda + 1;
                Payload = "{\"params\":{\"num_yunda\":" + String(num_yunda) + "},\"veision\":\"1.0.0\"}";
            }
            if (labeled == 2) {
                location.x = 195;
                location.y = 4.5;
                num_shunfeng = num_shunfeng + 1;
                Payload = "{\"params\":{\"num_shunfeng\":" + String(num_shunfeng) + "},\"veision\":\"1.0.0\"}";
            }
            if (labeled == 3) {
                location.x = 195;
                location.y = 47.5;
                num_yuantong = num_yuantong + 1;
                Payload = "{\"params\":{\"num_yuantong\":" + String(num_yuantong) + "},\"veision\":\"1.0.0\"}";
            }
            //上方
            Robot.robot_inversegeo(location.x, location.y, z + 20);
            Robot.robot_angle_clk();
            clear_reached();
            motor1->Emm_V5_Pos_Control(50, Robot.clk[0], 0, 1, 0);
            delay(12);
            motor2->Emm_V5_Pos_Control(50, Robot.clk[1], 0, 1, 0);
            delay(12);
            motor3->Emm_V5_Pos_Control(50, Robot.clk[2], 0, 1, 0);
            while (check_not_reached()) {
                delay(10);
            }
            clear_reached();

            Robot.robot_inversegeo(location.x, location.y, 0);
            Robot.robot_angle_clk();
            motor1->Emm_V5_Pos_Control(50, Robot.clk[0], 0, 1, 0);
            delay(12);
            motor2->Emm_V5_Pos_Control(50, Robot.clk[1], 0, 1, 0);
            delay(12);
            motor3->Emm_V5_Pos_Control(50, Robot.clk[2], 0, 1, 0);
            while (check_not_reached()) {
                delay(10);
            }
            clear_reached();

            Robot.robot_inversegeo(location.x, location.y, -75);
            Robot.robot_angle_clk();
            delay(1000);
            delay(15);
            motor1->Emm_V5_Pos_Control(50, Robot.clk[0], 0, 1, 0);
            delay(12);
            motor2->Emm_V5_Pos_Control(50, Robot.clk[1], 0, 1, 0);
            delay(12);
            motor3->Emm_V5_Pos_Control(50, Robot.clk[2], 0, 1, 0);
            while (check_not_reached()) {
                delay(10);
            }
            digitalWrite(21, LOW);
            delay(2000);
            //上方
            Robot.robot_inversegeo(location.x, location.y, 0);
            Robot.robot_angle_clk();
            motor1->Emm_V5_Pos_Control(50, Robot.clk[0], 0, 1, 0);
            delay(12);
            motor2->Emm_V5_Pos_Control(50, Robot.clk[1], 0, 1, 0);
            delay(12);
            motor3->Emm_V5_Pos_Control(50, Robot.clk[2], 0, 1, 0);
            while (check_not_reached()) {
                delay(10);
            }
            delay(1500);
            //重置
            clear_reached();
            motor1->Emm_V5_Pos_Control(0, 100, 90, 25, 1, 0);
            delay(15);
            motor2->Emm_V5_Pos_Control(0, 100, 0, 25, 1, 0);
            delay(15);
            motor3->Emm_V5_Pos_Control(0, 100, 0, 25, 1, 0);
            delay(15);
            while (check_not_reached()) {
                delay(10);
            }
            clear_reached();


            willRetain = true;
            willQos = 1;
            subscribeQos = 1;

            mqttClient.beginMessage(outTopic, Payload.length(), willRetain, willQos);
            mqttClient.print(Payload);
            mqttClient.endMessage();
            fangzhifinish = 1;
            trace = 0;
            x = 1000;
        }
     }
    mqttClient.poll();
}
