#include<Arduino.h>
#include "motor.hpp"
#include <../0_96_OLED/oled.hpp>

#define pi 3.1415926
#define d 10  // mm
#ifdef v4_2
unsigned char st1234[4] = {
0x00,0xf3,0x00,0x6b
};

void Motor::control_motor(bool & dir,int16_t & speed,int &acc,uint16_t &palse){
  unsigned char location_ctrl[9] = {
  this->addr,0xfd,((uint8_t)dir<<4)|((int8_t)(speed/256)),speed%256,acc,palse/65536,(palse/256)%256,palse%256,0x6b
  };
  Serial.write(location_ctrl,9);
}

void Motor::speed_control(bool &dir,float &speed,uint8_t & acc){
// pi*transfered_speed*d/(400*16) = speed; 
  uint16_t transfered_speed = (float)speed/pi/d*6400;
//命令格式：地址 + 0xF6 + 方向和速度（共用 2 个字节） + 加速度
  unsigned char speed_ctrl[6] = {
  this->addr,0xf6,((uint8_t)dir<<4)|((int8_t)(transfered_speed/256)),transfered_speed%256,acc,0x6b
  };

  //想来想去不能用这个指令来控制，因为这里是速度档位，档位怎么理解？对应的是一次都少个脉冲？需要标定，或者问问客服
  Serial.println("transfered_speed:");
  Serial.println(transfered_speed);
  // Serial.write(speed_ctrl,6);
}

void Motor::get_location(void){
  int location[6];
  int i=0;
  unsigned char search_location[3] = {this->addr,0x36,0x6b};
  Serial.write(search_location,3);
  while(Serial.available()>0){
    location[i] = Serial.read();
    i++;
    delay(10);   // 延时函数用于等待字符完全进入缓冲区，可以尝试没有延时，输出结果会是什么
  }
  if ((location[1] == this->addr)&&(location [5] ==0x6b))
  {
    this->location = (int32_t)(
      ((int32_t)location[1]<<24)|
      ((int32_t)location[2]<<16)|
      ((int32_t)location[3]<<8)|
      ((int32_t)location[4])
    );
  }
}

  void Motor::follow_object(int32_t goal_palse){
    this->get_location();
    this->needpalse = goal_palse-this->location;
    this->control_motor(this->dir,this->speed,this->acc,this->needpalse);
  }

  void Motor::control_motor_pwm(float pipeline_speed){
    // v  = 20*3.1415926*fre/(200*16) 
    uint32_t fre = (uint32_t)(pipeline_speed*502.654816);
    analogWriteFreq(fre);
    OLED_ShowNum(0,3,fre,5,8);
    OLED_ShowNum(0,4,int(pipeline_speed*100),5,8);//mm->0.01mm
    analogWriteRange(1000);
    analogWrite(12,500);
  }
#endif

#ifdef v5_0

/*
raf = 0 相对位置运动； raf = 1 绝对位置运动 snf多机同步标志
*/
void Motor::Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, uint32_t angle, uint8_t acc, bool raF, bool snF) {
    uint8_t cmd[16] = {0};
    uint32_t clk = uint32_t(angle * 654.22222222);//3200*4.6--360//256*200*4.6--360
    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0xFD;                      // 功能码
    cmd[2] = dir;                       // 方向
    cmd[3] = (uint8_t) (vel >> 8);       // 速度(RPM)高8位字节
    cmd[4] = (uint8_t) (vel >> 0);       // 速度(RPM)低8位字节
    cmd[5] = acc;                       // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t) (clk >> 24);      // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t) (clk >> 16);      // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t) (clk >> 8);       // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t) (clk >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                       // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                      // 校验字节
    // 发送命令
    Serial.write(cmd, 13);
}

void
Motor::Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, unsigned char angle1, unsigned char angle2, unsigned char angle3,
                          unsigned char angle4, uint8_t acc, bool raF, bool snF) {
    uint8_t cmd[16] = {0};
    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0xFD;                      // 功能码
    cmd[2] = dir;                       // 方向
    cmd[3] = (uint8_t) (vel >> 8);       // 速度(RPM)高8位字节
    cmd[4] = (uint8_t) (vel >> 0);       // 速度(RPM)低8位字节
    cmd[5] = acc;                       // 加速度，注意：0是直接启动
    cmd[6] = angle1;      // 脉冲数(bit24 - bit31)
    cmd[7] = angle2;      // 脉冲数(bit16 - bit23)
    cmd[8] = angle3;       // 脉冲数(bit8  - bit15)
    cmd[9] = angle4;       // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                       // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                      // 校验字节
    // 发送命令
    Serial.write(cmd, 13);
}

void Motor::Emm_V5_Pos_Control(uint16_t vel, unsigned char dir_angle1[], uint8_t acc, bool raF, bool snF) {
    uint8_t cmd[16] = {0};
    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0xFD;                      // 功能码
    cmd[2] = dir_angle1[0];                       // 方向
    cmd[3] = (uint8_t) (vel >> 8);       // 速度(RPM)高8位字节
    cmd[4] = (uint8_t) (vel >> 0);       // 速度(RPM)低8位字节
    cmd[5] = acc;                       // 加速度，注意：0是直接启动
    cmd[6] = dir_angle1[1];         // 脉冲数(bit24 - bit31)
    cmd[7] = dir_angle1[2];        // 脉冲数(bit16 - bit23)
    cmd[8] = dir_angle1[3];         // 脉冲数(bit8  - bit15)
    cmd[9] = dir_angle1[4];         // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                       // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                      // 校验字节
    // 发送命令
    Serial.write(cmd, 13);
}

void Motor::Emm_V5_EN(void) {
    uint8_t cmd[6] = {0};

    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0xF3;                      // 功能码
    cmd[2] = 0xAB;                       // 方向
    cmd[3] = 0;      // 速度(RPM)高8位字节
    cmd[4] = 0;       // 速度(RPM)低8位字节
    cmd[5] = 0x6B;                       // 加速度，注意：0是直接启动
    // 发送命令
    Serial.write(cmd, 6);
}

/*
 * 当前位置清零，误差清零，脉冲清零
*/
void Motor::Emm_V5_0set(void) {
    uint8_t cmd[6] = {0};

    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0x0A;                      // 功能码
    cmd[2] = 0x6D;                       // 方向
    cmd[3] = 0x6B;                       // 加速度，注意：0是直接启动
    // 发送命令
    Serial.write(cmd, 4);
}

void Motor::Emm_V5_home(void) {
    uint8_t cmd[6] = {0};

    // 装载命令
    cmd[0] = this->addr;                      // 地址
    cmd[1] = 0x9A;                      // 功能码
    cmd[2] = 0x02;                       //
    cmd[3] = 0x00;                       // 加速度，注意：0是直接启动
    cmd[4] = 0x6B;                       // 加速度，注意：0是直接启动
    // 发送命令
    Serial.write(cmd, 5);
}

#endif