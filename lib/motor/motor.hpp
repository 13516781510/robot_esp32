#pragma once
#ifndef __MOTOR__
#define __MOTOR__
#define en_motor1 1
#define en_motor2 1
#define en_motor3 1
#define en_motor4 0
#define en_motor5 0
#define en_motor6 0
#define v5_0
#ifdef v4_2
class Motor{
public:
  unsigned char addr;
  int32_t location;
  uint16_t rela_palse ;
  uint16_t needpalse;
  bool dir;
  int16_t  speed;
  int acc;
  unsigned char search_location[3];
  Motor(unsigned char _addr){
    this->addr = _addr;
    this->rela_palse = 0;
    }
  void control_motor(bool & dir,int16_t & speed,int &acc,uint16_t &palse);
  //speed 0-4FF
  //acc 0-255 
  // 0.9  16细分，6400个脉冲完成一圈
  void get_location(void);

  void follow_object(int32_t goal_palse);

  void speed_control(bool &dir,float &speed,uint8_t &acc);
//prepared for the control of pipeline,robotic system are suggested to use location control

  void control_motor_pwm(float pipeline_speed);
};
#endif
#ifdef v5_0 //张大头V5.0

/*
 * 初始化时，带上电机的通信地址，可以进行当前位置清零，误差清零，脉冲清零，以及绝对位置、相对位置的控制 
*/
class Motor {
public:
    unsigned char addr;

    Motor(unsigned char _addr) {
        this->addr = _addr;
    }
    //speed 0-4FF
    //acc 0-255
    // 1.8  16细分，3200个脉冲完成一圈 4.6的转速比
    /*
    raf = 0 相对位置运动； raf = 1 绝对位置运动 snf多机同步标志
    */
    void Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, uint32_t angle, uint8_t acc, bool raF = true, bool snF = false);

    void Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, unsigned char angle1, unsigned char angle2, unsigned char angle3,
                            unsigned char angle4, uint8_t acc, bool raF, bool snF);

    void Emm_V5_Pos_Control(uint16_t vel, unsigned char dir_angle1[], uint8_t acc, bool raF, bool snF);

    void Emm_V5_EN(void);

    void Emm_V5_home(void);

    /*
     * 当前位置清零，误差清零，脉冲清零，以及绝
    */
    void Emm_V5_0set(void);
};

#endif
#endif