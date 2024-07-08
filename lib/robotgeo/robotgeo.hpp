//
// Created by 29451 on 2023/12/22.
//

#ifndef ROBOTGEO_ROBOTGEO_HPP
#define ROBOTGEO_ROBOTGEO_HPP

class guanjie {
public:
    int d = 0;
    int h = 0;
    int a = 0;
    int alpha = 0;

    guanjie(int d, int h, int a, int alpha) {
        this->d = d;
        this->h = h;
        this->a = a;
        this->alpha = alpha;
    }

    guanjie() {
        this->d = 0;
        this->h = 0;
        this->a = 0;
        this->alpha = 0;
    }
};

class robotgeo {

public:
    guanjie *guanjie1;
    guanjie *guanjie2;
    guanjie *guanjie3;
    float x;
    float y;
    float z;
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    unsigned char clk[3][5];

    robotgeo(guanjie *guanjie1, guanjie *guanjie2, guanjie *guanjie3) {
        this->guanjie1 = guanjie1;
        this->guanjie2 = guanjie2;
        this->guanjie3 = guanjie3;
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->theta1 = 0;
        this->theta2 = 0;
        this->theta3 = 0;
        this->theta4 = 0;
    }

    robotgeo() {
        this->guanjie1 = new guanjie;
        this->guanjie2 = new guanjie;
        this->guanjie3 = new guanjie;
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->theta1 = 0;
        this->theta2 = 0;
        this->theta3 = 0;
        this->theta4 = 0;
    }

    void robot_forwardgeo(float _theta1, float _theta2, float _theta3);
    void robot_angle_clk();
    unsigned char robot_inversegeo(float xmm, float ymm, float zmm);

};


#endif //ROBOTGEO_ROBOTGEO_HPP

