#include "robotgeo.hpp"
#include <cmath>
 unsigned char robotgeo::robot_inversegeo(float xmm, float ymm, float zmm) {
    float rrot = std::sqrt((xmm * xmm) + (ymm * ymm));

    if ((xmm == 0) and (ymm == 0)) {
        this->theta1 = 0;
    } else { this->theta1 = std::asin(xmm / rrot); }
    xmm = xmm - 41 * (xmm / rrot);
    ymm = ymm - 41 * (ymm / rrot);
    rrot = std::sqrt((xmm * xmm) + (ymm * ymm));
    float rside = std::sqrt(
            (rrot * rrot) + (zmm * zmm));
    this->theta3 = std::acos((rside * 0.5) / 120.0) * 2.0;  //120mm shank length

    if (zmm > 0) {
        this->theta2 = (std::asin(rrot / rside) + ((M_PI - this->theta3) / 2.0) - (M_PI / 2.0));
    } else {
        this->theta2 = (M_PI - std::asin(rrot / rside) + ((M_PI - this->theta3) / 2.0) - (M_PI / 2.0));
    }
    this->theta3 = this->theta3 + this->theta2;
    this->theta3 = this->theta3 * 180 / M_PI - 90;
    this->theta2 = this->theta2 * 180 / M_PI;
    this->theta1 = -(this->theta1 * 180 / M_PI - 90);
    if( (std::isnan(this->theta1))  or (std::isnan(this->theta2)) or (std::isnan(this->theta3))){
        return 0;
    }
    else{
        return 1;
    }
}

void robotgeo::robot_angle_clk(void) {
    if (this->theta1 >= 0) {
        this->clk[0][0] = 0;
    } else {
        this->clk[0][0] = 1;
    }
    if (this->theta2 >= 0) {
        this->clk[1][0] = 0;
    } else {
        this->clk[1][0] = 1;
    }
    if (this->theta3 >= 0) {
        this->clk[2][0] = 0;
    } else {
        this->clk[2][0] = 1;
    }
    this->clk[0][1] = (unsigned char) ((int(this->theta1 * 654.222222222)) >> 24);
    this->clk[0][2] = (unsigned char) ((int(this->theta1 * 654.222222222)) >> 16);
    this->clk[0][3] = (unsigned char) ((int(this->theta1 * 654.222222222)) >> 8);
    this->clk[0][4] = (unsigned char) ((int(this->theta1 * 654.222222222)) >> 0);
    this->clk[1][1] = (unsigned char) (abs((int(this->theta2 * 654.222222222))) >> 24);
    this->clk[1][2] = (unsigned char) (abs((int(this->theta2 * 654.222222222)))>> 16);
    this->clk[1][3] = (unsigned char) (abs((int(this->theta2 * 654.222222222))) >> 8);
    this->clk[1][4] = (unsigned char) (abs((int(this->theta2 * 654.222222222))) >> 0);
    this->clk[2][1] = (unsigned char) (abs((int(this->theta3 * 654.222222222))) >> 24);
    this->clk[2][2] = (unsigned char) (abs((int(this->theta3 * 654.222222222))) >> 16);
    this->clk[2][3] = (unsigned char) (abs((int(this->theta3 * 654.222222222))) >> 8);
    this->clk[2][4] = (unsigned char) (abs((int(this->theta3 * 654.222222222))) >> 0);
}

void robotgeo::robot_forwardgeo(float _theta1, float _theta2, float _theta3) {
    this->z = 120 * std::cos(_theta2) + 120 * std::cos(_theta3);
    this->x = std::sin(_theta1) * (120 * std::sin(_theta2) + 120 * std::sin(_theta3));
    this->y = std::cos(_theta1) * (120 * std::sin(_theta2) + 120 * std::sin(_theta3));
}