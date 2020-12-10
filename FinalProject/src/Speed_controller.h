#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1; 
        float E_left = 0; 
        float E_right = 0;
        float BigR = 519.75;
        float BigE = 1440;
        float counts = 1450;
        float littleR = 220.5;
         const float Kpw = 7;
        const float Kdw = 2; 
        float E_distance = 0;
        float prev_time = 0;
        float prev_e_distance = 0;
        float time = 0;
        float dE = 0;
        float dT = 0;

    public:
        void Init(void);
        void Process(float, float);
        boolean Turn(int, int);
        void Stop(void);
        void WallFollow(int,int);
        void Straight(int, int);
        void Run(int);
};

#endif