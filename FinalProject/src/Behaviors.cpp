#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"
#include "Sonar_sensor.h"
#include "Encoders.h"


//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;


//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//motor-speed controller
SpeedController PIcontroller;


void Behaviors::Init(void)
{

    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    //assignment 2
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if(abs(data[2]) > threshold_pick_up) return 1;
    else return 0;
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
        case IDLE:
        Serial.println(robot_state);
        if(buttonA.getSingleDebouncedRelease()){
            delay(1000);
            robot_state = DRIVE;
        }
        else{
            PIcontroller.Stop();
            robot_state = IDLE;
        }
        break;

        case DRIVE:
        Serial.println(robot_state);
        if(DetectCollision()){
            PIcontroller.Stop();
            robot_state = OBJECT;
        }
        else{
            PIcontroller.Process(100, 100);
            robot_state = DRIVE;
        }
        break;

        case OBJECT:
        Serial.println(robot_state);
     
        if(buttonA.getSingleDebouncedRelease()){
            PIcontroller.Turn(90, 1);
            robot_state = SONAR1;
        }
        else{
            PIcontroller.Stop();
            robot_state = OBJECT;
        }
        break;

        case SONAR1:
        Serial.println(robot_state);
        
        if(DetectCollision()){
            Serial.println("turning");
            if(PIcontroller.Turn(90, 1)){
            flag = true;
            }
            
        }
        else if(flag){
            
            robot_state = SONAR2;
        }
        else{
            time_now = millis();
            dt = (time_now - prev_time);
            if(dt >= 100){
            PIcontroller.WallFollow(100 , 10);
            prev_time = time_now;
            }
            else{
            robot_state = SONAR1;
            }
        }
        break;

        case SONAR2:
        Serial.println(robot_state);
        if(DetectBeingPickedUp()){
            //while(true){
            //PIcontroller.Straight(50, 10);
           
            robot_state = FINALSTRETCH;
            //break;
            //}
            
        }
        else{
            PIcontroller.WallFollow(100 , 10);
            robot_state = SONAR2;
        }
        break;

        case FINALSTRETCH:
        Serial.println(robot_state);
        PIcontroller.Run(10);
        robot_state = IDLE;
        break;

    }

    }
    
