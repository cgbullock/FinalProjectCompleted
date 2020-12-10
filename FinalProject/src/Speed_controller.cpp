#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Arduino.h"
#include "Sonar_sensor.h"

Romi32U4Motors motors;
Encoder MagneticEncoder;
SonarSensor HCSR04; 

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    HCSR04.Init();
}

void SpeedController::Process(float target_velocity_left, float target_velocity_right)
{
    
    if(MagneticEncoder.UpdateEncoderCounts()){

        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        
        //Serial.print(MagneticEncoder.ReadVelocityLeft());
        //Serial.print('\t');
        //Serial.println(MagneticEncoder.ReadVelocityRight());
    }
}

void SpeedController::Straight(int target_velocity, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();
  

    while ((unsigned long)(millis() - now) <= time*1000){
        Process(target_velocity,target_velocity);
    }
    Process(0, 0);
}


   boolean SpeedController::Turn(int degree, int direction)
{
    
    MagneticEncoder.UpdateEncoderCounts();
    int turns = counts*(degree/180.0); //assignment 1: convert degree into counts
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();
 
    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        
        MagneticEncoder.UpdateEncoderCounts();
        if(!direction) motors.setEfforts(50,-50);
        else motors.setEfforts(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}


void SpeedController::WallFollow(int speed, int dist){
    //Serial.println(HCSR04.ReadData());
     E_distance = dist - HCSR04.ReadData();
  time = millis();
  dE = E_distance - prev_e_distance;
  dT = time - prev_time;
 
 
  
  float speed2 = (Kpw * E_distance) + (Kdw * (dE/dT));
  
  prev_e_distance = E_distance;
  prev_time = time;
  motors.setEfforts(speed - speed2, speed + speed2);
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
}

void SpeedController::Run(int distance)//in cm
{
    float time = millis();
while((millis() - time) < (distance * 150)){
  motors.setEfforts(50,50);
}
motors.setEfforts(0,0);
}
