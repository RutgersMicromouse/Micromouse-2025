#include "pidStraight.h"
#define DIA 32
#define PI 3.1415926535897932384626433832795


double minPWM = 100;
double maxPWM = 150;

void straight(char direction){
    double currentAngle = angle();
    double targetDirection = 0;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double kp = 1;
    double previousTime = millis();
    double totalTime = millis();
    double previousAngle = 0;
    double derivative = angle();
    double kd = 0; 

   //( 360 ticks / 1 rev ) * (1 rev / 32pi mm) * (160 mm)

    double distance = 10;
    int numTicks = (360 * distance)/(DIA * PI);

    switch(direction) {
        case 'N':
            targetDirection = 0;
            break;
        case 'S':
            targetDirection = 180;
            break;
        case 'E':
            targetDirection = 90;
            break;
        case 'W':
            targetDirection = 270;
            break;
    }
    // encLeft.write(0);
    // encRight.write(0);

    int startleft = encLeft.read();
    int startright = encRight.read();
    
    while(abs(startleft) < 50){
        Serial.printf("%d < %d || %d < %d\n",startleft,numTicks,abs(startright),numTicks);
        currentAngle = angle();
        double error = targetDirection - currentAngle;

        totalTime = millis();

        if (error > 180){
            error -= 360;
        }
        
        if (error <= -180){
            error += 360;
        }

        derivative = (currentAngle - previousAngle)/(totalTime - previousTime);

        leftMotorSpeed = 125 + kp*error - kd*derivative;
        rightMotorSpeed = 125 - kp*error + kd*derivative;
        rightMotorSpeed *= 1.2;

        if (leftMotorSpeed < minPWM){
            leftMotorSpeed = minPWM;
        }
        else if (leftMotorSpeed > maxPWM){
            leftMotorSpeed = maxPWM;
        }

        if (rightMotorSpeed < minPWM){
            rightMotorSpeed = minPWM;
        }
        else if (rightMotorSpeed > maxPWM){
            rightMotorSpeed = maxPWM;
        }
        
      //  Serial.printf("%lf\t%lf\t",error,leftMotorSpeed);
       // Serial.printf("%lf\n",rightMotorSpeed);
        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);
       // Serial.printf("%d\n",getLeftEncoder());
        previousTime = totalTime;
        previousAngle = currentAngle;
        startleft = encLeft.read();
        startright = encRight.read();

        // Serial.printf("Left Motor Speed: %lf\t", leftMotorSpeed);
        // Serial.printf("Right Motor Speed: %lf\n", rightMotorSpeed);

    }

    moveLeftMotor(0);
    moveRightMotor(0);
//    while(1)
//    Serial.println("BRUH");

}

