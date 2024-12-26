/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "ai_functions.h"
#include <string>
#include <iostream>
using namespace vex;
using namespace std;

//FILE *fp = fopen("/dev/serial2","wb");
// Calculates the distance to the coordinates from the current robot position
double distanceTo(double target_x, double target_y, vex::distanceUnits units = vex::distanceUnits::in)
{
    double distance = sqrt(pow((target_x - GPS.xPosition(units)), 2) + pow((target_y - GPS.yPosition(units)), 2));
    return distance;
}


// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY)
{
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}



void moveToPoint(double target_x, double target_y, double target_theta = 1000){   
    float ThresholdRad = 12; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrived2Target = false;
  
    while(!arrived2Target)
    {

        double X_Pos = GPS.xPosition(vex::distanceUnits::cm);
        double Y_Pos = GPS.yPosition(vex::distanceUnits::cm);
        // Check to see if we have arrived to target 
        double threshold = pow((X_Pos - target_x), 2) + pow((Y_Pos - target_y),2);
        if(threshold <= pow(ThresholdRad, 2))
        {   
                fprintf(fp,"\rRobot is within the threshold of target\n");
                break;
        }
        // Turn Function
        double targetheading = calculateBearing(X_Pos, Y_Pos, target_x, target_y);
        double diff = fabs(GPS.heading(vex::rotationUnits::deg) - targetheading);
        double result = (diff <= 180.0) ? diff : 360.0 - diff;

        if((result > 90))
        {
            targetheading +=  180;
        }
        chassis.set_heading(GPS.heading(deg));
        chassis.turn_to_angle(targetheading);
        //Drive Function
        // chassis.desired_heading = targetheading;
        float distance = distanceTo(target_x, target_y);
        if((result > 90))
        {
            distance = distance * -1;
        }
        chassis.drive_distance(distance/2.54);
        wait(20,msec);
        if(target_theta <= 360)
        {
            chassis.turn_to_angle(target_theta);
        }
   }

}






DETECTION_OBJECT findTarget(int type)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;

    if(local_map.detectionCount > 0)
    {
    fprintf(fp,"\r\nNumber of objects in local map: %ld ",local_map.detectionCount);
        for (int i = 0; i < local_map.detectionCount; i++)
        {
            if(local_map.detections[i].classID == type)
            {
                if(local_map.detections[i].probability > 0.70 && local_map.detections[i].probability <= 1) 
                {
                    double Obj_Dist = distanceTo(local_map.detections[i].mapLocation.x*39.37, local_map.detections[i].mapLocation.y*39.37,inches);
                    if (Obj_Dist < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = Obj_Dist; 
                    }
                }
            } 
        }
    }
    else
    {
        if(target.mapLocation.x < -3 || target.mapLocation.x > 3 )
        {
            //target.mapLocation.x = 0.00;
            //target.classID = 99;
        }
        if(target.mapLocation.y < -3 || target.mapLocation.y > 3 )
        {
            //target.mapLocation.y = 0.00;
            target.classID = 99;
        }
    }
    
    fprintf(fp,"\r\n(findtarget)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
   

    return target;
}


void GetMobileGoal()
{ 
    //add code thats loops scans for mobile goal 
    DETECTION_OBJECT target = findTarget(0);
    // while(target.mapLocation.x == 0 || target.mapLocation.y == 0)
    while(target.classID == 99)
    {
        target = findTarget(0);
    }
    //fprintf(fp,"\r\n(FindMobile Goal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
    GrabMobileGoal(target);
}

void GrabMobileGoal(DETECTION_OBJECT Target_MG)
{
    //fprintf(fp,"\r\n(GrabMobileGoal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",Target_MG.mapLocation.x, Target_MG.mapLocation.y, Target_MG.classID, Target_MG.probability );
    Clamp.set(false);
    float MG_Offset = 18;
    float Tx = Target_MG.mapLocation.x*39.37 ; 
    float Ty = Target_MG.mapLocation.y*39.37 ;
    double X_Pos = GPS.xPosition(vex::distanceUnits::in);
    double Y_Pos = GPS.yPosition(vex::distanceUnits::in);
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading(vex::rotationUnits::deg) - targetheading);
    double result = (diff <= 180.0) ? diff : 360.0 - diff;
    if((result > 90))
    {
        targetheading +=  180;
    }
    chassis.set_heading(GPS.heading(deg));
    chassis.turn_to_angle(targetheading);
    //Drive Function
    float distance = distanceTo(Tx, Ty);
    distance = distance - MG_Offset ;
    chassis.drive_distance(distance);
    wait(20,msec);
    chassis.turn_to_angle(targetheading+180);
    chassis.drive_with_voltage(-5,-5);
    bool detectMG = false;
    float MG_max = 70;
    float MG_min = 60;
    while(!detectMG)
    {
        if (MG_Opt.hue() >= MG_min && MG_Opt.hue() <= MG_max)
        {
            detectMG = true;
        }
    }
    wait(150,msec);
    Clamp.set(true);
    chassis.drive_stop(hold);    
}
// void GetRing()
// {
//     fprintf(fp, "\rStart of GetRing function\n");
//     int Ring_Color = 0;
//     if (Side == RED)
//         Ring_Color = 1;
//     if (Side == BLUE)
//         Ring_Color = 2;
//     int turn_step = 45;

//     // vex::wait(250,msec);
//     DETECTION_OBJECT target = findTarget(Ring_Color);
//     vex::wait(100,msec);
//     if(target.probability > 0.8 && target.probability < 1)
//     {
//         Intake.spin(vex::directionType::fwd);
//         // moveToPosition(target.mapLocation.x * 100, target.mapLocation.y * 100,-1,true,75,75);
//         ScoreRing(target);
//         target.probability = 0;
//         vex::wait(100,msec);
//     }
//     else
//     {
//         chassis.turn_max_voltage = 9;
//         fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading(deg) + turn_step);
//         chassis.turn_to_angle(GPS.heading(deg) + turn_step);
//         vex::wait(100,msec);
//         DETECTION_OBJECT target = findTarget(Ring_Color);
//     }
// }



void GetRing()
{
    int Ring_Color = 0;
    if (Side == RED)
        Ring_Color = 1;
    if (Side == BLUE)
        Ring_Color = 2;

    float TurnStep = 45;

    DETECTION_OBJECT target = findTarget(Ring_Color);
    // while(target.mapLocation.x == 0 || target.mapLocation.y == 0)
    while(target.classID == 99)
    {
        fprintf(fp,"\r\n(GetRing) Inside turn loop: New heading: %.2f\n", chassis.get_absolute_heading() + TurnStep );
        chassis.turn_to_angle(chassis.get_absolute_heading() + TurnStep);
        wait(1.5,sec);
        target = findTarget(Ring_Color);
        
    }
    fprintf(fp,"\r\n(GetRing) Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
    ScoreRing(target);
}

void ScoreRing(DETECTION_OBJECT Target_Ring)
{
    fprintf(fp,"\r\n(ScoreRing) Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",Target_Ring.mapLocation.x, Target_Ring.mapLocation.y, Target_Ring.classID, Target_Ring.probability );
    float Ring_Offset = 6;
    float Tx = Target_Ring.mapLocation.x*39.37 ; 
    float Ty = Target_Ring.mapLocation.y*39.37 ;
    double X_Pos = GPS.xPosition(vex::distanceUnits::in);
    double Y_Pos = GPS.yPosition(vex::distanceUnits::in);
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading(vex::rotationUnits::deg) - targetheading);
    double result = (diff <= 180.0) ? diff : 360.0 - diff;
    if((result > 90))
    {
        targetheading +=  180;
    }
    chassis.set_heading(GPS.heading(deg));
    chassis.turn_to_angle(targetheading);
    Intake.spin(fwd);
    //Drive Function
    float distance = distanceTo(Tx, Ty);
    distance = distance - Ring_Offset;
    chassis.drive_distance(distance);
       
}

void ColorSort_Task()
{
    int R_min = 12; 
    int R_max = 20;
    int B_min = 205;
    int B_max = 225;
    
    while (true)
    {
        if(Side == BLUE)//Blue Side
        {
            if (Top_Opt.hue() >= R_min && Top_Opt.hue() <= R_max)
            {
                top_detect = true;
                //fprintf(fp, "\n\rTop Optical Triggered\n");
            }
            else
            {
                top_detect =false;
                //fprintf(fp, "\n\rTop Optical NOT Triggered\n");
            }
            
            if (Btm_Opt.hue() >= R_min && Btm_Opt.hue() <= R_max)
            {
                btm_detect = true;
                //fprintf(fp, "\n\r Bottom Optical Triggered\n");
            }
            else
            {
                btm_detect = false; 
                //fprintf(fp, "\n\rBottom Optical NOT Triggered\n");
            }
        }
        if(Side == RED)//Red Side
        {
            if (Top_Opt.hue() >= B_min && Top_Opt.hue() <= B_max)
            {
                top_detect = true;
                //fprintf(fp, "\n\rTop Optical Triggered\n");
            }
            else
            {
                top_detect = false;
                //fprintf(fp, "\n\rTop Optical NOT Triggered\n");
            }
            if (Btm_Opt.hue() >= B_min && Btm_Opt.hue() <= B_max)
            {
                btm_detect = true;
                //fprintf(fp, "\n\r Bottom Optical Triggered\n");
            }
            else
            {
                btm_detect =false;
                //fprintf(fp, "\n\rBottom Optical NOT Triggered\n");
            }
        }
        
    
        if(top_detect && btm_detect)
        {
            Intake.stop();
            Intake.spinFor(vex::directionType::fwd,400,degrees);
            // Intake.spin(vex::directionType::rev);
            // wait(250,msec);
            Intake.stop();
        }
    }
}

void MGLimit_Check()
{
    int R_min = 6; 
    int R_max = 15;
    int B_min = 205;
    int B_max = 225;
    bool start_count = false; 
    bool end_count = false; 
    int count = 0;
    
    
    while(true)
    {
        if(Side == RED)//Red Side
        {
            if (MG_Rear_Opt.hue() >= R_min && MG_Rear_Opt.hue() <= R_max)
            {
                if(MG_Rear_Opt.isNearObject())
                {
                    fprintf(fp, "\r\nMobile Goal 1 trigger is on\n");
                   start_count = true; 
                }
            }
            else
            {
                fprintf(fp, "\r\nMobile Goal 1 trigger is off\n");
                start_count = false; 
            }
            wait(5,sec);
            if (MG_Rear_Opt.hue() >= R_min && MG_Rear_Opt.hue() <= R_max)
            {
                if(MG_Rear_Opt.isNearObject())
                {
                    fprintf(fp, "\r\nMobile Goal 2 trigger is on\n");
                    end_count= true;
                }
            }
            else
            {
                fprintf(fp, "\r\nMobile Goal 2 trigger is off\n");
                end_count = false; 

            }

            if(start_count)
            {
                if(end_count)
                {
                fprintf(fp, "\r\nMobile Goal is Full\n");
                FullMG = true;
                }
            }
            else
            {
                fprintf(fp, "\r\nMobile Goal is hungry for more\n");
                FullMG = false; 
            }
        }

        if(Side == BLUE)//Blue Side
        {
            if (MG_Rear_Opt.hue() >= B_min && MG_Rear_Opt.hue() <= B_max)
            {
                if(MG_Rear_Opt.isNearObject())
                {
                   start_count = true; 
                }
            }
            else
            {
                start_count = false; 
            }
            wait(5,sec);
            if (MG_Rear_Opt.hue() >= B_min && MG_Rear_Opt.hue() <= B_max)
            {
                if(MG_Rear_Opt.isNearObject())
                {
                   end_count= true;
                }
            }
            else
            {
                end_count = false; 

            }

            if(start_count && end_count)
            {
                fprintf(fp, "\r\nMobile Goal is Full\n");
                FullMG = true;
            }
            else
            {
                fprintf(fp, "\r\nMobile Goal is hungry for more\n");
                FullMG = false; 
            }
        }
    }
}
        








      
 
     