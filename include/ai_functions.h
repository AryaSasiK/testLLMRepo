/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>



using namespace vex;

double distanceTo(double target_x, double target_y, vex::distanceUnits units);
double calculateBearing(double currX, double currY, double targetX, double targetY);
void moveToPoint(double target_x, double target_y, double target_theta );
DETECTION_OBJECT findTarget(int type);
void GetMobileGoal();
void GrabMobileGoal(DETECTION_OBJECT Target_MG);
void GetRing();
void ScoreRing(DETECTION_OBJECT Target_Ring);
void ColorSort_Task();
void MGLimit_Check();
