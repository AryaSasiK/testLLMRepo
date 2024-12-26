/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
//#pragma message("building for the manager")
ai::robot_link       link( PORT21, "robot_32456_1", linkType::manager );
#else
//#pragma message("building for the worker")
ai::robot_link       link( PORT21, "robot_32456_1", linkType::worker );
#endif

// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
FILE *fp = fopen("/dev/serial2","wb");
ai::jetson  jetson_comms;


brain Brain;
controller Controller1 = controller(primary);
//Constants Units (inches)
int Side = RED; 
const int32_t inertial_port = PORT10;
float wheel_size = 3.12;
float gear_ratio = .75;
float DT_Base = 9.5; 
float DT_Width = 11.56;
float GPS_Off_Base = 2.25;
float GPS_Off_Width = 2.375;
float Ox = ((DT_Base/2) - GPS_Off_Base);
float Oy = ((DT_Width/2)- GPS_Off_Width);
bool top_detect = false;
bool btm_detect =false;
bool FullMG = false; 
//Manipulators
motor Intake = motor(PORT3, ratio6_1, true);
digital_out Clamp = digital_out(Brain.ThreeWirePort.F);
//Sensors
gps GPS = gps(PORT7,Ox,Oy,distanceUnits::in,90);
optical Top_Opt = optical(PORT4);
optical Btm_Opt = optical(PORT19);
optical MG_Opt = optical(PORT17);
optical MG_Rear_Opt = optical(PORT2);

//DriveTrain 
motor leftMotorA = motor(PORT13, ratio6_1, true);
motor leftMotorB = motor(PORT12, ratio6_1, true);
motor leftMotorC = motor(PORT11, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT9, ratio6_1, false);
motor rightMotorB = motor(PORT8, ratio6_1, false);
motor rightMotorC = motor(PORT15, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
Drive chassis(ZERO_TRACKER_ODOM,
LeftDriveSmart, RightDriveSmart,
inertial_port, wheel_size,
gear_ratio, 360,
PORT21, -PORT21,
-PORT21, PORT21,
3,2.75,2,1,2.75,5.5);

// PID Constants for JAR Template
void tuned_constants()
{
  //chassis.set_drive_constants(12, 0.7, 0.0, 10, 0); // Original
  chassis.set_drive_constants(12, 0.8, 0.1, 9.8, 0); // Modded
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, 0.20, 0.005, 1.25, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.set_swing_exit_conditions(1, 300, 1000);
}

void pre_auton(void) 
{
 
  Top_Opt.setLightPower(100,pct);
  Top_Opt.setLight(ledState::on);
  Btm_Opt.setLightPower(100,pct);
  Btm_Opt.setLight(ledState::on); 
  MG_Opt.setLightPower(100,pct);
  MG_Opt.setLight(ledState::on);
  MG_Rear_Opt.setLightPower(100,pct);
  MG_Rear_Opt.setLight(vex::ledState::on);

  chassis.set_heading(GPS.heading(degrees));
  Intake.setVelocity(100,pct);
  tuned_constants();
  //Calibrate IMU & GPS
  chassis.Gyro.calibrate();
  GPS.calibrate();
  while (chassis.Gyro.isCalibrating() || GPS.isCalibrating() ) 
  {
    wait(25, msec);
  }
  wait(50, msec);
  Brain.Screen.clearScreen();
 
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void auto_Isolation(void) 
{

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void auto_Interaction(void) 
{

}

void testing_auton()
{
  
  GetMobileGoal();
  while (true)
  {
    GetRing();
    wait(500,msec);
  }

}

void toggle_clamp()
{
  if(Clamp.value())
    Clamp.set(false);
  else
    Clamp.set(true);
  wait(250,msec);
}

void usercontrol(void) {
  bool Controller1IntakeControlMotorsStopped = true;
  // User control code here, inside the loop
  while(true)
  {
    //Color_Sort_Task();
    //chassis.control_arcade();
    Controller1.ButtonA.pressed(toggle_clamp);
    // check the ButtonR1/ButtonR2 status to control Intake
      if (Controller1.ButtonR1.pressing()) {
        Intake.spin(forward);
        Controller1IntakeControlMotorsStopped = false;
      } else if (Controller1.ButtonL1.pressing()) {
        Intake.spin(reverse);
        Controller1IntakeControlMotorsStopped = false;
      } else if (!Controller1IntakeControlMotorsStopped) {
        Intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1IntakeControlMotorsStopped = true;
      }

     
  }
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) 
{
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();
  firstAutoFlag = false;
  

}

int main() 
{
  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;
  // Run at about 15Hz
  int32_t loop_time = 33;
  // start the status update display
  thread t1(dashboardTask);
  thread t2(ColorSort_Task);
  thread t3(MGLimit_Check);
  pre_auton(); 
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(testing_auton);
  Competition.drivercontrol(usercontrol);
  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);
  while(1) 
  {
      // get last map data
      jetson_comms.get_data( &local_map );
      // chassis.odom.update_position();
      // set our location to be sent to partner robot
      // link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );
      //fprintf(fp, "\r\nJetson Local Map || Position:(%.2f, %.2f) Heading: %.2f\n", local_map.pos.x * 25.4, local_map.pos.y * 25.4, local_map.pos.az);
      //fprintf(fp,"\r\nGPS Offsets: Ox: %.2f, Oy: %.2f ",Ox,Oy);
      //fprintf(fp,"\r\nGPS Position: (%.2f, %.2f) Heading: %.2f\n" , GPS.xPosition(vex::distanceUnits::in), GPS.yPosition(vex::distanceUnits::in) , GPS.heading(deg));
      //fprintf(fp, "JAR Positioning || Position:(%.2f, %.2f) Heading: %.2f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
      //fprintf(fp, "Left MG:  Dir & Pos (%d, %.2f) Righ MG:  Dir & Pos (%d, %.2f)\n", LeftDriveSmart.direction(), LeftDriveSmart.position(degrees), RightDriveSmart.direction() , RightDriveSmart.position(degrees));
      //fprintf(fp,"\r\nMobile Goal Optical Value: %.2f",MG_Opt.hue());
      //fprintf(fp,"\r\nGPS Offsets(Meters): (%.4f,%.4f)", Ox/39.37, Oy/39.37);
      
     

      // LeftDriveSmart.direction();
      // RightDriveSmart.direction();
      // RightDriveSmart.position(degrees);
      // LeftDriveSmart.position(degrees);
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();
      

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}