#include "vex.h"
using namespace vex;

extern brain Brain;
extern controller Controller1;
  
//Manipulators
extern motor Intake;
extern digital_out Clamp;
//Sensors
extern gps GPS;
extern optical Top_Opt;
extern optical Btm_Opt;
extern optical MG_Opt;
extern optical MG_Rear_Opt;

extern int Side ;
extern bool top_detect;
extern bool btm_detect;
extern FILE *fp ;
extern bool FullMG ;

extern Drive chassis;


enum TeamColor 
{ 
  RED, 
  BLUE 
}; 
