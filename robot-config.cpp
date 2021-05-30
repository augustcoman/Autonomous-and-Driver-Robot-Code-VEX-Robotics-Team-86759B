#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor MotorFL = motor(PORT11, ratio18_1, false);
motor MotorFR = motor(PORT12, ratio18_1, false);
motor MotorBL = motor(PORT13, ratio18_1, false);
motor MotorBR = motor(PORT14, ratio18_1, false);
motor IntakeL = motor(PORT15, ratio6_1, false);
motor IntakeR = motor(PORT9, ratio6_1, true);
motor ConveyorL = motor(PORT17, ratio18_1, true);
motor ConveyorR = motor(PORT18, ratio18_1, false);
inertial Inertial8 = inertial(PORT8);
encoder EncoderFB = encoder(Brain.ThreeWirePort.E);
encoder EncoderLR = encoder(Brain.ThreeWirePort.G);
line LineTrackerI = line(Brain.ThreeWirePort.C);
line LineTrackerC = line(Brain.ThreeWirePort.D);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}