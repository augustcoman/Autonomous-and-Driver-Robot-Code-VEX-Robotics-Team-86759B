using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor MotorFL;
extern motor MotorFR;
extern motor MotorBL;
extern motor MotorBR;
extern motor IntakeL;
extern motor IntakeR;
extern motor ConveyorL;
extern motor ConveyorR;
extern inertial Inertial8;
extern encoder EncoderFB;
extern encoder EncoderLR;
extern line LineTrackerI;
extern line LineTrackerC;
extern limit LimitSwitchA;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );