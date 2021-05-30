/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX and August Coman                                      */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// MotorFL              motor         11              
// MotorFR              motor         12              
// MotorBL              motor         13              
// MotorBR              motor         14              
// IntakeL              motor         15              
// IntakeR              motor         9               
// ConveyorL            motor         17              
// ConveyorR            motor         18              
// Inertial8            inertial      8               
// EncoderFB            encoder       E, F            
// EncoderLR            encoder       G, H            
// LineTrackerI         line          C               
// LineTrackerC         line          D               
// LimitSwitchA         limit         A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "autonmethods.h"

using namespace vex;

// A global instance of competition
competition Competition;
int autonSelect = 0;
int option1 = 0;
int option2 = 0;
bool option3 = true;
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.render(true,false);
  bool screenCooldown = false;
  std::string temporary;
  std::string op1txt[4] = {"Right Side", "Left Side", "Middle", "Off"};
  std::string op2txt[3] = {"Middle", "Other Side", "Off"};
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setCursor(12, 1);
  Brain.Screen.print("Tap each box to cycle options");
  Brain.Screen.setCursor(1, 6);
  Brain.Screen.print("First:");
  Brain.Screen.setCursor(1, 22);
  Brain.Screen.print("Second:");
  Brain.Screen.setCursor(1, 38);
  Brain.Screen.print("Third:");
  Brain.Screen.drawRectangle(0, 20, 159, 200);
  Brain.Screen.setCursor(6, 6);
  Brain.Screen.print("Off");
  Brain.Screen.render();

  while(true){
    if(Brain.Screen.pressing() && !screenCooldown){
      if(Brain.Screen.xPosition() < 160){
        option1++;
        if(option1 > 3)
          option1 = 0;
      }
      else if(Brain.Screen.xPosition() < 320 && Brain.Screen.xPosition() > 160){
        option2++;
        if(option2 > 2)
          option2 = 0;
      }
      else {
        if(option3){
          option3 = false;
        }
        else {
          option3 = true;
        }
      }

      Brain.Screen.drawRectangle(0, 20, 159, 200);
      Brain.Screen.setCursor(6, 3);
      Brain.Screen.print("%s", op1txt[option1].c_str());
      Brain.Screen.drawRectangle(160, 20, 159, 200);
      Brain.Screen.setCursor(6, 20);
      Brain.Screen.print("%s", op2txt[option2].c_str());
      Brain.Screen.drawRectangle(320, 20, 159, 200);
      Brain.Screen.setCursor(6, 38);
      Brain.Screen.print((option3?"enabled" : "disabled"));
      if(option1 > 1){
        Brain.Screen.setPenColor(black);
        Brain.Screen.drawRectangle(160, 20, 159, 200);
        Brain.Screen.drawRectangle(320, 20, 159, 200);
        Brain.Screen.setPenColor(white);
      }
      else if(option2 > 0){
        Brain.Screen.setPenColor(black);
        Brain.Screen.drawRectangle(320, 20, 159, 200);
        Brain.Screen.setPenColor(white);
      }

      Brain.Screen.render();
      screenCooldown = true;
    }
    if(!Brain.Screen.pressing()){
      screenCooldown = false;
    }
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  if(option1 == 2){
    //Middle Only
    Inertial8.setRotation(0, deg);
    MotorFL.setStopping(brake);
    MotorFR.setStopping(brake);
    MotorBL.setStopping(brake);
    MotorBR.setStopping(brake);
    IntakeL.setVelocity(100,percent);
    IntakeR.setVelocity(100,percent);
    ConveyorL.setVelocity(100,percent);
    ConveyorR.setVelocity(100,percent);
    ConveyorL.setStopping(hold);
    ConveyorR.setStopping(hold);
    IntakeL.setStopping(hold);
    IntakeR.setStopping(hold);
    ConveyorL.startRotateFor(120, deg);
    ConveyorR.startRotateFor(120, deg);
    driveFB(-16.5, 60);
    IntakeL.startRotateFor(-120, deg);
    IntakeR.startRotateFor(-120, deg);
    driveLR(24, 60);
    driveFB(7, 40);
    doGoalWork(2, 2, true);
    driveFB(-5, 60);
  } else if (option1 != 3){
    if(option3 && option2 == 0){
      if(option1 == 0){
        //Right Clutch
          Inertial8.setRotation(0, deg);
          MotorFL.setStopping(brake);
          MotorFR.setStopping(brake);
          MotorBL.setStopping(brake);
          MotorBR.setStopping(brake);
          IntakeL.setVelocity(100,percent);
          IntakeR.setVelocity(100,percent);
          ConveyorL.setVelocity(100,percent);
          ConveyorR.setVelocity(100,percent);
          ConveyorL.setStopping(hold);
          ConveyorR.setStopping(hold);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          ConveyorL.startRotateFor(120, deg);
          ConveyorR.startRotateFor(120, deg);
          driveLR(-11.5, 100);
          ConveyorL.startRotateFor(-120, deg);
          ConveyorR.startRotateFor(-120, deg);
          //ConveyorL.startRotateFor(120, deg);
          //ConveyorR.startRotateFor(120, deg);
          IntakeL.startRotateFor(-120, deg);  
          IntakeR.startRotateFor(-120, deg);
          waitUntil(IntakeL.isDone() && IntakeR.isDone());
          //wait(0.4, sec);
          ConveyorL.startRotateFor(800, deg);
          ConveyorR.startRotateFor(800, deg);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          IntakeL.stop();
          IntakeR.stop();
          driveAndTake(10, 100);
          IntakeL.setStopping(hold);
          IntakeR.setStopping(hold);
          IntakeL.stop();
          IntakeR.stop();
          waitUntil(ConveyorL.isDone());
          ConveyorL.startRotateFor(-120, deg);
          ConveyorR.startRotateFor(-120, deg);
          driveFB(-12.7, 100);
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(45, 100);
          driveLR(47, 100);
          ConveyorL.startRotateFor(700, deg);
          ConveyorR.startRotateFor(700, deg);
          IntakeL.startRotateFor(360, deg);
          IntakeR.startRotateFor(360, deg);
          driveFB(18, 100);
          driveFB(-28.5, 100);
          //-29
          rotate(59, 100);
          ConveyorL.spin(forward);
          ConveyorR.spin(forward);
          IntakeL.spin(forward);
          IntakeR.spin(forward);
          //IntakeL.startRotateFor(180, deg);
          //IntakeR.startRotateFor(180, deg);
          //driveAndTake(68 + 1, 100);
          driveAndTake(68 + 1, 100);
          while(LineTrackerI.reflectivity() < 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
          }
          IntakeL.stop();
          IntakeR.stop();
          doGoalWork(1, 0, true);
          IntakeL.startRotateFor(-240, deg);  
          IntakeR.startRotateFor(-240, deg);
          //ConveyorL.startRotateFor(700, deg);
          //ConveyorR.startRotateFor(700, deg);
          driveFB(-11, 90);
      } else {
        //Left Clutch
          Inertial8.setRotation(0, deg);
          MotorFL.setStopping(brake);
          MotorFR.setStopping(brake);
          MotorBL.setStopping(brake);
          MotorBR.setStopping(brake);
          IntakeL.setVelocity(100,percent);
          IntakeR.setVelocity(100,percent);
          ConveyorL.setVelocity(100,percent);
          ConveyorR.setVelocity(100,percent);
          ConveyorL.setStopping(hold);
          ConveyorR.setStopping(hold);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          driveLR(11.5, 100);
          ConveyorL.startRotateFor(120, deg);
          ConveyorR.startRotateFor(120, deg);
          IntakeL.startRotateFor(-120, deg);  
          IntakeR.startRotateFor(-120, deg);
          waitUntil(IntakeL.isDone() && IntakeR.isDone());
          //wait(0.4, sec);
          ConveyorL.startRotateFor(700, deg);
          ConveyorR.startRotateFor(700, deg);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          IntakeL.stop();
          IntakeR.stop();
          driveAndTake(10, 100);
          IntakeL.setStopping(hold);
          IntakeR.setStopping(hold);
          IntakeL.stop();
          IntakeR.stop();
          ConveyorL.startRotateFor(-120, deg);
          ConveyorR.startRotateFor(-120, deg);
          driveFB(-12.7, 100);
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(-45, 100);
          driveLR(-47, 100);
          ConveyorL.startRotateFor(700, deg);
          ConveyorR.startRotateFor(700, deg);
          IntakeL.startRotateFor(360, deg);
          IntakeR.startRotateFor(360, deg);
          driveFB(18, 100);
          driveFB(-29, 100);
          rotate(-59, 100);
          ConveyorL.spin(forward);
          ConveyorR.spin(forward);
          IntakeL.spin(forward);
          IntakeR.spin(forward);
          //IntakeL.startRotateFor(180, deg);
          //IntakeR.startRotateFor(180, deg);
          //driveAndTake(68 + 1, 100);
          driveFB(68 + 1, 100);
          wait(0.3, sec);
          doGoalWork(1, 0, true);
          IntakeL.startRotateFor(-240, deg);  
          IntakeR.startRotateFor(-240, deg);
          //ConveyorL.startRotateFor(700, deg);
          //ConveyorR.startRotateFor(700, deg);
          driveFB(-11, 90);
      }
    } else {
      if(option1 == 0){
        //Right Goal
        Inertial8.setRotation(0, deg);
        MotorFL.setStopping(brake);
        MotorFR.setStopping(brake);
        MotorBL.setStopping(brake);
        MotorBR.setStopping(brake);
        IntakeL.setVelocity(100,percent);
        IntakeR.setVelocity(100,percent);
        ConveyorL.setVelocity(100,percent);
        ConveyorR.setVelocity(100,percent);
        ConveyorL.setStopping(hold);
        ConveyorR.setStopping(hold);
        IntakeL.setStopping(coast);
        IntakeR.setStopping(coast);
        driveLR(-11.5, 100);
        ConveyorL.startRotateFor(180, deg);
        ConveyorR.startRotateFor(180, deg);
        IntakeL.startRotateFor(-120, deg);  
        IntakeR.startRotateFor(-120, deg);
        waitUntil(IntakeL.isDone() && IntakeR.isDone());
        IntakeL.setStopping(coast);
        IntakeR.setStopping(coast);
        IntakeL.stop();
        IntakeR.stop();
        driveAndTake(10, 100);
        doGoalWork(1, 0, true);
        IntakeL.setStopping(hold);
        IntakeR.setStopping(hold);
        IntakeL.stop();
        IntakeR.stop();
        if(option2 != 1){
          driveFB(-12.7, 100);
        }
        if(option2 == 0){
          //Right to Middle
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(45, 100);
          driveLR(47, 100);
          IntakeL.startRotateFor(360, deg);
          IntakeR.startRotateFor(360, deg);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          driveFB(18, 100);
          doGoalWork(2, 2, true);
          driveFB(-10, 90);
        } else if(option2 == 1){
          //Right to Side
          driveFB(-46, 100);
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(45, 100);
          driveLR(46 + 0.5, 100);
          ConveyorL.stop();
          ConveyorR.stop();
          IntakeL.stop();
          IntakeR.stop();
          rotate(45, 100);
          driveAndTake(46 + 4, 100);
          doGoalWork(3, 2, true);
          driveFB(-12, 90);
        }
      } else if(option1 == 1){
        //Left Goal
        Inertial8.setRotation(0, deg);
        MotorFL.setStopping(brake);
        MotorFR.setStopping(brake);
        MotorBL.setStopping(brake);
        MotorBR.setStopping(brake);
        IntakeL.setVelocity(100,percent);
        IntakeR.setVelocity(100,percent);
        ConveyorL.setVelocity(100,percent);
        ConveyorR.setVelocity(100,percent);
        ConveyorL.setStopping(hold);
        ConveyorR.setStopping(hold);
        IntakeL.setStopping(coast);
        IntakeR.setStopping(coast);
        driveLR(11.5, 100);
        ConveyorL.startRotateFor(180, deg);
        ConveyorR.startRotateFor(180, deg);
        IntakeL.startRotateFor(-120, deg);  
        IntakeR.startRotateFor(-120, deg);
        waitUntil(IntakeL.isDone() && IntakeR.isDone());
        IntakeL.setStopping(coast);
        IntakeR.setStopping(coast);
        IntakeL.stop();
        IntakeR.stop();
        driveAndTake(10, 100);
        doGoalWork(1, 0, true);
        IntakeL.setStopping(hold);
        IntakeR.setStopping(hold);
        IntakeL.stop();
        IntakeR.stop();
        if(option2 != 1){
          driveFB(-12.7, 100);
        }
        if(option2 == 0){
          //Left to Middle
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(-45, 100);
          driveLR(-47, 100);
          IntakeL.startRotateFor(360, deg);
          IntakeR.startRotateFor(360, deg);
          IntakeL.setStopping(coast);
          IntakeR.setStopping(coast);
          driveFB(18, 100);
          doGoalWork(2, 2, true);
          driveFB(-10, 90);
        } else if(option2 == 1){
          //Left to Side
          driveFB(-46.5, 100);
          if(LineTrackerI.reflectivity() >= 5){
            IntakeL.spin(forward);
            IntakeR.spin(forward);
            ConveyorL.spin(forward);
            ConveyorR.spin(forward);
            waitUntil(LineTrackerI.reflectivity() < 5);
            IntakeL.stop();
            IntakeR.stop();
            ConveyorL.stop();
            ConveyorR.stop();
          }
          rotate(-45, 100);
          driveLR(-46, 100);
          ConveyorL.stop();
          ConveyorR.stop();
          IntakeL.stop();
          IntakeR.stop();
          rotate(-45, 100);
          driveAndTake(46 + 4, 100);
          doGoalWork(3, 2, true);
          driveFB(-12, 90);
        }
      }
    }
  }

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {





  // User control code here, inside the loop
  /*
    wait(3, sec);
  Inertial8.setRotation(0, deg);
  waitUntil(Controller1.ButtonA.pressing());
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  //ConveyorL.startRotateFor(120, deg);
  //ConveyorR.startRotateFor(120, deg);
  //IntakeL.startRotateFor(-120, deg);  
  //IntakeR.startRotateFor(-120, deg);
  //waitUntil(IntakeL.isDone() && IntakeR.isDone());
  driveFB(6, 100);
  */
  

  
  //Brain.Screen.render(true,false);
  MotorFL.setStopping(coast);
  MotorFR.setStopping(coast);
  MotorBL.setStopping(coast);
  MotorBR.setStopping(coast);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);
  IntakeR.startRotateFor(-120, deg);
  waitUntil((IntakeL.isDone() && IntakeR.isDone()) || Controller1.ButtonL1.pressing());
  MotorFL.spin(forward);
  MotorFR.spin(forward);
  MotorBL.spin(forward);
  MotorBR.spin(forward);
  double speedMult = 0.63;
  //bool cool = false;
  //bool cool2 = false;
  //int cv = 100;
  //Controller1.Screen.setCursor(4, 1);
  //Controller1.Screen.print(cv);
  double speedFL;
  double speedFR;
  double speedBL;
  double speedBR;
  //double slew2;
  //double slew3;
  //double slew4;
  
  while (true) {
   /* This is the old speedhalf code. the new one will do fullspeed while button A is pressing
   if ((Controller1.ButtonA.pressing() and cool == false)) {
      if (speedHalf == 1) {
        speedHalf = 2;
      } else {
        speedHalf = 1;
      }
    }
    if (Controller1.ButtonA.pressing()) {
      cool = true;
    } else {
      cool = false;
    }
    */
    if(Controller1.ButtonA.pressing() || (Controller1.ButtonR2.pressing() && Controller1.ButtonR1.pressing())){
      speedMult = 1.0;
    } else {
      speedMult = 0.63;
    }
    /*
    if ((Controller1.ButtonUp.pressing() and cool2 == false) and cv < 100){
      cv = (cv + 10);
    }
     if ((Controller1.ButtonDown.pressing() and cool2 == false) and cv > 0){
      cv = (cv - 10);
    }
    if ((Controller1.ButtonUp.pressing())or(Controller1.ButtonDown.pressing())) {
      cool2 = true;
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(4, 1);
      Controller1.Screen.print(cv);
    } else {
      cool2 = false;
    }
    */
    //ConveyorL.setVelocity(cv,percent);
    //ConveyorR.setVelocity(cv,percent);

    if (Controller1.ButtonR2.pressing() && !(Controller1.ButtonR1.pressing())) {
      ConveyorL.spin(forward);
      ConveyorR.spin(forward);
    } else if (Controller1.ButtonR1.pressing() && !(Controller1.ButtonR2.pressing())) {
      ConveyorL.spin(reverse);
      ConveyorR.spin(reverse);
    } else {
      ConveyorL.stop();
      ConveyorR.stop();
    }

     if (Controller1.ButtonL2.pressing()) {
      IntakeL.spin(forward);
      IntakeR.spin(forward);
    } else if (Controller1.ButtonL1.pressing()) {
      IntakeL.spin(reverse);
      IntakeR.spin(reverse);
    } else {
      IntakeL.stop();
      IntakeR.stop();
    }
    
    speedFL = ((Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent)) - Controller1.Axis2.position(percent));
    speedFR = ((Controller1.Axis4.position(percent) - Controller1.Axis3.position(percent)) - Controller1.Axis2.position(percent));
    speedBL = ((-1) * (Controller1.Axis4.position(percent) - Controller1.Axis3.position(percent)) - Controller1.Axis2.position(percent));
    speedBR = ((-1) * (Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent)) - Controller1.Axis2.position(percent));
    
    //slew2 = pow((double)Controller1.Axis2.position(percent) / 100, 2) * ((Controller1.Axis2.position(percent) >= 0)?1:-1);
    //slew3 = pow((double)Controller1.Axis3.position(percent) / 100, 2) * ((Controller1.Axis3.position(percent) >= 0)?1:-1);
    //slew4 = pow((double)Controller1.Axis4.position(percent) / 100, 2) * ((Controller1.Axis4.position(percent) >= 0)?1:-1);
    //((slew3 + slew4) - slew2) * 100
    //((slew4 - slew3) - slew2) * 100
    //((-1)*(slew4 - slew3) - slew2) * 100
    //((-1)*(slew3 + slew4) - slew2) * 100
    

    MotorFL.setVelocity(speedFL * speedMult, percent);
    MotorFR.setVelocity(speedFR * speedMult, percent);
    MotorBL.setVelocity(speedBL * speedMult, percent);
    MotorBR.setVelocity(speedBR * speedMult, percent);
    
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    //
    //Brain.Screen.clearLine(1,color::black);
    //Brain.Screen.setCursor(1,0);
    //Brain.Screen.print(ConveyorL.rotation(deg));
    //Brain.Screen.render();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

/*
  wait(3, sec);
  waitUntil(Controller1.ButtonA.pressing());
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  driveLR(-11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone());
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  ConveyorL.stop();
  ConveyorR.stop();
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-160, deg);
  ConveyorR.startRotateFor(-160, deg);
  driveFB(-12.7, 100);
  rotate(45, 90);
  driveLR(47, 100);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  driveFB(6 + 4, 100);
  doGoalWork(1, 0, false);
  driveFB(-9, 100);
  driveLR(43, 100);
  rotate(45, 90);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  driveAndTake(17 + 4, 100);
  doGoalWork(1, 0, false);
  driveFB(-11.3, 100);
*/

/* Old skills auton:
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone());
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(26.5, 60);
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  rotate(135, 60);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(29, 60);
  doGoalWork(1, 0, true);
  ConveyorL.startRotateFor(-160, deg);
  ConveyorR.startRotateFor(-160, deg);
  driveFB(-46, 70);
  rotate(135, 60);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(23.5, 50);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  rotate(-90, 60);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(30.5, 60);
  doGoalWork(1, 0, true);
  driveFB(-5, 60);
  rotate(90, 60);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(49.5, 70);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  rotate(-45, 60);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(11.3, 60);
  doGoalWork(1, 0, true);
  ConveyorL.startRotateFor(-160, deg);
  ConveyorR.startRotateFor(-160, deg);
  driveFB(-12.7, 60);
  rotate(135, 60);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(47, 70);
  rotate(-90, 60);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(6, 60);
  doGoalWork(1, 0, true);
  driveFB(-5, 60);
  driveLR(49, 70);
  rotate(45, 60);
  driveFB(11.3, 60);
  doGoalWork(1, 1, false);
  IntakeL.spin(reverse);
  IntakeR.spin(reverse);
  ConveyorL.spin(reverse);
  ConveyorR.spin(reverse);
  //driveFB(-11.3, 60);
  //Now here is the added middle goal:
  driveFB(-45.3, 70);
  rotate(135, 60);
  ConveyorL.stop();
  ConveyorR.stop();
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(24, 60);
  rotate(90, 60);
  IntakeL.stop();
  IntakeR.stop();
  IntakeL.setVelocity(60,percent);
  IntakeR.setVelocity(60,percent);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(13, 50);
  IntakeL.stop();
  IntakeR.stop();
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  doGoalWork(1, 3, false);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveFB(-12, 60);
*/

/* Old Tournament Auton:
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(-12.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone());
  wait(0.4, sec);
  ConveyorL.startRotateFor(600, deg);
  ConveyorR.startRotateFor(600, deg);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  ConveyorL.stop();
  ConveyorR.stop();
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-160, deg);
  ConveyorR.startRotateFor(-160, deg);
  driveFB(-12.7, 100);
  rotate(45, 60);
  driveLR(47, 100);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  driveFB(18, 100);
  ConveyorL.startRotateFor(600, deg);
  ConveyorR.startRotateFor(600, deg);
  waitUntil(ConveyorL.isDone());
  driveFB(-9, 100);
  driveLR(43, 90);
  rotate(45, 90);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  driveAndTake(17 + 4, 100);
  ConveyorL.startRotateFor(600, deg);
  ConveyorR.startRotateFor(600, deg);
  waitUntil(ConveyorL.isDone());
*/
/* Here is what I plan on adding to mah functions:
  driveFB(-34, 100);
  rotate(53, 100);
  driveAndTake(68 + 1, 100);
  doGoalWork(1, 0, true);
*/

/* Tourney Auton Methods:

//Right Goal:
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(-11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  doGoalWork(3, 2, true);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  driveFB(-12.7, 100);

//Right to Middle: 
  rotate(45, 100);
  IntakeL.spin(reverse);
  IntakeR.spin(reverse);
  ConveyorL.spin(reverse);
  ConveyorR.spin(reverse);
  driveLR(47, 100);
  ConveyorL.stop();
  ConveyorR.stop();
  IntakeL.stop();
  IntakeR.stop();
  driveFB(18, 100);
  doGoalWork(1, 2, false);
  driveFB(-10, 100);

//Right to Side:
  driveFB(-46, 100);
  rotate(45, 100);
  driveLR(46, 100);
  ConveyorL.stop();
  ConveyorR.stop();
  IntakeL.stop();
  IntakeR.stop();
  rotate(45, 100);
  driveAndTake(46 + 4, 100);
  doGoalWork(2, 2, false);
  driveFB(-12, 90);

//Right Clutch
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(-11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  //wait(0.4, sec);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  driveFB(-12.7, 100);
  if(LineTrackerI.reflectivity() >= 5){
    IntakeL.spin(forward);
    IntakeR.spin(forward);
    ConveyorL.spin(forward);
    ConveyorR.spin(forward);
    waitUntil(LineTrackerI.reflectivity() < 5);
    IntakeL.stop();
    IntakeR.stop();
    ConveyorL.stop();
    ConveyorR.stop();
  }
  rotate(45, 100);
  driveLR(47, 100);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.startRotateFor(360, deg);
  IntakeR.startRotateFor(360, deg);
  driveFB(18, 100);
  driveFB(-29, 100);
  rotate(59, 100);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  //IntakeL.startRotateFor(180, deg);
  //IntakeR.startRotateFor(180, deg);
  //driveAndTake(68 + 1, 100);
  driveFB(68 + 1, 100);
  wait(0.3, sec);
  doGoalWork(1, 0, true);
  IntakeL.startRotateFor(-240, deg);  
  IntakeR.startRotateFor(-240, deg);
  //ConveyorL.startRotateFor(700, deg);
  //ConveyorR.startRotateFor(700, deg);
  driveFB(-11, 90);

  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(-11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  //wait(0.4, sec);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  driveFB(-12.7, 100);
  if(LineTrackerI.reflectivity() >= 5){
    IntakeL.spin(forward);
    IntakeR.spin(forward);
    ConveyorL.spin(forward);
    ConveyorR.spin(forward);
    waitUntil(LineTrackerI.reflectivity() < 5);
    IntakeL.stop();
    IntakeR.stop();
    ConveyorL.stop();
    ConveyorR.stop();
  }
  rotate(45, 100);
  driveLR(47, 100);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.startRotateFor(360, deg);
  IntakeR.startRotateFor(360, deg);
  driveFB(18, 100);
  driveFB(-34, 100);
  rotate(53, 100);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  IntakeL.startRotateFor(180, deg);
  IntakeR.startRotateFor(180, deg);
  driveAndTake(68 + 1, 100);
  doGoalWork(1, 0, true);
  driveFB(-11, 90);

//Middle only(start near right goal)
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  driveFB(-16.5, 60);
  IntakeL.startRotateFor(-120, deg);
  IntakeR.startRotateFor(-120, deg);
  driveLR(24, 60);
  driveFB(7, 40);
  doGoalWork(2, 2, true);
  driveFB(-5, 60);

//to put in auton:
  if(option1 == 2){
    //Middle Only
  } else if (option1 != 3){
    if(option3 && option2 == 0){
      if(option1 == 0){
        //Right Clutch
      } else {
        //Left Clutch
      }
    } else {
      if(option1 == 0){
        //Right Goal
        if(option2 == 0){
          //Right to Middle
        } else if(option2 == 1){
          //Right to Side
        }
      } else if(option1 == 1){
        //Left Goal
        if(option2 == 0){
          //Left to Middle
        } else if(option2 == 1){
          //Left to Side
        }
      }
    }
  }

*/





/*Old Auton & Skills:
  if(autonSelect == 0){



  //Note: this is for auton skills. Robot Starts on right of field.
   Here are some temporary ideas:
     -make intakes coast right before entering a goal to reduce pushing
     -Give robot greater overshoot heading for the goals
     -Maybe avoid pushing down balls with conveyor until you reach the goal?
     -Get rid of the rotational correction tolerance
  
  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(26.5, 60);
  rotate(135, 90);
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(29 + 2, 90);
  doGoalWork(1, 0, true);
  IntakeL.startRotateFor(180, deg);
  IntakeR.startRotateFor(180, deg);
  driveFB(-46, 90);
  rotate(135, 90);
  ConveyorL.startRotateFor(-160, deg);
  ConveyorR.startRotateFor(-160, deg);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(23, 90);
  rotate(-90, 90);
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(31.5 + 2, 90);
  doGoalWork(1, 0, true);
  driveFB(-5, 90);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  rotate(90, 90);
  driveFB(47, 90);
  //47.5
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  rotate(-45, 90);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(12.7 + 2, 90);
  doGoalWork(1, 0, true);
  //Maybe try to make the intakes spin forward a bit to get the balls in. If not, add a stop statement for sure.
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();

  //ConveyorL.startRotateFor(-160, deg);
  //ConveyorR.startRotateFor(-160, deg);
  //this is where it branches off
  driveFB(-46, 90);
  IntakeL.startRotateFor(1000, deg);
  IntakeR.startRotateFor(1000, deg);
  ConveyorL.setVelocity(60,percent);
  ConveyorR.setVelocity(60,percent);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  waitUntil(LineTrackerC.reflectivity() >= 5);
  ConveyorL.stop();
  ConveyorR.stop();
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  rotate(135, 90);
  driveAndTake(23.5, 90);
  rotate(-90, 90);
  driveAndTake(30.5 + 2, 90);
  IntakeL.startRotateFor(-120, deg);
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  doGoalWork(1, 0, true);
  driveFB(-5, 90);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveLR(49, 90);
  rotate(45, 90);
  //
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  //
  ConveyorL.startRotateFor(-80, deg);
  ConveyorR.startRotateFor(-80, deg);
  driveFB(11.3 + 2, 90);
  doGoalWork(1, 0, true);
  //
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  //
  driveFB(-12.7, 90);
  ConveyorL.setVelocity(70,percent);
  ConveyorR.setVelocity(70,percent);
  ConveyorL.spin(reverse);
  ConveyorR.spin(reverse);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  waitUntil(LineTrackerI.reflectivity() >= 5);
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  waitUntil(LineTrackerI.reflectivity() < 5);
  ConveyorL.stop();
  ConveyorR.stop();
  rotate(45, 90);
  driveLR(47.5, 90);
  //48.5 orig
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.spin(forward);
  ConveyorR.spin(forward);
  driveFB(9, 90);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.startRotateFor(120, deg);
  IntakeR.startRotateFor(120, deg);
  waitUntil(ConveyorL.isDone());
  driveFB(-8, 90);
  rotate(-180, 90);
  //driveLR(-1.0, 90);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(34, 50);
  IntakeL.setVelocity(100, percent);
  IntakeR.setVelocity(100, percent);
  ConveyorL.setStopping(coast);
  ConveyorR.setStopping(coast);
  doGoalWork(1, 1, false);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveFB(-12, 90);
  IntakeL.spin(reverse);
  IntakeR.spin(reverse);
  ConveyorL.spin(reverse);
  ConveyorR.spin(reverse);
  rotate(90, 90);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.stop();
  ConveyorR.stop();
  rotate(-135, 90);
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  driveFB(34, 90);
  ConveyorL.startRotateFor(240, deg);
  ConveyorR.startRotateFor(240, deg);
  rotate(-101.8, 90);
  //-77, -101.3
  IntakeL.stop();
  IntakeR.stop();
  driveFB(67, 90);
  doGoalWork(1, 0, true);
  driveFB(-10, 90);
  } else if (autonSelect == 1){



  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(-11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  //wait(0.4, sec);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  driveFB(-12.7, 100);
  if(LineTrackerI.reflectivity() >= 5){
    IntakeL.spin(forward);
    IntakeR.spin(forward);
    ConveyorL.spin(forward);
    ConveyorR.spin(forward);
    waitUntil(LineTrackerI.reflectivity() < 5);
    IntakeL.stop();
    IntakeR.stop();
    ConveyorL.stop();
    ConveyorR.stop();
  }
  rotate(45, 100);
  driveLR(47, 100);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.startRotateFor(360, deg);
  IntakeR.startRotateFor(360, deg);
  driveFB(18, 100);
  driveFB(-10, 100);
  driveLR(44, 100);
  rotate(45, 100);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  //
  //ConveyorL.spin(forward);
  //ConveyorR.spin(forward);
  //
  driveFB(17 + 4, 100);
  //IntakeL.startRotateFor(180, deg);
  //IntakeR.startRotateFor(180, deg);
  //ConveyorL.startRotateFor(700, deg);
  //ConveyorR.startRotateFor(700, deg);
  //waitUntil(ConveyorL.isDone());
  } else if(autonSelect == 2){



  Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  driveLR(11.5, 100);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  waitUntil(IntakeL.isDone() && IntakeR.isDone());
  //wait(0.4, sec);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  driveAndTake(10, 100);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  IntakeL.stop();
  IntakeR.stop();
  ConveyorL.startRotateFor(-120, deg);
  ConveyorR.startRotateFor(-120, deg);
  driveFB(-12.7, 100);
  if(LineTrackerI.reflectivity() >= 5){
    IntakeL.spin(forward);
    IntakeR.spin(forward);
    ConveyorL.spin(forward);
    ConveyorR.spin(forward);
    waitUntil(LineTrackerI.reflectivity() < 5);
    IntakeL.stop();
    IntakeR.stop();
    ConveyorL.stop();
    ConveyorR.stop();
  }
  rotate(-45, 100);
  driveLR(-47, 100);
  ConveyorL.startRotateFor(700, deg);
  ConveyorR.startRotateFor(700, deg);
  IntakeL.startRotateFor(360, deg);
  IntakeR.startRotateFor(360, deg);
  driveFB(18, 100);
  driveFB(-10, 100);
  driveLR(-44, 100);
  rotate(-45, 100);
  IntakeL.setStopping(coast);
  IntakeR.setStopping(coast);
  IntakeL.stop();
  IntakeR.stop();
  //
  //ConveyorL.spin(forward);
  //ConveyorR.spin(forward);
  //
  driveFB(17 + 4, 100);
  //IntakeL.startRotateFor(180, deg);
  //IntakeR.startRotateFor(180, deg);
  //ConveyorL.startRotateFor(700, deg);
  //ConveyorR.startRotateFor(700, deg);
  //waitUntil(ConveyorL.isDone());
  } else {
    
    Inertial8.setRotation(0, deg);
  MotorFL.setStopping(brake);
  MotorFR.setStopping(brake);
  MotorBL.setStopping(brake);
  MotorBR.setStopping(brake);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  ConveyorL.setStopping(hold);
  ConveyorR.setStopping(hold);
  IntakeL.setStopping(hold);
  IntakeR.setStopping(hold);
  ConveyorL.startRotateFor(120, deg);
  ConveyorR.startRotateFor(120, deg);
  driveFB(-13.5, 60);
  driveLR(24, 60);
  driveFB(4, 40);
  ConveyorL.startRotateFor(600, deg);
  ConveyorR.startRotateFor(600, deg);
  waitUntil(ConveyorL.isDone());
  driveFB(-4, 50);
  rotate(180, 70);
  IntakeL.startRotateFor(-120, deg);  
  IntakeR.startRotateFor(-120, deg);
  driveFB(30, 50);
  


    bool screenCool = false;
  while(true){
    if(Brain.Screen.pressing() && !screenCool){
      autonSelect++;
      if(autonSelect > 3){
        autonSelect = 0;
      }
      screenCool = true;
    }
    if(!Brain.Screen.pressing()){
      screenCool = false;
    }
    Brain.Screen.clearLine(1,color::black);
    Brain.Screen.setCursor(1,0);
    if(autonSelect == 0){
      Brain.Screen.print("Skills Autonomous");
    } else if(autonSelect == 1){
      Brain.Screen.print("15 Second Auton: Right side");
    } else if(autonSelect == 2){
      Brain.Screen.print("15 Second Auton: Left side");
    } else {
      Brain.Screen.print("Autonomous is OFF");
    }
    Brain.Screen.render();

    Controller1.Screen.clearLine(4);
    Controller1.Screen.setCursor(4, 1);
    Controller1.Screen.print(Inertial8.rotation(deg));
    wait(20, msec);
  }

*/