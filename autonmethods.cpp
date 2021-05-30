#include "autonmethods.h"

double pi = 3.14159265358979323846;
// This holds the value for pi used in converting radians to degrees
double radius = 1.375;
// The radius of the encoder wheel is important in converting degrees to distance traveled
double kp = 0.155;
// This is the constant multiplied to the error in proportion control. It has been adjusted through trials to minimize overshoot.(1)
// Originally 0.12 before the added change
double kpr = 5;
// This is the constant multiplied to the error in proportion control for the rotation method.(1)
double kd = 0.0115;
//originally 0.012
// This is the constant multiplied to the derivative (error minus previousError) in derivative control. It too has been adjusted.(2)
double correct = 0;
// This is the correction variable, subtracted from the velocities of each motor to get the robot to face its original orientation.(3)
int minCap = 20;

double error;
//This is the difference in degrees of the target distance and the distance travelled. It is used in calculating proportion and derivative.(1)
double derivative;
//This is the derivative of the error, the current error minus the error of the previous iteration, divided by the loop delay time.(2)
double previousError;
//For calculating the derivative, the error is stored in this at the end of the loop to provide the error of the previous iteration to the next.(2)
double prevDegrees = 0;
//This is the running sum of all of the degrees changes for each rotation command. It stores the previous degrees from the original position.(3)
double speed;
//This value, after a few calculations, tells the motors how fast they have to move given proportion and derivative control.
int maxSpeed;
//This is the maximum speed the driving motors will be allowed to reach. Since it is already a parameter, this seems redundant, and I may delete it.
int capSpeed;
//This caps the driving motors' speeds if they attempt to exceed this value. It starts at 20 and gradually reaches maxSpeed as it moves.(1)

/*
Proportion Control (1):
The robot's velocity cannot immediately reach zero once it is at its destination. attempting to decelerate to rest instantly will result in the robot
overshooting past its target distance by an unpredictable amount. Proportion control aims to solve this by gradually slowing down the robot as it
approaches its target distance. By setting the speed of the motors to the error (target distance minus distance travelled), the speed will decrease
as the robot comes closer and closer to its target (causing the error to decrease). Here's a calculation:

error = targetDistance - distanceTravelled; (note: I will convert the target distance to degrees to work with the encoder, which senses degrees.)

motor.setVelocity(error * constant);

I must multiply the error by a constant, since values that are too large will cause the robot to slow down rapidly and too late, while values too small
will cause the robot to slow down too early. I had to run tests and tune this value to optimize speed and minimize overshoot.

constant = Kp
targetDistance = (dist * 180) / (radius * pi)     (note: the target distance dist is in inches. It is converted on the spot to degrees.)
distanceTravelled = -EncoderFB.rotation(deg)      (note: this method returns the rotation of this encoder object. It is negative, hence the "-".)

The motor.setVelocity sets each of the motor objects to the given velocity. Some of them are facing other sides, so they are multiplied by -1. 
*/
/*
Derivative Control(2):
Proportion control is not perfect. It will undershoot at longer distances and overshoot at short distances, regardless of kP. The derivative is the rate of change in 
the error, added directly to the speed. It too is tuned by kD, another constant to adjust its intensity. 
This minimizes the problem in just proportion control as previously stated.

derivative = (error - previousError) / 0.02     the divided by 0.02 is the change in time, for each cycle of the loop.
speed = error * kP + derivative - kD;
*/


//Here are the variables for the doGoalWork function, a function that will score and descore a number of balls based on parameters.
double reachDelayTime;
double reachStartTime;
double distReaching;
int bDescNow;
int bScoreNow;
int bDescPrev;
int iRep;
bool intakePause;
//bool intakeCooldown;
bool intakeEnd;
bool convEnd;
bool outCool;
bool cRev;
bool timeCool;
bool isReaching;
bool goBack;


/** driveFB - drives the robot forward or backward with given parameters
  * Preconditions - distance and maximum speed must be entered. maxSpeed must be between 0 and 100.
  * Postconditions - the robot drives forward given the distance entered converted to rotations.
  * @parameter dist - the distance in inches the robot is to move forward. Negative values are used to go in reverse.
  * @parameter maxSpeed - when the robot moves, it's motors speeds will not exceed this value.
  */
void driveFB(double dist, int maxSpeed){
  EncoderFB.setRotation(0,deg);
  previousError = 0;
  //the int below declares the counter for the iterations.
  int toleranceCount = 0;
  //the double below declares the tolerance range in degrees
  double tolerance = (0.5 * 180) / (radius * pi);
  wait(20, msec);
  //the part with the limit switch ends the driveFB when its going forward so it doesn't spend the whole time running into the goal.
  //a side effect of the limit switch is that it "re-calibrates" the position on an axis, as it ends on a constant position.
  while (/*fabs(EncoderFB.rotation(deg)) <= fabs((dist * 180) / (radius * pi)) &&*/ !(LimitSwitchA.pressing() && dist > 0)){
    //below is where the error is calculated, converting the distance to radians and then to degrees. EncoderFB is negative, which is why its added.
    error = (dist * 180) / (radius * pi) + EncoderFB.rotation(deg);
    //below is the derivative (2)
    derivative = (error - previousError) / 0.02;
    /*
    the below if statement sets the capspeed, which uses proportion control to slowly accelerate to a faster speed at start.
    It increases with the encoder rotation, and stops at the maximum specified speed. 
    The purpose is to prevent sliding from rapid acceleration by making it gradual.
    */
    if (fabs(EncoderFB.rotation(deg)) * kp + minCap > maxSpeed) {
      capSpeed = maxSpeed;
    }
    else {
      capSpeed = fabs(EncoderFB.rotation(deg)) * kp + minCap;
    }
    //if the sum that makes the speed is greater than the capspeed, it is capped. Otherwise it is calculated normally(2)
    if (fabs(error * kp + derivative * kd) > capSpeed) {
      speed = capSpeed * ((dist >= 0)?1:-1);
    }
    else {
      speed = (error * kp + derivative * kd + 2.0 * ((dist >= 0)?1:-1));
    }
    //above, a constant is added to keep a minimum speed. This is necessary, as the speed value can go so low that the robot stops moving

    /*
    The if statements below set the correct variable to the difference between the current orientation
    (Inertial8.rotation()) and the previous rotation prevDegrees. It adds this value to the wheels differently depending on the
    side of the robot to cause it to turn to the correct orientation while moving.
    */
    correct = (Inertial8.rotation() - prevDegrees) * ((fabs(speed) >= 40)?2:1);
    //the speed and correct variables are used to set each motor's velocity
    MotorFL.setVelocity(speed - correct, percent);
    MotorFR.setVelocity(speed * (-1) - correct, percent);
    MotorBL.setVelocity(speed - correct, percent);
    MotorBR.setVelocity(speed * (-1) - correct, percent);
    MotorFL.spin(forward);
    MotorBL.spin(forward);
    MotorFR.spin(forward);
    MotorBR.spin(forward);
    //sets the previous error to the current error. this becomes the previous error in the next iteration.
    previousError = error;
    
    //The autonomous will stop if it is within a tolerance range that is acceptable
    //the tolerance count counts the number of iterations, 20 milliseconds long, that the roboti is within tolerance.
    if((error < tolerance) && (error > -tolerance)){
      toleranceCount++;
    } else {
      toleranceCount = 0;
    }
    if(toleranceCount > 10){
      break;
    }

    wait(20, msec);
  }
  MotorFL.stop();
  MotorFR.stop();
  MotorBL.stop();
  MotorBR.stop();
}

//below is the same as driveFB, but it will spin until a ball enters the intakes
//this is for taking a ball in, specifically for a goal to avoid taking in multiple
void driveAndTake(double dist, int maxSpeed){
  EncoderFB.setRotation(0,deg);
  previousError = 0;
  IntakeL.spin(forward);
  IntakeR.spin(forward);
  intakePause = false;
  int toleranceCount = 0;
  double tolerance = (0.5 * 180) / (radius * pi);
  wait(20, msec);
  //the part with the limit switch ends the driveFB when its going forward so it doesn't spend the whole time running into the goal.
  //a side effect of the limit switch is that it "re-calibrates" the position on an axis, as it ends on a constant position.
  while (/*fabs(EncoderFB.rotation(deg)) <= fabs((dist * 180) / (radius * pi)) &&*/ !(LimitSwitchA.pressing() && dist > 0)){
    error = (dist * 180) / (radius * pi) + EncoderFB.rotation(deg);
    derivative = (error - previousError) / 0.02;
    if (fabs(EncoderFB.rotation(deg)) * kp + minCap > maxSpeed) {
      capSpeed = maxSpeed;
    }
    else {
      capSpeed = fabs(EncoderFB.rotation(deg)) * kp + minCap;
    }
    if (fabs(error * kp + derivative * kd) > capSpeed) {
      speed = capSpeed * ((dist >= 0)?1:-1);
    }
    else {
      speed = (error * kp + derivative * kd + 2.0 * ((dist >= 0)?1:-1));
    }
    correct = (Inertial8.rotation() - prevDegrees) * ((fabs(speed) >= 40)?2:1);
    MotorFL.setVelocity(speed - correct, percent);
    MotorFR.setVelocity(speed * (-1) - correct, percent);
    MotorBL.setVelocity(speed - correct, percent);
    MotorBR.setVelocity(speed * (-1) - correct, percent);
    MotorFL.spin(forward);
    MotorBL.spin(forward);
    MotorFR.spin(forward);
    MotorBR.spin(forward);
    //sets the previous error to the current error. this becomes the previous error in the next iteration.
    previousError = error;
    //This is the code that is different. If it detects that it recieved a ball, the intakes stop
    if (LineTrackerI.reflectivity() < 5){
      intakePause = true;
    }
    if (intakePause && LineTrackerI.reflectivity() >= 5){
      IntakeL.stop();
      IntakeR.stop();
    }
    if((error < tolerance) && (error > -tolerance)){
      toleranceCount++;
    } else {
      toleranceCount = 0;
    }
    if(toleranceCount > 10){
      break;
    }
    wait(20, msec);
  }
  IntakeL.stop();
  IntakeR.stop();
  MotorFL.stop();
  MotorFR.stop();
  MotorBL.stop();
  MotorBR.stop();
}

//right is positive

//below is the same as driveFB, just for strafing.
void driveLR(double dist, int maxSpeed){
  EncoderLR.setRotation(0,deg);
  previousError = 0;
  int toleranceCount = 0;
  double tolerance = (0.5 * 180) / (radius * pi);
  wait(20, msec);
  while (true /*fabs(EncoderLR.rotation(deg)) <= fabs((dist * 180) / (radius * pi))*/){
    error = (dist * 180) / (radius * pi) + EncoderLR.rotation(deg);
    derivative = (error - previousError) / 0.02;
    if (fabs(EncoderLR.rotation(deg)) * kp + minCap > maxSpeed) {
      capSpeed = maxSpeed;
    }
    else {
      capSpeed = fabs(EncoderLR.rotation(deg)) * kp + minCap;
    }
    if (fabs(error * kp + derivative * kd) > capSpeed) {
      speed = capSpeed * ((dist >= 0)?1:-1);
    }
    else {
      speed = (error * kp + derivative * kd + 2.0 * ((dist >= 0)?1:-1));
    }
    correct = (Inertial8.rotation() - prevDegrees) * ((fabs(speed) >= 40)?2:1);
    MotorFL.setVelocity(speed - correct, percent);
    MotorFR.setVelocity(speed - correct, percent);
    MotorBL.setVelocity(speed * (-1) - correct, percent);
    MotorBR.setVelocity(speed * (-1) - correct, percent);
    MotorFL.spin(forward);
    MotorBL.spin(forward);
    MotorFR.spin(forward);
    MotorBR.spin(forward);
    previousError = error;
    if((error < tolerance) && (error > -tolerance)){
      toleranceCount++;
    } else {
      toleranceCount = 0;
    }
    if(toleranceCount > 10){
      break;
    }
    wait(20, msec);
  }
  MotorFL.stop();
  MotorFR.stop();
  MotorBL.stop();
  MotorBR.stop();
}

/** rotate - rotates the robot to the specified degrees (3)
  * Preconditions - maxSpeed must be between 0 and 100
  * Postconditions - the robot rotates the specified degrees
  * @parameter degrees - the amount of degrees the robot will rotate clockwise. Use negative for counter.
  * @parameter maxSpeed - the maximum speed the robot may rotate.
  */

//clockwise is positive
void rotate(double degrees, int maxSpeed){
  /*
  at first, the robot rotates at the maximum speed until it inevitably overshoots.
  This is done on purpose, since the inertial sensor will allow for the overshoot to be 
  accurately corrected, yielding the desired rotation.
  */
  MotorFL.setVelocity(maxSpeed * ((degrees>=0)?1:-1), percent);
  MotorFR.setVelocity(maxSpeed * ((degrees>=0)?1:-1), percent);
  MotorBL.setVelocity(maxSpeed * ((degrees>=0)?1:-1), percent);
  MotorBR.setVelocity(maxSpeed * ((degrees>=0)?1:-1), percent);
  MotorFL.spin(forward);
  MotorBL.spin(forward);
  MotorFR.spin(forward);
  MotorBR.spin(forward);
  while (true){
    wait(20, msec);
    /* the Inertial rotation minus the previous degrees makes up the rotation from
    its original orientation (prevDegrees). if it is greater than the value of the desired rotation (degrees), then it will proceed
    because it met and exceeded its desired rotation.
    */
    if (fabs(Inertial8.rotation() - prevDegrees) >= fabs(degrees)){
      break;
    }
  }
  //the robot stops and waits a little
  MotorFL.setVelocity(0, percent);
  MotorFR.setVelocity(0, percent);
  MotorBL.setVelocity(0, percent);
  MotorBR.setVelocity(0, percent);
  wait(20, msec);
  /*
  This loop then uses proportion control to slowly rotate back to the desired orientation. it has a default maximum speed of 20.
  I did not find derivative control necessary, since proportion worked just fine.
  */
  while (true){
    //if the formula for the desired speed is over 20, speed is set to 20 times 1 or -1 depending on direction.
    if (fabs(Inertial8.rotation() - prevDegrees - degrees) * kpr >= 20) {
      speed = 20 * ((degrees>=0)?1:-1);
   }
   //If the formula is below 20, the speed is set to that formula's value.
    else {
      speed = ((Inertial8.rotation() - prevDegrees - degrees) * kpr + 2.0 * ((degrees >= 0)?1:-1));
    }
    if (fabs(Inertial8.rotation() - prevDegrees) <= fabs(degrees)) {
      break;
    }
  MotorFL.setVelocity(speed * (-1), percent);
  MotorFR.setVelocity(speed * (-1), percent);
  MotorBL.setVelocity(speed * (-1), percent);
  MotorBR.setVelocity(speed * (-1), percent);
  wait(20, msec);
  }
  MotorFL.stop();
  MotorFR.stop();
  MotorBL.stop();
  MotorBR.stop();
  //prevDegrees is the running sum of all rotations from the original. It keeps track of its current orientation.
  prevDegrees += degrees;
}


/* This is the method responsible for scoring and descoring from the goals
everything is contained in a loop. The robot performs actions, which are toggled on or 
off with booleans once certain conditions are met.
*/
void doGoalWork(int bScore, int bDesc, bool intakeCooldown){
  EncoderFB.setRotation(0,deg);
  IntakeL.setVelocity(100,percent);
  IntakeR.setVelocity(100,percent);
  ConveyorL.setVelocity(100,percent);
  ConveyorR.setVelocity(100,percent);
  MotorFL.setVelocity(8, percent);
  MotorFR.setVelocity(-8, percent);
  MotorBL.setVelocity(8, percent);
  MotorBR.setVelocity(-8, percent);
  bDescNow = 0;
  intakePause = true;
  outCool = false;
  bScoreNow = 0;
  isReaching = false;
  goBack = true;
  intakeEnd = false;
  convEnd = false;
  distReaching = 40;
  reachStartTime = Brain.timer(sec);
  while (bDescNow < bDesc || bScoreNow < bScore || isReaching || !intakeEnd || !convEnd){
    bDescPrev = bDescNow;
    if(bDescNow < bDesc) {
      if(!intakeCooldown){
      if (LineTrackerI.reflectivity() < 5) {
        intakePause = false;
        iRep = 0;
      } else if (!intakePause){
        if (iRep < 1) {
          iRep++;
        } else {
          intakePause = true;
          intakeCooldown = true;
          bDescNow++;
          iRep = 0;
        }
      }
      }
    } else if(!intakeEnd){
      IntakeL.stop();
      IntakeR.stop();
      IntakeL.startRotateFor(-120, deg);
      IntakeL.startRotateFor(-120, deg);
      intakeEnd = true;
    }
    /*To space the balls, when a ball is taken in, it won't take another until another is
    launched, or if there are no more balls to launch. intakeCooldown is activated when
    a ball is detected in the intakes, and deactivated when a ball is launched.*/
    if (convEnd || LineTrackerC.reflectivity() > 4){
      intakeCooldown = false;
    }
    if (intakeCooldown){
      IntakeL.stop();
      IntakeR.stop();
    } else if (!intakeEnd) {
      IntakeL.spin(forward);
      IntakeR.spin(forward);
    }
    if(bScoreNow < bScore || outCool){
      if(!outCool) {
        ConveyorL.spin(forward);
        ConveyorR.spin(forward);
        if(LineTrackerC.reflectivity() > 4){
          ConveyorL.resetRotation();
          outCool = true;
          bScoreNow++;
        }
      }  else if (ConveyorL.rotation(degrees) > 300 && !cRev){
        /* if (LineTrackerC.reflectivity() > 4) {
          ConveyorL.spin(reverse);
          ConveyorR.spin(reverse);
          cRev = true;
        } else {
          outCool = false;
        }
        */
        outCool = false;
      }
      if (cRev && LineTrackerC.reflectivity() <= 5) {
        cRev = false;
        outCool = false;
      }
    } else if (bDescNow > bDescPrev && LineTrackerC.reflectivity() <= 5){
      ConveyorL.startRotateFor(190, deg);
      ConveyorR.startRotateFor(190, deg);
    } else if (ConveyorL.isDone()){
      ConveyorL.stop();
      ConveyorR.stop();
      convEnd = true;
    }
    reachDelayTime = Brain.timer(sec) - reachStartTime;
    if (!isReaching && reachDelayTime > 2){
      isReaching = true;
    }
    if (isReaching){
      if (goBack) {
        MotorFL.spin(reverse);
        MotorFR.spin(reverse);
        MotorBL.spin(reverse);
        MotorBR.spin(reverse);
        if (EncoderFB.rotation(deg) >= distReaching) {
          goBack = false;
        } 
      } else {
        MotorFL.spin(forward);
        MotorFR.spin(forward);
        MotorBL.spin(forward);
        MotorBR.spin(forward);
        if (EncoderFB.rotation(deg) <= 0){
          goBack = true;
          isReaching = false;
          reachStartTime = Brain.timer(sec);
          MotorFL.stop();
          MotorFR.stop();
          MotorBL.stop();
          MotorBR.stop();
        }
      }
    }    reachDelayTime = Brain.timer(sec) - reachStartTime;
    if (!isReaching && reachDelayTime > 2){
      isReaching = true;
    }
    if (isReaching){
      if (goBack) {
        MotorFL.spin(reverse);
        MotorFR.spin(reverse);
        MotorBL.spin(reverse);
        MotorBR.spin(reverse);
        if (EncoderFB.rotation(deg) >= distReaching) {
          goBack = false;
        } 
      } else {
        MotorFL.spin(forward);
        MotorFR.spin(forward);
        MotorBL.spin(forward);
        MotorBR.spin(forward);
        if (EncoderFB.rotation(deg) <= 0){
          goBack = true;
          isReaching = false;
          reachStartTime = Brain.timer(sec);
          MotorFL.stop();
          MotorFR.stop();
          MotorBL.stop();
          MotorBR.stop();
        }
      }
    }
    if (intakeEnd){
      reachStartTime = Brain.timer(sec);
    }
    wait(20, msec);
  }
}

//double kp = 0.115;