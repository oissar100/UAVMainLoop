#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"
#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop

// some constants defined here

Vec3f estGyroBias = Vec3f(0,0,0);
Vec3f angDes = Vec3f(0,0,0);
Vec3f rateGyro_corr;
float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;

float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

MainLoopOutput MainLoop(MainLoopInput const & in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y

  float a = 8.0f; // commanded thrust (m/s^2)
  Vec3f aa = Vec3f(0,0,0); // commanded angular acceleration (rad/s^2)
  Vec3f av = Vec3f(0,0,0); // commanded angular velocity (rad/s)

  /*if (in.userInput.buttonBlue){
        angDes = Vec3f(0,0.5236,0); // 30 degree pitch when blue button pressed
      }
  else{
      angDes = Vec3f(0,0,0); // 0 pitch otherwise
    }*/

  if (in.currentTime < 1.0f) {
      estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f); // update bias
    }

  float rollMeas = in.imuMeasurement.accelerometer.y / gravity;
  float pitchMeas = -in.imuMeasurement.accelerometer.x / gravity;

  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  float dt = 1.0f/500.0f;
  float p = 0.01f;

  // here we are defining the roll and pitch bias corrected rates

  estRoll = (1.0f-p)*(estRoll + dt*rateGyro_corr.x) + p*rollMeas;
  estPitch = (1.0f-p)*(estPitch + dt*rateGyro_corr.y) + p*pitchMeas;
  //  estRoll = estRoll + dt*rateGyro_corr.x;
  //  estPitch = estPitch + dt*rateGyro_corr.y;
  estYaw = estYaw + dt*rateGyro_corr.z;

  // height estimator
  // prediction
  estHeight = estHeight + estVelocity_3*dt;
  estVelocity_3 = estVelocity_3 + 0*dt; // assume constant

  // correction
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
    if (in.heightSensor.value < 5.0f) {
      float hMeas = in.heightSensor.value*cosf(estRoll)*cosf(estPitch);
      estHeight = (1 - mixHeight)*estHeight + mixHeight*hMeas;
      float v3Meas = (hMeas - lastHeightMeas_meas)/(in.currentTime - lastHeightMeas_time);
      estVelocity_3 = (1 - mixHeight)*estVelocity_3 + mixHeight*v3Meas;
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  // horizontal estimator
  // prediction
  estVelocity_1 = estVelocity_1 + 0*dt; // assume constant
  estVelocity_2 = estVelocity_2 + 0*dt; // assume constant

  // correction
  float const mixHorizVel = 0.1f;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;
    float div = cosf(estRoll) * cosf(estPitch);
    if (div > 0.5f) {
      float deltaPredict = estHeight/div;
      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y)*deltaPredict;
      float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x)*deltaPredict;
      estVelocity_1 = (1 - mixHorizVel)*estVelocity_1 + mixHorizVel*v1Meas;
      estVelocity_2 = (1 - mixHorizVel)*estVelocity_2 + mixHorizVel*v2Meas;
    }
  }

  // time constants

  float const timeConstant_rollAngle = 0.12f; // s
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f; // s

  float const timeConstant_rollRate = 0.04f; // s
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f; // s

  const float timeConst_horizVel = 2.0f;

  // horizontal control
  float desAcc1 = -(1/timeConst_horizVel)*estVelocity_1;
  float desAcc2 = -(1/timeConst_horizVel)*estVelocity_2;

  float desRoll = -desAcc2/gravity;
  float desPitch = desAcc1/gravity;
  float desYaw = 0;

  // vertical control
  const float desHeight = 0.5f;
  const float desAcc3 = -2*dampingRatio_height*natFreq_height*estVelocity_3 - natFreq_height*natFreq_height*(estHeight-desHeight);
  float desNormalizedAcceleration = (gravity + desAcc3)/(cosf(estRoll)*cosf(estPitch));

  // calculating desired angular velocity

  av.x = -(estRoll - angDes.x)/timeConstant_rollAngle;
  av.y = -(estPitch - angDes.y)/timeConstant_pitchAngle;
  av.z = -(estYaw - angDes.z)/timeConstant_yawAngle;

  // calculating desired angular acceleration

  aa.x = (av.x-rateGyro_corr.x)/timeConstant_rollRate;
  aa.y = (av.y-rateGyro_corr.y)/timeConstant_pitchRate;
  aa.z = (av.z-rateGyro_corr.z)/timeConstant_yawRate;

  Vec3f torque = Vec3f(inertia_xx*aa.x, inertia_yy*aa.y, inertia_zz*aa.z);
  float force = mass*a;

  float l = 33.0e-3f;
  float k = 0.01f;

//here we are calculating the mixer matrix multiplication the algebraic way
  float cp1 = (0.25f*force) + (0.25f*torque.x/l) - (0.25f*torque.y/l) + (0.25f*torque.z/k);
  float cp2 = (0.25f*force) - (0.25f*torque.x/l) - (0.25f*torque.y/l) - (0.25f*torque.z/k);
  float cp3 = (0.25f*force) - (0.25f*torque.x/l) + (0.25f*torque.y/l) + (0.25f*torque.z/k);
  float cp4 = (0.25f*force) + (0.25f*torque.x/l) + (0.25f*torque.y/l) - (0.25f*torque.z/k);

  //here we are using the function to take in force and return a PWM command
  float speed1 = speedFromForce(cp1);
  int c1 = pwmCommandFromSpeed(speed1); // function from UtilityFunctions.cpp

  float speed2 = speedFromForce(cp2);
  int c2 = pwmCommandFromSpeed(speed2);

  float speed3 = speedFromForce(cp3);
  int c3 = pwmCommandFromSpeed(speed3);

  float speed4 = speedFromForce(cp4);
  int c4 = pwmCommandFromSpeed(speed4);

    outVals.motorCommand1 = c1;
    outVals.motorCommand2 = c2;
    outVals.motorCommand3 = c3;
    outVals.motorCommand4 = c4;

  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;
  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  outVals.telemetryOutputs_plusMinus100[7] = desRoll;
  outVals.telemetryOutputs_plusMinus100[8] = desPitch;
  outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;

  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc:");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n"); // new line

  //Rate gyro measurement
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");

  printf("Last Range = %6.3fm, ",\
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last Flow: x = %6.3f, y = %6.3f\n",\
         double(lastMainLoopInputs.opticalFlowSensor.value_x),\
         double(lastMainLoopInputs.opticalFlowSensor.value_y));

  printf("Rate Gyro stuff:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  Roll = %6.3f\n", double(estRoll));
  printf("  Pitch = %6.3f\n", double(estPitch));
  printf("  Yaw = %6.3f\n", double(estYaw));

  //We print the Vec3f by printing it's three components independently:
  printf("  Estimated Gyro Bias = (%6.3f, %6.3f, %6.3f)\n",
         double(estGyroBias.x), double(estGyroBias.y), double(estGyroBias.z));
  printf("  Corrected Rate Gyro = (%6.3f, %6.3f, %6.3f)\n",
           double(rateGyro_corr.x), double(rateGyro_corr.y), double(rateGyro_corr.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");


  //If buttons are pushed these print commands are executed
  if (lastMainLoopInputs.userInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.userInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.userInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.userInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.userInput.buttonArm)
    printf("buttonArm ");



  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));
}
