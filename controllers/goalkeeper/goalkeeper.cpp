#include "BallChaserBot.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2VisionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static double clampValue(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double motorMinPositions[NMOTORS];
static double motorMaxPositions[NMOTORS];

static const char *jointNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

BallChaserBot::BallChaserBot() : Robot() {
  timeStep = getBasicTimeStep();

  eyeLED = getLED("EyeLed");
  headLED = getLED("HeadLed");
  headLED->set(0x00FF00);
  backLedRed = getLED("BackLedRed");
  backLedGreen = getLED("BackLedGreen");
  backLedBlue = getLED("BackLedBlue");
  cam = getCamera("Camera");
  cam->enable(2 * timeStep);
  accel = getAccelerometer("Accelerometer");
  accel->enable(timeStep);
  gyro = getGyro("Gyro");
  gyro->enable(timeStep);
  speaker = getSpeaker("Speaker");

  for (int i = 0; i < NMOTORS; i++) {
    motors[i] = getMotor(jointNames[i]);
    string sensorLabel = jointNames[i];
    sensorLabel.push_back('S');
    sensors[i] = getPositionSensor(sensorLabel);
    sensors[i]->enable(timeStep);
    motorMinPositions[i] = motors[i]->getMinPosition();
    motorMaxPositions[i] = motors[i]->getMaxPosition();
  }

  motionManager = new RobotisOp2MotionManager(this);
  gaitManager = new RobotisOp2GaitManager(this, "config.ini");
  visionManager = new RobotisOp2VisionManager(cam->getWidth(), cam->getHeight(), 28, 20, 50, 45, 0, 30);
}

BallChaserBot::~BallChaserBot() {}

void BallChaserBot::stepRobot() {
  if (step(timeStep) == -1)
    exit(EXIT_SUCCESS);
}

void BallChaserBot::delay(int ms) {
  double start = getTime();
  while (getTime() < start + (double)ms / 1000.0)
    stepRobot();
}

bool BallChaserBot::locateBall(double &x, double &y) {
  static int w = cam->getWidth();
  static int h = cam->getHeight();

  const unsigned char *image = cam->getImage();
  bool found = visionManager->getBallCenter(x, y, image);

  if (!found) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = 2.0 * x / w - 1.0;
    y = 2.0 * y / h - 1.0;
    return true;
  }
}

void BallChaserBot::startSoccer() {
  cout << "Soccer Bot Program Initiated - Prepared by Abdhulla" << endl;
  
  speaker->speak("Hello! I am Ball Chaser Bot. I can find the ball, walk, and even kick it!", 1.0);

  stepRobot();
  eyeLED->set(0x00FF00);

  motionManager->playPage(1);
  motionManager->playPage(24);
  motionManager->playPage(9);
  delay(200);

  gaitManager->start();
  gaitManager->step(timeStep);

  double previousX = 0.0;
  double previousY = 0.0;
  int countFaceDown = 0;
  int countFaceUp = 0;
  const double tolerance = 70.0;
  const int fallSteps = 18;

  while (true) {
    double x, y, neckAngle, headAngle;
    bool ballVisible = locateBall(x, y);
    const double *accValues = accel->getValues();

    if (accValues[1] < 512.0 - tolerance)
      countFaceUp++;
    else
      countFaceUp = 0;

    if (accValues[1] > 512.0 + tolerance)
      countFaceDown++;
    else
      countFaceDown = 0;

    if (countFaceUp > fallSteps) {
      motionManager->playPage(1);
      motionManager->playPage(10);
      motionManager->playPage(9);
      countFaceUp = 0;
    }
    else if (countFaceDown > fallSteps) {
      motionManager->playPage(1);
      motionManager->playPage(11);
      motionManager->playPage(9);
      countFaceDown = 0;
    }
    else if (ballVisible) {
      eyeLED->set(0x0000FF);

      x = 0.02 * x + previousX;
      y = 0.02 * y + previousY;
      previousX = x;
      previousY = y;
      neckAngle = clampValue(-x, motorMinPositions[18], motorMaxPositions[18]);
      headAngle = clampValue(-y, motorMinPositions[19], motorMaxPositions[19]);

      if (y < 0.1)
        gaitManager->setXAmplitude(1.0);
      else
        gaitManager->setXAmplitude(0.5);

      gaitManager->setAAmplitude(neckAngle);
      gaitManager->step(timeStep);

      motors[18]->setPosition(neckAngle);
      motors[19]->setPosition(headAngle);

      if (y > 0.35) {
        gaitManager->stop();
        delay(400);
        eyeLED->set(0x00FF00);

        if (x < 0.0)
          motionManager->playPage(13); 
        else
          motionManager->playPage(12); 

        motionManager->playPage(9);
        gaitManager->start();
        previousX = 0.0;
        previousY = 0.0;
      }
    }
    else {
      eyeLED->set(0xFF0000);

      gaitManager->setXAmplitude(0.0);
      gaitManager->setAAmplitude(-0.25);
      gaitManager->step(timeStep);

      headAngle = clampValue(0.7 * sin(2.5 * getTime()), motorMinPositions[19], motorMaxPositions[19]);
      motors[19]->setPosition(headAngle);
    }

    stepRobot();
  }
}
