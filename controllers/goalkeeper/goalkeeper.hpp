#ifndef BALLCHASERBOT_HPP
#define BALLCHASERBOT_HPP

#include <webots/Robot.hpp>

#define NMOTORS 20

namespace webots {
  class Motor;
  class PositionSensor;
  class Accelerometer;
  class Camera;
  class Gyro;
  class Speaker;
  class LED;
}

namespace managers {
  class RobotisOp2GaitManager;
  class RobotisOp2MotionManager;
  class RobotisOp2VisionManager;
}

class BallChaserBot : public webots::Robot {
public:
  BallChaserBot();
  virtual ~BallChaserBot();
  void startSoccer();

private:
  void stepRobot();
  void delay(int ms);
  bool locateBall(double &x, double &y);

  int timeStep;
  webots::Motor *motors[NMOTORS];
  webots::PositionSensor *sensors[NMOTORS];
  webots::Accelerometer *accel;
  webots::Camera *cam;
  webots::Gyro *gyro;
  webots::Speaker *speaker;
  webots::LED *eyeLED;
  webots::LED *headLED;
  webots::LED *backLedRed;
  webots::LED *backLedGreen;
  webots::LED *backLedBlue;
  managers::RobotisOp2MotionManager *motionManager;
  managers::RobotisOp2GaitManager *gaitManager;
  managers::RobotisOp2VisionManager *visionManager;
};

#endif
