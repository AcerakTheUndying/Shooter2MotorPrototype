// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>

 double rightMotorPower;
 double leftMotorPower;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and motor controller inputs as
 * range from -1 to 1 making it easy to work together.
 *
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
class Robot : public frc::TimedRobot {
 public:
  void TeleopPeriodic() override {
      double rightMotorPower = (m_stick.GetThrottle());
      double leftMotorPower = -rightMotorPower;

  bool start = m_stick.GetRawButton(kStartButton);
  bool stop = m_stick.GetRawButton(kStopButton);

  if (start){

  m_rightMotor.Set(ControlMode::PercentOutput, rightMotorPower);
  m_leftMotor.Set(ControlMode::PercentOutput, leftMotorPower);
  }
  else if (stop){
    m_rightMotor.Set(ControlMode::PercentOutput, 0);
  m_leftMotor.Set(ControlMode::PercentOutput, 0);
  }
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  void RobotPeriodic() override {

  }

  void RobotInit() override {}

 private:
  frc::Joystick m_stick{0};
  TalonFX m_leftMotor{0};
  TalonFX m_rightMotor{1};

  static constexpr int kStartButton = 2;
  static constexpr int kStopButton = 1;

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
