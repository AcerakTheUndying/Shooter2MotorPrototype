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
#include <frc2/command/CommandPtr.h>
#include <numbers>
#include <frc/Preferences.h>

inline constexpr std::string_view motorPercentagePowerKey = "Right Motor Percentage Power";
double motorPercentagePower;

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
class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    //Robot::CreateFloatPreferenceKey(rightMotorPercentagePowerKey, 0.5);
    //Robot::CreateFloatPreferenceKey(leftMotorPercentagePowerKey, -0.5);

    frc::Preferences::InitDouble(motorPercentagePowerKey, motorPercentagePower);

  }

  void TeleopInit() override{
    

  }

  void TeleopPeriodic() override
  {

    double rightMotorPower = frc::Preferences::GetDouble(motorPercentagePowerKey);
    double leftMotorPower = -rightMotorPower;

    // double rightMotorPower = (m_stick.GetThrottle());
    // double leftMotorPower = -rightMotorPower;

    bool start = m_stick.GetRawButton(kStartButton);
    bool stop = m_stick.GetRawButton(kStopButton);

    if (start)
    {

      m_rightMotor.Set(ControlMode::PercentOutput, rightMotorPower);
      m_leftMotor.Set(ControlMode::PercentOutput, leftMotorPower);
    }
    else if (stop)
    {
      m_rightMotor.Set(ControlMode::PercentOutput, 0);
      m_leftMotor.Set(ControlMode::PercentOutput, 0);
    }


    if (m_stick.GetRawButtonPressed(7)) motorPercentagePower+=0.1;
    if (m_stick.GetRawButtonPressed(7)) motorPercentagePower-=0.1;
    if (m_stick.GetRawButtonPressed(7)) motorPercentagePower+=0.01;
    if (m_stick.GetRawButtonPressed(7)) motorPercentagePower-=0.01;
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  void RobotPeriodic() override
  {
  }
  void CreateDoublePreferenceKey(std::string_view KeyName, int DefaultKeyValue)
  {
    if (!frc::Preferences::ContainsKey(KeyName)) // Check if it doesn't already exit
    {
      frc::Preferences::InitDouble(KeyName, DefaultKeyValue); // Create it and set to value if it doesn't exit
    }
  }

private:

  void CreateDoublePreferenceKey(std::string_view KeyName,
                                double DefaultFloatKeyValue);

  frc::Joystick m_stick{0};
  TalonFX m_leftMotor{3};
  TalonFX m_rightMotor{4};

  static constexpr int kStartButton = 1;
  static constexpr int kStopButton = 12;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
