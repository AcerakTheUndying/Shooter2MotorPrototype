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

inline constexpr std::string_view FeedSpeedKey = "Feed Speed";
double FeedSpeed;
inline constexpr std::string_view ShooterSpeedKey = "Shooter Speed";
double ShooterSpeed;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Robot::CreateFloatPreferenceKey(rightMotorPercentagePowerKey, 0.5);
    // Robot::CreateFloatPreferenceKey(leftMotorPercentagePowerKey, -0.5);

    frc::Preferences::InitDouble(FeedSpeedKey, FeedSpeed);
    frc::Preferences::InitDouble(ShooterSpeedKey, ShooterSpeed);
  }

  void TeleopInit() override
  {
  }

  void TeleopPeriodic() override
  {

    double feedMotorPower = frc::Preferences::GetDouble(FeedSpeedKey);
    double shooterMotorPower = frc::Preferences::GetDouble(ShooterSpeedKey);

    // double rightMotorPower = (m_stick.GetThrottle());
    // double leftMotorPower = -rightMotorPower;

    bool start = m_stick.GetRawButton(kStartButton);
    bool stop = m_stick.GetRawButton(kStopButton);

    if (start)
    {

      m_feedMotor.Set(ControlMode::PercentOutput, FeedSpeed);
      m_shooterMotor.Set(ControlMode::PercentOutput, ShooterSpeed);
    }
    else if (stop)
    {
      m_feedMotor.Set(ControlMode::PercentOutput, 0);
      m_shooterMotor.Set(ControlMode::PercentOutput, 0);
    }
    // FeedSpeed Button Bindings
    if (m_stick.GetRawButtonPressed(7))
      feedMotorPower += 0.1;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    if (m_stick.GetRawButtonPressed(8))
      feedMotorPower -= 0.1;
     frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    if (m_stick.GetRawButtonPressed(9))
      feedMotorPower += 0.01;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    if (m_stick.GetRawButtonPressed(10))
      feedMotorPower -= 0.01;
     frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);

    //ShooterSpeed Button Bindings
        if (m_stick.GetRawButtonPressed(5))
      shooterMotorPower += 0.1;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    if (m_stick.GetRawButtonPressed(6))
      shooterMotorPower -= 0.1;
     frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    if (m_stick.GetRawButtonPressed(3))
      shooterMotorPower += 0.01;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    if (m_stick.GetRawButtonPressed(4))
      shooterMotorPower -= 0.01;
     frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
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
                                 double DefaultDoubleKeyValue);

  frc::Joystick m_stick{0};
  TalonFX m_feedMotor{3};
  TalonFX m_shooterMotor{4};

  static constexpr int kStartButton = 1;
  static constexpr int kStopButton = 12;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
