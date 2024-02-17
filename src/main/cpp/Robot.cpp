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
inline constexpr std::string_view ElevatorPositionKey = "Elevator Position";
double ElevatorPosition;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Robot::CreateFloatPreferenceKey(rightMotorPercentagePowerKey, 0.5);
    // Robot::CreateFloatPreferenceKey(leftMotorPercentagePowerKey, -0.5);

    frc::Preferences::InitDouble(FeedSpeedKey, FeedSpeed);
    frc::Preferences::InitDouble(ShooterSpeedKey, ShooterSpeed);
    frc::Preferences::InitDouble(ElevatorPositionKey, ElevatorPosition);

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

    bool startFeed = m_stick.GetRawButton(kStartFeedButton);
    bool stopFeed = m_stick.GetRawButton(kStopFeedButton);
    bool startShooter = m_stick.GetRawButton(kStartShooterButton);
    bool stopShooter = m_stick.GetRawButton(kStopShooterButton);

    if (startFeed)
      m_feedMotor.Set(ControlMode::PercentOutput, feedMotorPower);
    if (stopFeed)
      m_feedMotor.Set(ControlMode::PercentOutput, 0);
    if (startShooter)
      m_shooterMotor.Set(ControlMode::PercentOutput, shooterMotorPower);
    if (stopShooter)
      m_shooterMotor.Set(ControlMode::PercentOutput, 0);

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

    // ShooterSpeed Button Bindings
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
  {}

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

  TalonFX m_shooterMotor{3};
  TalonFX m_feedMotor{4};
  TalonFX m_elevatorMotor{5};

  static constexpr int kStartFeedButton = 11;
  static constexpr int kStopFeedButton = 12;
  static constexpr int kStartShooterButton = 1;
  static constexpr int kStopShooterButton = 2;

  const double kElevatorPositionBottom = -0.3; 
  const double kElevatorPositionMiddle = 2.0;
  const double kElevatorPositionTop = 3.6;

};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
