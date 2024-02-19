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
#include <frc2/command/CommandPtr.h>
#include <numbers>
#include <frc/Preferences.h>
#include <frc2/command/button/POVButton.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

using namespace ctre::phoenix6;

inline constexpr std::string_view FeedSpeedKey = "Feed Speed";
double FeedSpeed = -0.5;
inline constexpr std::string_view ShooterSpeedKey = "Shooter Speed";
double ShooterSpeed = -0.7;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Robot::CreateFloatPreferenceKey(rightMotorPercentagePowerKey, 0.5);
    // Robot::CreateFloatPreferenceKey(leftMotorPercentagePowerKey, -0.5);

    frc::Preferences::InitDouble(FeedSpeedKey, FeedSpeed);
    frc::Preferences::InitDouble(ShooterSpeedKey, ShooterSpeed);

    configs::TalonFXConfiguration cfg{};

    configs::MotionMagicConfigs &mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 20; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 20;   // Take approximately 0.5 seconds to reach max vel

    configs::Slot0Configs &slot0 = cfg.Slot0;
    slot0.kP = 20;

    m_initialThrottlePosition = m_stick.GetThrottle();

    
  }

  void TeleopInit() override
  {
  }

  void TeleopPeriodic() override
  {
    units::angle::turn_t throttleEquation = (0.1135 * m_stick.GetThrottle() + 0.1265) * 1_tr;

  double topThrottlePosition;
  double bottomThrottlePosition;

    double feedMotorPower = frc::Preferences::GetDouble(FeedSpeedKey);
    double shooterMotorPower = frc::Preferences::GetDouble(ShooterSpeedKey);

    // double rightMotorPower = (m_stick.GetThrottle());
    // double leftMotorPower = -rightMotorPower;

    bool startFeed = m_stick.GetRawButton(kStartFeedButton);
    bool stopFeed = m_stick.GetRawButton(kStopFeedButton);
    bool startShooter = m_stick.GetRawButton(kStartShooterButton);
    bool stopShooter = m_stick.GetRawButton(kStopShooterButton);

    bool toggleFeed = true;
    bool toggleShooter = true;

    bool elevator1 = m_stick.GetRawButton(kElevatorPositionButton1);
    bool elevator2 = m_stick.GetRawButton(kElevatorPositionButton2);
    bool elevator3 = m_stick.GetRawButton(kElevatorPositionButton3);
    bool elevator4 = m_stick.GetRawButton(kElevatorPositionButton4);

    if (startFeed)
      toggleFeed = true;
    if (stopFeed)
      toggleFeed = false;
    if (toggleFeed)
      m_feedMotor.Set(feedMotorPower);

    if (startShooter)
      toggleShooter = true;
    if (stopShooter)
      toggleShooter = false;
    if (toggleShooter)
      m_shooterMotor.Set(shooterMotorPower);

    //Elevator Position Buttons
    if (elevator1)
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPosition1).WithSlot(0));
    if (elevator2)
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPosition2).WithSlot(0));
    if (elevator3)
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPosition3).WithSlot(0));
    if (elevator4)
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPosition4).WithSlot(0));
   
    //Elevator Throttle Settings
      if (m_initialThrottlePosition != m_stick.GetThrottle())
    {
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(throttleEquation).WithSlot(0));
    }

    


    // ShooterSpeed Button Bindings
    if (m_stick.GetRawButtonPressed(5))
      shooterMotorPower += 0.1;
    frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    if (m_stick.GetRawButtonPressed(6))
      shooterMotorPower -= 0.1;
    frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);

    // FeedSpeed Button Bindings
    if (m_stick.GetRawButtonPressed(3))
      feedMotorPower += 0.1;
    frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    if (m_stick.GetRawButtonPressed(4))
      feedMotorPower -= 0.1;
    frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);

int TopHatDirection = m_stick.GetPOV();

if (TopHatDirection = 90){
  topThrottlePosition = m_stick.GetThrottle();
}

if (TopHatDirection = 180){
  bottomThrottlePosition = m_stick.GetThrottle();
}




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
  

  ctre::phoenix6::hardware::TalonFX m_shooterMotor{3};
  ctre::phoenix6::hardware::TalonFX m_feedMotor{4};
  ctre::phoenix6::hardware::TalonFX m_elevatorMotor{5};
  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};

  static constexpr int kStartFeedButton = 11;
  static constexpr int kStopFeedButton = 12;
  static constexpr int kStartShooterButton = 1;
  static constexpr int kStopShooterButton = 2;

  static constexpr int kElevatorPositionButton1 = 7;
  static constexpr int kElevatorPositionButton2 = 8;
  static constexpr int kElevatorPositionButton3 = 9;
  static constexpr int kElevatorPositionButton4 = 10;

  const units::angle::turn_t kElevatorPosition1 = 0.013_tr;
  const units::angle::turn_t kElevatorPosition2 = 0.07_tr;
  const units::angle::turn_t kElevatorPosition3 = 0.15_tr;
  const units::angle::turn_t kElevatorPosition4 = 0.24_tr;

  double m_initialThrottlePosition;


};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
