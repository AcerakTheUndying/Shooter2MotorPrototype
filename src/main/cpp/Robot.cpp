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

#include <frc/XboxController.h>

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
    frc::Preferences::InitDouble("Elevator Angle", 0.14);

    configs::TalonFXConfiguration cfg{};

    configs::MotionMagicConfigs &mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 1; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 1;   // Take approximately 0.5 seconds to reach max vel
    m_elevatorMotor.GetConfigurator().Apply(mm);

    // Invert Motor Direction
    configs::MotorOutputConfigs &motorOutputConfig = cfg.MotorOutput;
    motorOutputConfig.Inverted = true;
    m_elevatorMotor.GetConfigurator().Apply(motorOutputConfig);

    configs::Slot0Configs &slot0 = cfg.Slot0;
    slot0.kP = 20;
    // Not sure what this is for.  Not applied to a motor at the moment

    configs::Slot1Configs &slot1 = cfg.Slot1;
    slot1.kP = 100.0;
    slot1.kG = 0.56;
    slot1.kS = 0.144;
    slot1.kV = 1.0;
    slot1.kI = 3.0;
    m_elevatorMotor.GetConfigurator().Apply(slot1);

    /* Setup Remote Cancoder as feedback device*/

    // Create a feedback config object
    configs::FeedbackConfigs &motorFeedbackConfig = cfg.Feedback;
    // Using a remote cancoder as the feedback source
    motorFeedbackConfig.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    // Cancoder has an ID of 61
    motorFeedbackConfig.FeedbackRemoteSensorID = 61;
    // Setup the ratio between the Falcon and Elevator
    motorFeedbackConfig.RotorToSensorRatio = 30.952381; // Falcon 14:78 x 18:10
    m_elevatorMotor.GetConfigurator().Apply(motorFeedbackConfig);

    m_previousThrottlePosition = m_stick.GetThrottle();
  }

  void TeleopInit() override
  {
  }

  void TeleopPeriodic() override
  {

    units::angle::turn_t throttleEquation = (-0.0665285 * m_stick.GetThrottle() + 0.183472) * 1_tr;

    // units::angle::turn_t topThrottlePosition= 0.25_tr;
    // units::angle::turn_t bottomThrottlePosition = 0.116943_tr;

    double feedMotorPower = frc::Preferences::GetDouble(FeedSpeedKey);
    double shooterMotorPower = frc::Preferences::GetDouble(ShooterSpeedKey);

    // double rightMotorPower = (m_stick.GetThrottle());
    // double leftMotorPower = -rightMotorPower;

    bool startFeed = m_stick.GetRawButton(kStartFeedButton);
    bool stopFeed = m_stick.GetRawButton(kStopFeedButton);
    bool startShooter = m_stick.GetRawButton(kStartShooterButton);
    bool stopShooter = m_stick.GetRawButton(kStopShooterButton);

    //  Stop start feeder
    if (startFeed)
      m_feedRunning = true;
    if (stopFeed)
      m_feedRunning = false;

    if (m_feedRunning)
      m_feedMotor.Set(-feedMotorPower);
    else
      m_feedMotor.Set(0.0);

    // Stop start Shooter
    if (startShooter)
      m_shooterRunning = true;
    if (stopShooter)
      m_shooterRunning = false;
    if (m_shooterRunning)
      m_shooterMotor.Set(-shooterMotorPower);
    else
      m_shooterMotor.Set(0.0);

    // Elevator Throttle Settings
    if (m_stick.GetThrottle() != m_previousThrottlePosition)
    {
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(throttleEquation).WithSlot(1));
      frc::Preferences::SetDouble("Elevator Angle", throttleEquation.value());
      m_previousThrottlePosition = m_stick.GetThrottle();
    }

    // ShooterSpeed Button Bindings
    if (m_stick.GetRawButtonPressed(3))
    {
      shooterMotorPower += 0.1;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    }
    if (m_stick.GetRawButtonPressed(4))
    {
      shooterMotorPower -= 0.1;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    }
    if (m_stick.GetRawButtonPressed(5))
    {
      shooterMotorPower += 0.01;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    }
    if (m_stick.GetRawButtonPressed(6))
    {
      shooterMotorPower -= 0.01;
      frc::Preferences::SetDouble(ShooterSpeedKey, shooterMotorPower);
    }
    // FeedSpeed Button Bindings
    if (m_stick.GetRawButtonPressed(7))
    {
      feedMotorPower += 0.1;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    }
    if (m_stick.GetRawButtonPressed(8))
    {
      feedMotorPower -= 0.1;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    }
    if (m_stick.GetRawButtonPressed(9))
    {
      feedMotorPower += 0.01;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    }
    if (m_stick.GetRawButtonPressed(10))
    {
      feedMotorPower -= 0.01;
      frc::Preferences::SetDouble(FeedSpeedKey, feedMotorPower);
    }

    if (m_controller.GetAButton())
    {
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPositionBottom).WithSlot(1));
      frc::Preferences::SetDouble("Elevator Angle", kElevatorPositionBottom.value());
    }
    if (m_controller.GetYButton())
    {
      m_elevatorMotor.SetControl(m_mmReq.WithPosition(kElevatorPositionTop).WithSlot(1));
      frc::Preferences::SetDouble("Elevator Angle", kElevatorPositionTop.value());
    }


    // int TopHatDirection = m_stick.GetPOV();
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

  frc::XboxController m_controller{1};

  ctre::phoenix6::hardware::TalonFX m_shooterMotor{3};
  ctre::phoenix6::hardware::TalonFX m_feedMotor{4};
  ctre::phoenix6::hardware::TalonFX m_elevatorMotor{5};
  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};

  static constexpr int kStartFeedButton = 11;
  static constexpr int kStopFeedButton = 12;
  static constexpr int kStartShooterButton = 1;
  static constexpr int kStopShooterButton = 2;

  const units::angle::turn_t kElevatorPositionBottom = 0.116943_tr;

  const units::angle::turn_t kElevatorPositionTop = 0.25_tr;

  double m_previousThrottlePosition;

  // Bools to hold the states of the Shooter and Feed System
  bool m_feedRunning = false;
  bool m_shooterRunning = false;

  units::angle::turn_t m_currentElevatorAngle;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

