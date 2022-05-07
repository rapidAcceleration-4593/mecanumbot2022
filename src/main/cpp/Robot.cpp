// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/XboxController.h>
#include "ctre/Phoenix.h"
#include <frc/AnalogGyro.h>
#include <frc/Timer.h>
#include <frc/Encoder.h>

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
   //m_frontRight.SetInverted(true);
  //m_rearRight.SetInverted(true);
  //m_frontLeft.SetInverted(true);
  m_rearLeft.SetInverted(true);
  m_magicEncoder.SetDistancePerPulse((1./1024.));
  m_magicEncoder.SetSamplesToAverage(5);
  m_magicEncoder.SetMinRate(1.0);
  // m_magicEncoder.Reset();
  
  }
  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
	
    
	if (m_stick.GetAButton()){

		m_shooterLeft.Set(-1);
  	m_shooterRight.Set(1);
	
    m_timer.Start();
    while (double(m_timer.Get()) < .5)
    {
      if(double(m_timer.Get()) > .35)
      {
        m_meterWheel.Set(-.25);
      }
    }
    // m_timer.Stop();
    // m_timer.Reset();

    // m_meterWheel.Set(0);
    // m_shooterRight.Set(0);
    // m_shooterLeft.Set(0);
      
    // m_timer.Start();
    // while(double(m_timer.Get()) < 2)
    // {
    //   std::cout << "hold your horses buckaroo" << std::endl;

    // }
    // m_timer.Stop();
    // m_timer.Reset();
  }
  else 
  {
    m_shooterLeft.Set(0);
    m_shooterRight.Set(0);
  }
  
	if (m_stick.GetBButton())
  {
		m_meterWheel.Set(-.25);
	} 
  else if (m_stick.GetYButton())
  {
    m_meterWheel.Set(.25);
  }
	else
  {
		m_meterWheel.Set(0);
	}
	
	//std::cout << m_stick.GetRightX() << std::endl;
	//std::cout << m_stick.GetRightY() << std::endl;
  
	m_robotDrive.DriveCartesian(m_stick.GetLeftX()*.5, m_stick.GetRightY()*.5, m_stick.GetRightX()*.5);//, m_gyro.GetAngle());
    
  std::cout << m_magicEncoder.GetRate()*60 << std::endl;
	// std::cout << "hello cash" << std::endl;
	//std::cout << m_gyro.GetAngle() << std::endl;  
  }

 private:
  

  static constexpr int kFrontLeftChannel = 4;
  static constexpr int kRearLeftChannel = 3;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 1;

  static constexpr int kLeftShooter = 6;
  static constexpr int kRightShooter = 5;
  static constexpr int kMeterWheel = 0;

  static constexpr int m_gyroPort = 0;

  static constexpr int kJoystickChannel = 0;
 
  frc::Talon m_shooterRight{kRightShooter};
  frc::Talon m_shooterLeft{kLeftShooter};
  frc::Talon m_meterWheel{kMeterWheel};

  frc::PWMSparkMax m_frontLeft{kFrontLeftChannel};
  frc::PWMSparkMax m_rearLeft{kRearLeftChannel};
  frc::PWMSparkMax m_frontRight{kFrontRightChannel};
  frc::PWMSparkMax  m_rearRight{kRearRightChannel};

  frc::MecanumDrive m_robotDrive{ m_rearLeft, m_frontLeft,  m_rearRight, m_frontRight};

  frc::AnalogGyro m_gyro{m_gyroPort};
  frc::XboxController m_stick{kJoystickChannel};

  frc::Encoder m_magicEncoder{0,1, false, frc::Encoder::EncodingType::k1X};

  frc::Timer m_timer;
};
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif