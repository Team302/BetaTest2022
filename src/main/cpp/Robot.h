// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <gamepad/TeleopControl.h>
#include <subsys/interfaces/IChassis.h>
#include <states/arm/ArmStateMgr.h>
#include <states/ballrelease/BallReleaseStateMgr.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/intake/IntakeStateMgr.h>
#include <subsys/Arm.h>
#include <subsys/BallRelease.h>
#include <subsys/BallTransfer.h>
#include <subsys/Intake.h>
#include <auton/CyclePrimitives.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  TeleopControl*        m_controller;
  IChassis*             m_chassis;
  frc::Timer*           m_timer;

  ArmStateMgr*          m_armStateMgr;
  BallReleaseStateMgr*  m_ballReleaseStateMgr;
  BallTransferStateMgr* m_ballTransferStateMgr;
  IntakeStateMgr*       m_intakeStateMgr;

  Arm*                  m_arm;
  BallRelease*          m_ballRelease;
  BallTransfer*         m_ballTransfer;
  Intake*               m_intake;
  CyclePrimitives*      m_cyclePrims;
};
