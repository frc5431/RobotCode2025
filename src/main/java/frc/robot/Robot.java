// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    // Logger.recordMetadata("Titan2025", "5431Robot");
    // AutoLogOutputManager.addPackage("frc.robot.Subsystems");

    // if (isReal()) {
    //   Logger.addDataReceiver(new WPILOGWriter());
    //   Logger.addDataReceiver(new NT4Publisher());
    //   Logger.start();

    // } else {

    //   setUseTiming(false);
    //   String logPath = LogFileUtil.findReplayLog();
    //   Logger.setReplaySource(new WPILOGReader(logPath));
    //   Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    //       "_sim")));

    // }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.onInitialize();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() {
    m_robotContainer.onInitialize();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    m_robotContainer.onInitialize();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
