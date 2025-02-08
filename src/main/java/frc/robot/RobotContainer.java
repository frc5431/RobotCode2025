// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.team5431.titan.core.joysticks.TitanController;

public class RobotContainer {

  private System systems = new System();
  private Intake intake = systems.getIntake();
  private Climber climber = systems.getClimber();

  private TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
  private TitanController operator = new TitanController(ControllerConstants.operatorPort, ControllerConstants.deadzone);

  // Controls

  // Driver Controls

  // Operator Controls

  // Intake Controls
  private Trigger intakeCoral = driver.a();


  // Climber Controls
  private Trigger climberAim = driver.b();

  public RobotContainer() {

    configureBindings();
  }

  public void periodicSubsystems() {
    intake.periodic();

  }

  public void periodic() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

  }

  private void configureDriverControls() {

  }

  private void configureOperatorControls() {

    // Intake Controls
    intakeCoral.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
