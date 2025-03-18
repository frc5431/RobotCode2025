// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Field;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.SmartStowCommand;
import frc.robot.Subsytems.CANdle.CANdleSystem;
import frc.robot.Subsytems.CANdle.CANdleSystem.AnimationTypes;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.RobotMechanism;
import frc.robot.Util.SwerveConstants;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.team5431.titan.core.joysticks.TitanController;

import lombok.Getter;

public class RobotContainer {

	private final Systems systems = new Systems();
	private final RobotMechanism robotMechanism = new RobotMechanism();
	private final Intake intake = systems.getIntake();
	private final IntakePivot intakePivot = systems.getIntakePivot();
	private final Feeder feeder = systems.getFeeder();
	private final Elevator elevator = systems.getElevator();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();
	private final CANdleSystem candle = Systems.getTitanCANdle();
	private final Drivebase drivebase = Systems.getDrivebase();
	private final SendableChooser<Command> autoChooser;

	private TitanController driver = Systems.getDriver();
	private TitanController operator = Systems.getOperator();

	// Triggers

	// Automated Triggers

	// Game Status
	private @Getter Trigger isEndgame = new Trigger(
			() -> DriverStation.getMatchTime() <= 5 && DriverStation.isTeleop());
	private @Getter Trigger isAutonEnabled = new Trigger(() -> DriverStation.isAutonomousEnabled());
	private @Getter Trigger isAutonDisabled = new Trigger(
			() -> DriverStation.isAutonomous() || DriverStation.isDisabled());
	private Trigger elevatorTarget = new Trigger(
			() -> elevator.getPositionSetpointGoal(elevator.getPosition().rotation, ElevatorConstants.error)
					&& manipJoint.getPositionSetpointGoal(manipJoint.getMode().position, ManipJointConstants.error));

	// Subsystem Triggers
	private @Getter Trigger isIntaking = new Trigger(
			() -> intake.getMode() == IntakeModes.INTAKE || intake.getMode() == IntakeModes.FEED);

	// Subsystem Triggers
	private @Getter Trigger hasCoral = new Trigger(
			() -> manipulator.hasCoral());

	// LED Triggers
	/*
	 * // Driver Controls
	 * // private Trigger alignRightReef = driver.rightBumper();
	 * // private Trigger alignLeftReef = driver.leftBumper();
	 * // private Trigger alignCenterReef = driver.a();
	 * 
	 * // more Game Status
	 * // private @Getter Trigger reefAlignment = new Trigger(
	 * // () -> alignRightReef.getAsBoolean() || alignLeftReef.getAsBoolean());
	 */

	private Trigger zeroDrivebase = driver.y();
	private Trigger robotOriented = driver.start();
	private Trigger driverStow = driver.x();
	private Trigger killElevator = driver.b();
	private Trigger driverOutake = driver.leftTrigger(0.5);
	private Trigger driverIntake = driver.rightTrigger(0.5);

	// Operator Controls

	// Preset Controls
	private Trigger feedPreset = operator.downDpad();
	private Trigger stowPreset = operator.start();
	private Trigger cleanL2Preset = operator.back();
	private Trigger scoreL2Preset = operator.rightDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();
	private Trigger ejectL4Preset = operator.y();
	private Trigger deplotIntake = operator.rightBumper();

	// Intake Controls
	private Trigger reverseFeed = operator.b();
	private Trigger stowIntake = operator.leftBumper();
	private Trigger manipFeed = operator.x();

	private Trigger intakeCoral = operator.leftTrigger(.5);
	private Trigger scoreCoral = operator.rightTrigger(.5);

	public RobotContainer() {
		// Path Planner reccomends that construction of their namedcommands happens
		// before anything else in robot container
		setCommandMappings();
		configureOperatorControls();
		configureDriverControls();
		configDriverFacingAngle();

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

	}

	public void subsystemPeriodic() {
		drivebase.periodic();
		intake.periodic();
		feeder.periodic();
		elevator.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		intakePivot.periodic();
		candle.periodic();
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);
	}

	private void configureDriverControls() {

		drivebase.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivebase.applyRequest(
						() -> drivebase.getDriverFOControl()
								.withVelocityX(deadzone(-driver.getLeftY())
										* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
								.withVelocityY(deadzone(-driver.getLeftX())
										* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
								.withRotationalRate(
										deadzone(-driver.getRightX())
												* DrivebaseConstants.MaxAngularRate.in(RadiansPerSecond)))
						.withName("Swerve Default Command"));

		robotOriented.whileTrue(drivebase.applyRequest(
				() -> drivebase.getDriverROControl()
						.withVelocityX(deadzone(-driver.getLeftY())
								* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
						.withVelocityY(deadzone(-driver.getLeftX())
								* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
						.withRotationalRate(
								deadzone(-driver.getRightX())
										* DrivebaseConstants.MaxAngularRate.in(RadiansPerSecond)))
				.withName("Swerve Robot Oriented"));

		// Align Reef Commands
		// alignLeftReef.onTrue(
		// new AlignReefCommand(false).withName("Align Left Reef"));
		// alignRightReef.onTrue(
		// new AlignReefCommand(true).withName("Align Right Reef"));
		// alignCenterReef.onTrue(
		// new AlignReefCommand().withName("Align Center Reef"));

		driverStow.onTrue(
				new SmartStowCommand(elevator, manipJoint, manipulator)
						.alongWith(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW))
						.withName("Driver Smart Stow"));

		killElevator.onTrue(manipJoint.killManipJoingCommand().alongWith(elevator.killElevatorCommand())
				.withName("Kill Elevator System"));

		zeroDrivebase.onTrue(new InstantCommand(() -> drivebase.resetGyro())
				.withName("Zero Drivebase"));

		driverIntake.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE).asProxy().withName("Driver Intake"));
		driverOutake.whileTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE).asProxy().withName("Driver Outake"));

	}

	private void configureOperatorControls() {

		// Intake Controls
		intakeCoral.whileTrue(new ParallelCommandGroup(
				intake.runIntakeCommand(IntakeModes.INTAKE),
				(manipulator.runManipulatorCommand(ManipulatorModes.FEED)))
						.withName("Intake System"));

		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.SCORE).asProxy()
				.withName("Score Coral"));

		manipFeed.whileTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.MANUAL)
						.withName("Manipulator Feed"));

		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator)
				.withName("Coral Outake"));

		// Pivot Controls
		deplotIntake.onTrue(
				intakePivot.runIntakePivotCommand(IntakePivotModes.DEPLOY)
						.withName("Deploy Intake"));

		stowIntake.onTrue(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW)
				.withName("Stow Intake"));

		// Elevator Controls
		stowPreset.onTrue(new ElevatorStowCommand(elevator, manipJoint)
				.withName("Elevator Stow"));

		feedPreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.withName("Feed Preset"));

		cleanL2Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL1Position, elevator, manipJoint)
						.withName("Clean L2 Preset"));

		scoreL2Preset.onTrue(new ElevatorStowCommand(elevator, manipJoint)
				.withName("Elevator L2 Preset"));

		scoreL3Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint)
						.withName("Elevator L3 Preset"));

		scoreL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint)
						.withName("Elevator L4 Preset"));

		ejectL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.EjectL4Position, elevator, manipJoint)
						.withName("Eject L4 Preset"));

	}

	private Command lockToAngleCommand(Angle redAngle, Angle blueAngle) {
		return drivebase.applyRequest(
				() -> drivebase.getFacingRequest()
						.withVelocityX(
								deadzone(-driver.getLeftY()) * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
						.withHeading(Field.isBlue() ? blueAngle : redAngle)
						.withVelocityY(
								deadzone(-driver.getLeftX()) * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond)))
				.until(() -> Math.abs(driver.getHID().getRawAxis(4)) > 0.15);
	}

	private void configDriverFacingAngle() {
		driver.upDpad().onTrue(new InstantCommand(() -> {
			drivebase.getFacingRequest().pid.reset();
		})).toggleOnTrue(lockToAngleCommand(Degrees.of(180), Degrees.of(180)));

		// driver.upRightDpad().onTrue(new InstantCommand(() -> {
		// drivebase.getFacingRequest().pid.reset();
		// })).toggleOnTrue(lockToAngleCommand(Degrees.of(30), Degrees.of(30)));

		driver.rightDpad().onTrue(new InstantCommand(() -> {
			drivebase.getFacingRequest().pid.reset();
		})).toggleOnTrue(lockToAngleCommand(Degrees.of(90), Degrees.of(90)));

		// driver.downRightDpad().onTrue(new InstantCommand(() -> {
		// drivebase.getFacingRequest().pid.reset();
		// })).toggleOnTrue(lockToAngleCommand(Degrees.of(60), Degrees.of(60)));

		driver.downDpad().onTrue(new InstantCommand(() -> {
			drivebase.getFacingRequest().pid.reset();
		})).toggleOnTrue(lockToAngleCommand(Degrees.of(0), Degrees.of(0)));

		// driver.downLeftDpad().onTrue(new InstantCommand(() -> {
		// drivebase.getFacingRequest().pid.reset();
		// })).toggleOnTrue(lockToAngleCommand(Degrees.of(120), Degrees.of(120)));

		driver.leftDpad().onTrue(new InstantCommand(() -> {
			drivebase.getFacingRequest().pid.reset();
		})).toggleOnTrue(lockToAngleCommand(Degrees.of(270), Degrees.of(270)));

		// driver.upLeftDpad().onTrue(new InstantCommand(() -> {
		// drivebase.getFacingRequest().pid.reset();
		// })).toggleOnTrue(lockToAngleCommand(Degrees.of(330), Degrees.of(330)));

	}

	public void onInitialize() {
		// Default Commands
		manipJoint.runManipJointCommand(ManipJointPositions.STOW);
		elevator.runOnce(() -> elevator.riseAboveFriction());
		intake.setDefaultCommand(intake.runIntakeCommand(IntakeModes.IDLE)
				.withName("Intake Default Command"));
		feeder.setDefaultCommand(feeder.runFeederCommand(FeederModes.IDLE)
				.withName("Feeder Default Command"));
		manipulator.setDefaultCommand(manipulator.runManipulatorCommand(ManipulatorModes.IDLE)
				.withName("Manipulator Default Command"));
		candle.setDefaultCommand(candle.changeCANdle(AnimationTypes.SPIRIT).withName("CANDle Default"));

		// Subsystem Status
		isIntaking.whileTrue(feeder.runFeederCommand(FeederModes.FEED).withName("Feeder Auto Control"));
		hasCoral.onTrue(candle.changeCANdle(AnimationTypes.Larson).withName("CANdle Coral").withTimeout(0.5));
		// LED Status
		isEndgame.whileTrue(candle.changeCANdle(AnimationTypes.STRESS_TIME).withName("LED Endgame"));
		isAutonEnabled.whileTrue(
				candle.changeCANdle((Field.isRed() ? AnimationTypes.BLINK_RED : AnimationTypes.BLINK_BLUE))
						.withName("LED Auton Alliance"));

		elevatorTarget
				.onTrue(candle.changeCANdle(AnimationTypes.ELEVATOR_GOAL).withTimeout(0.5).withName("CANDle Goal"));

		isAutonDisabled.whileTrue(
				candle.changeCANdle(AnimationTypes.Rainbow).ignoringDisable(true)
						.withName("Auton Disabled"));

	}

	/**
	 * Sets a Deazone
	 * Make a linear function with deadson at 0 and 1 at 1.
	 * Then need to have this work on both positive and negative.
	 * 
	 * @param num
	 * @return
	 */
	public double deadzone(double num) {
		if (Math.abs(num) > ControllerConstants.deadzone) {
			double w = 1.0 / (1.0 - ControllerConstants.deadzone);
			double b = w * ControllerConstants.deadzone;
			return (w * Math.abs(num) - b) * (num / Math.abs(num));
		} else {
			return 0;
		}
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setCommandMappings() {
		NamedCommands.registerCommand("EjectCoral",
				new EjectCoralCommand(intake, feeder, manipulator));
		NamedCommands.registerCommand("ElevatorFeed",
				new ElevatorFeedCommand(elevator, manipJoint));
		NamedCommands.registerCommand("L1Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL1Position, elevator, manipJoint));
		NamedCommands.registerCommand("L2Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint));
		NamedCommands.registerCommand("L3Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint));
		NamedCommands.registerCommand("L4Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint));
		NamedCommands.registerCommand("StowPreset",
				new ElevatorStowCommand(elevator, manipJoint).withTimeout(0.5));
		NamedCommands.registerCommand("SimpleScore",
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE));
		NamedCommands.registerCommand("ScoreL4",
				new ParallelCommandGroup(
						new ElevatorPresetCommand(ControllerConstants.EjectL4Position, elevator, manipJoint)
								.alongWith(manipulator.runManipulatorCommand(ManipulatorModes.SCORE).withTimeout(1))));

		// NamedCommands.registerCommand("AlignLeftReef", new AlignReefCommand(false));
		// NamedCommands.registerCommand("AlignRightReef", new AlignReefCommand(true));

	}
}
