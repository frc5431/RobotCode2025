// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Field;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Chained.AlignToReef.FieldBranchSide;
import frc.robot.Commands.Chained.DriveToPoseCommand;
import frc.robot.Subsytems.PoseEstimator.PoseEstimator;
import frc.robot.Commands.Chained.AlignToReef;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorLebron;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.PickCoralCommand;
import frc.robot.Commands.Chained.SmartScoreCommand;
import frc.robot.Commands.Chained.SmartPresetCommand;

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
	private final Feeder feeder = systems.getFeeder();
	private final Elevator elevator = systems.getElevator();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();
	private final CANdleSystem candle = Systems.getTitanCANdle();
	private final Drivebase drivebase = Systems.getDrivebase();
	private final AprilTagFieldLayout layout = Systems.getLayout();
	private final SendableChooser<Command> autoChooser;

	private final PoseEstimator poseEstimator = Systems.getEstimator();

	private TitanController driver = Systems.getDriver();
	private TitanController operator = Systems.getOperator();
	private AlignToReef alignToReefCommandFactory = new AlignToReef(drivebase, layout);


	private enum AutoFilters {
		Comp, TEST, States, NONE
	}

	AutoFilters autoFilter = AutoFilters.NONE;
	// Triggers

	// Automated Triggers

	// Game Status
	private @Getter Trigger isEndgame = new Trigger(
			() -> DriverStation.getMatchTime() <= 5);
	private @Getter Trigger isAutonEnabled = new Trigger(() -> DriverStation.isAutonomousEnabled());
	private @Getter Trigger isAutonDisabled = new Trigger(
			() -> DriverStation.isAutonomous() && DriverStation.isDisabled());
	private Trigger elevatorTarget = new Trigger(
			() -> elevator.getPositionSetpointGoal(elevator.getPosition().rotation, ElevatorConstants.error)
					&& manipJoint.getPositionSetpointGoal(manipJoint.getMode().position,
							ManipJointConstants.error));

	// Subsystem Triggers
	private @Getter Trigger isIntaking = new Trigger(
			() -> intake.getMode() == IntakeModes.INTAKE || intake.getMode() == IntakeModes.FEED);

	private @Getter Trigger isScoring = new Trigger(
			() -> manipulator.getMode() == ManipulatorModes.SCORE
					|| manipulator.getMode() == ManipulatorModes.SLOWSCORE);

	// Subsystem Triggers
	private @Getter Trigger hasCoral = new Trigger(
			() -> manipulator.hasCoral() && manipulator.getMode() == ManipulatorModes.SCORE
					|| manipulator.getMode() == ManipulatorModes.SLOWSCORE);

	private @Getter Trigger manipJointManual = new Trigger(
			() -> (Math.abs(operator.getLeftY()) >= ControllerConstants.deadzone));

	// LED Triggers

	// Driver Controls
	private Trigger alignRightReef = driver.rightBumper();
	private Trigger alignLeftReef = driver.leftBumper();
	private Trigger alignCenterReef = driver.a();

	// more Game Status
	// private @Getter Trigger reefAlignment = new Trigger(
	// () -> alignRightReef.getAsBoolean() || alignLeftReef.getAsBoolean());

	private Trigger zeroDrivebase = driver.y();
	private Trigger robotOriented = driver.start();
	private Trigger driverStow = driver.x();
	private Trigger driverOutake = driver.leftTrigger(0.5);
	private Trigger driverIntake = driver.rightTrigger(0.5);

	// Operator Controls
	private Trigger lebronShot = operator.leftBumper();
	private Trigger pickCoral = operator.x();

	// Preset Controls
	private Trigger feedPreset = operator.downDpad();
	private Trigger cleanL2Preset = operator.start();
	private Trigger cleanL3Preset = operator.back();

	private Trigger scoreL2Preset = operator.rightDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();
	private Trigger processorPreset = operator.y();

	// Intake Controls
	private Trigger groundAlgae = operator.b();
	private Trigger smartScore = operator.rightBumper();

	private Trigger intakeAlgea = operator.leftTrigger(.5);
	private Trigger scoreCoral = operator.rightTrigger(.5);

	public RobotContainer() {
		// Path Planner reccomends that construction of their namedcommands happens
		// before anything else in robot container
		setCommandMappings();
		configureOperatorControls();
		configureDriverControls();
		configDriverFacingAngle();

		// TODO SWAP
		poseEstimator.setAlliance(Field.isBlue() ? Alliance.Blue : Alliance.Red);

		autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
				(stream) -> autoFilter != AutoFilters.NONE
						? stream.filter(auto -> auto.getName().startsWith(autoFilter.name()))
						: stream);

		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putBoolean("Alliance", Field.isBlue());

	}

	public void subsystemPeriodic() {
		drivebase.periodic();
		elevator.periodic();
		manipJoint.periodic();
		candle.periodic();
		poseEstimator.periodic();
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putNumber("Operator Left Y", -operator.getLeftY());
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);
	}

	private void configureDriverControls() {

		drivebase.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivebase.applyRequest(
						() -> drivebase.getDriverFOControl()
								.withVelocityX(drivebase.getDriverFOCInverter() * deadzone(-driver.getLeftY())
										* SwerveConstants.kSpeedAt12Volts
												.in(MetersPerSecond))
								.withVelocityY(drivebase.getDriverFOCInverter() * deadzone(-driver.getLeftX())
										* SwerveConstants.kSpeedAt12Volts
												.in(MetersPerSecond))
								.withRotationalRate(
										deadzone(-driver.getRightX())
												* DrivebaseConstants.MaxAngularRate
														.in(RadiansPerSecond)))
						.withName("Swerve Default Command"));

		robotOriented.whileTrue(drivebase.applyRequest(
				() -> drivebase.getDriverROControl()
						.withVelocityX(deadzone(-driver.getLeftY())
								* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
						.withVelocityY(deadzone(-driver.getLeftX())
								* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
						.withRotationalRate(
								deadzone(-driver.getRightX())
										* DrivebaseConstants.MaxAngularRate
												.in(RadiansPerSecond)))
				.withName("Swerve Robot Oriented"));

		// Align Reef Commands
		// todo: new commands!
		alignLeftReef.onTrue(
				alignToReefCommandFactory.generateCommand(FieldBranchSide.LEFT)
						.withName("Align Left Branch"));

		alignRightReef.onTrue(
				alignToReefCommandFactory.generateCommand(FieldBranchSide.RIGHT)
						.withName("Align Right Branch"));

		// alignCenterReef.onTrue(
		// 		alignToReefCommandFactory.generateCommand(FieldBranchSide.MIDDLE)
		// 				.withName("Align Right Branch"));
		alignCenterReef.onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), alignToReefCommandFactory.getClosestBranch(FieldBranchSide.MIDDLE, drivebase), 2).withName("Running Path"));

		zeroDrivebase.onTrue(new InstantCommand(() -> drivebase.resetGyro())
				.alongWith(new InstantCommand(() -> Systems.getEstimator().resetRotablion()))
				.withName("Zero Drivebase"));

		driverIntake.whileTrue(new ParallelCommandGroup(
				intake.runIntakeCommand(IntakeModes.INTAKE),
				feeder.runFeederCommand(FeederModes.FEED)).asProxy()
						.withName("Driver Intake"));

		driverOutake.whileTrue(new ParallelCommandGroup(
				intake.runIntakeCommand(IntakeModes.OUTTAKE),
				feeder.runFeederCommand(FeederModes.REVERSE)).asProxy()
						.withName("Driver Intake"));

	}

	private void configureOperatorControls() {

		// Intake Controls
		intakeAlgea.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.FEED)
				.withName("Intake Algea"));

		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.SCORE).asProxy()
				.withName("Score Coral"));

		groundAlgae.onTrue(
				new SmartPresetCommand(ControllerConstants.GroundAlgeaPosition, elevator, manipJoint)
						.withName("Ground Algea"));
		// Pivot Controls - changed for shot
		// lebronShot.onTrue(
		// intakePivot.runIntakePivotCommand(IntakePivotModes.DEPLOY)
		// .withName("Deploy Intake"));

		feedPreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.withName("Feed Preset"));

		cleanL2Preset.onTrue(
				new SmartPresetCommand(ControllerConstants.CleanL2Position, elevator, manipJoint)
						.withName("Clean L2 Preset"));

		cleanL3Preset.onTrue(
				new SmartPresetCommand(ControllerConstants.CleanL3Position, elevator, manipJoint)
						.withName("Clean L3 Preset"));

		scoreL2Preset.onTrue(new SmartPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint)
				.withName("Elevator L2 Preset"));

		scoreL3Preset.onTrue(
				new SmartPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint)
						.withName("Elevator L3 Preset"));

		scoreL4Preset.onTrue(
				new SmartPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint)
						.withName("Elevator L4 Preset"));

		smartScore.onTrue(
				new SmartScoreCommand(elevator, manipJoint, manipulator, candle)
						.withName("Smart Score Command"));
		
		processorPreset.onTrue(
				new SmartPresetCommand(ControllerConstants.ScoreProcessorPosition, elevator, manipJoint)
						.withName("Processor Preset"));

		operator.rightStick().whileTrue(manipJoint.runVoltageCommand(-0.3));

		lebronShot.onTrue(
				new ElevatorLebron(elevator, manipulator, manipJoint)
						.withName("Take The Shot"));

		pickCoral.onTrue(
				new PickCoralCommand(elevator, manipJoint, manipulator)
						.withName("Pick Coarl"));

		manipJointManual.whileTrue(manipJoint.runVoltageCommand(-operator.getLeftY() * 2));

	}

	private Command lockToAngleCommand(Angle redAngle, Angle blueAngle) {
		return drivebase.applyRequest(
				() -> drivebase.getFacingRequest()
						.withVelocityX(
								deadzone(-driver.getLeftY())
										* SwerveConstants.kSpeedAt12Volts
												.in(MetersPerSecond))
						.withHeading(Field.isBlue() ? blueAngle : redAngle)
						.withVelocityY(
								deadzone(-driver.getLeftX())
										* SwerveConstants.kSpeedAt12Volts
												.in(MetersPerSecond)))
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
		manipulator.setDefaultCommand(manipulator.smartStallCommand()
				.withName("Manipulator Default Command"));
		candle.setDefaultCommand(candle.changeCANdle(AnimationTypes.SPIRIT).withName("CANDle Default Command"));
		// Subsystem Status
		isScoring.whileTrue(candle.changeCANdle(AnimationTypes.SCORE).withName("CANdle Score"));
		// LED Status
		isEndgame.whileTrue(candle.changeCANdle(AnimationTypes.STRESS_TIME).withName("LED Endgame"));
		isAutonEnabled.whileTrue(
				candle.changeCANdle(
						(Field.isRed() ? AnimationTypes.BLINK_RED : AnimationTypes.BLINK_BLUE))
						.withName("LED Auton Alliance"));

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
		NamedCommands.registerCommand("L2Preset",
				new SmartPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint));
		NamedCommands.registerCommand("L3Preset",
				new SmartPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint));
		NamedCommands.registerCommand("L4Preset",
				new SmartPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint));
		NamedCommands.registerCommand("LoliPreset",
				new SmartPresetCommand(ControllerConstants.LolipopPosition, elevator, manipJoint));
		NamedCommands.registerCommand("StowPreset",
				new SmartPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint));
		NamedCommands.registerCommand("IntakeLolipop",
				manipulator.runManipulatorCommand(ManipulatorModes.FEED));
				NamedCommands.registerCommand("StopManip",
				manipulator.runManipulatorCommand(ManipulatorModes.IDLE));
		NamedCommands.registerCommand("SimpleScore",
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE));
		NamedCommands.registerCommand("SmartScore",
				new SmartScoreCommand(elevator, manipJoint, manipulator, candle));
		NamedCommands.registerCommand("AlignRightReef",
				alignToReefCommandFactory.generateCommand(FieldBranchSide.RIGHT));
		NamedCommands.registerCommand("AlignLeftReef",
				alignToReefCommandFactory.generateCommand(FieldBranchSide.LEFT));

	}
}
