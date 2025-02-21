// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.IntakeCoralCommand;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Field;
import frc.robot.Util.RobotMechanism;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;
import frc.robot.Util.Constants.CleanerConstants.CleanerModes;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.GamePieceConstants.GamePieceStates;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointStates;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
import frc.team5431.titan.core.joysticks.TitanController;
import lombok.Getter;

public class RobotContainer {

	private final Systems systems = new Systems();
	private final RobotMechanism robotMechanism = new RobotMechanism();

	private final Intake intake = systems.getIntake();
	private final IntakePivot intakePivot = systems.getIntakePivot();
	private final Feeder feeder = systems.getFeeder();
	private final Cleaner cleaner = systems.getCleaner();
	private final Elevator elevator = systems.getElevator();
	private final CleanPivot cleanPivot = systems.getCleanPivot();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();
	private final Climber climber = systems.getClimber();
	private @Getter final TitanCANdle candle = systems.getCandle();

	private TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
	private TitanController operator = new TitanController(ControllerConstants.operatorPort,
			ControllerConstants.deadzone);

	private GamePieceStates gamePieceStatus = GamePieceStates.NONE;

	// Triggers

	// Automated Triggers

	// Game Status
	private @Getter Trigger hasAlgae = new Trigger(() -> gamePieceStatus == GamePieceStates.ALGAE);
	private @Getter Trigger hasCoral = new Trigger(() -> gamePieceStatus == GamePieceStates.CORAL);
	private @Getter Trigger isEndgame = new Trigger(() -> DriverStation.getMatchTime() <= 20 && DriverStation.isTeleop());
	private @Getter Trigger isAutonEnabled = new Trigger(() -> DriverStation.isAutonomousEnabled());

	// Subsystem Triggers
	private @Getter Trigger isIntaking = new Trigger(
			() -> intake.getMode() == IntakeModes.INTAKE || intake.getMode() == IntakeModes.FEED);

	// LED Triggers

	// Driver Controls

	// Climber Controls
	private Trigger climberOut = driver.leftTrigger(ControllerConstants.triggerThreshold);
	private Trigger climberClimb = driver.rightTrigger(ControllerConstants.triggerThreshold);

	// Operator Controls
	private Trigger cycleThroughWrist = operator.start();

	// Preset Controls
	private Trigger intakePreset = operator.leftBumper();
	private Trigger processorPreset = operator.back();
	private Trigger feedPreset = operator.rightDpad();
	private Trigger scoreL2Preset = operator.downDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();

	// Cleaner Controls
	private Trigger intakeAlgae = operator.b();
	private Trigger outtakeAlgae = operator.x();

	// Intake Controls
	private Trigger intakeCoral = operator.a();
	private Trigger scoreCoral = operator.y();
	private Trigger reverseFeed = operator.rightStick();

	public RobotContainer() {
		configureBindings();
	}

	public void subsystemPeriodic() {
		intake.periodic();
		feeder.periodic();
		cleaner.periodic();
		climber.periodic();
		elevator.periodic();
		cleanPivot.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		intakePivot.periodic();
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);

		gamePieceStatus = (manipulator.getBeambreakStatus()) ? GamePieceStates.CORAL : GamePieceStates.NONE;
		manipulator.setState((manipulator.getBeambreakStatus()) ? ManipulatorStates.LOCKED : ManipulatorStates.EMPTY);

	}

	private void configureDriverControls() {

		// Climber Controls
		climberOut.onTrue(climber.runClimberCommand(ClimberModes.ALIGN));
		climberClimb.whileTrue(climber.runClimberCommand(ClimberConstants.climbVelocity));

	}

	private void configureOperatorControls() {
		//Mechanism Controls
		cycleThroughWrist.onTrue(robotMechanism.ChangeWristAngleCommand(40.00));

		// Elevator Controls
		intakePreset.onTrue(
				new IntakeCoralCommand(intake, intakePivot, manipulator, elevator, manipJoint)
						.withName("Intake Coral Preset"));

		processorPreset.onTrue(
				new ElevatorStowCommand(elevator, manipJoint)
						.withName("Elevator Algae Intake"));

		feedPreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.withName("Elevator Feed Positon"));

		scoreL2Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint)
						.withName("Elevator L2 Preset"));

		scoreL3Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint)
						.withName("Elevator L3 Preset"));

		scoreL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint)
						.withName("Elevator L4 Preset"));

		// Intake Controls
		intakeCoral.whileTrue(new ParallelCommandGroup(intake.runIntakeCommand(IntakeModes.INTAKE),
				(manipulator.runManipulatorCommand(ManipulatorModes.FEED))).withName("Run Intake System"));
		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.REVERSE).withName("Score Coral"));
		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator).withName("Coral Outake"));

		// Cleaner Controls
		intakeAlgae.whileTrue(cleaner.runCleanerCommand(CleanerModes.INTAKE).withName("Intake Algae"));
		outtakeAlgae.whileTrue(cleaner.runCleanerCommand(CleanerModes.OUTTAKE).withName("Outtake Algae"));

	}

	public void configureBindings() {

		configureOperatorControls();
		configureDriverControls();
	}

	public void onInitialize() {
		// Default Commands
		candle.setDefaultCommand(candle.titanCommand().withName("LED Default Command"));
		climber.runClimberCommand(ClimberModes.STOW);

		// Subsystem Status
		isIntaking.whileTrue(feeder.runFeederCommand(FeederModes.FEED).withName("Feeder Auto Control"));

		// LED Status
		isEndgame.whileTrue(candle.changeAnimationCommand(AnimationTypes.STRESS_TIME).withName("LED Endgame"));
		isAutonEnabled.whileTrue(
				candle.changeAnimationCommand((Field.isRed() ? AnimationTypes.BLINK_RED : AnimationTypes.BLINK_BLUE))
						.withName("LED Auton Alliance"));
		hasCoral.whileTrue(candle.changeAnimationCommand(AnimationTypes.CORAL).withTimeout(1).withName("LED Coral"));
		hasAlgae.whileTrue(candle.changeAnimationCommand(AnimationTypes.ALGAE));
		hasAlgae.and(hasCoral).onTrue(candle.changeAnimationCommand(AnimationTypes.BOTH));

	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

}
