package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.led.CANdle;

import frc.robot.Subsytems.CANdle.CANdleSystem;
import frc.robot.Subsytems.CANdle.CANdleSystem.AnimationTypes;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class SmartScoreCommand extends SequentialCommandGroup {

	/**
	 * We got that 2056 manip
	 * 
	 * @param elevator
	 * @param manipJoint
	 * @param manipulator
	 */
	public SmartScoreCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator, CANdleSystem candle) {
		addCommands(
			candle.changeCANdle(AnimationTypes.BLINK_BLUE).alongWith(
				//L4 Sequence
				new ConditionalCommand(
						new SequentialCommandGroup(
								manipJoint.runManipJointCommand(ManipJointPositions.PROCESSOR),
								new WaitUntilCommand(
										() -> manipJoint.getPositionSetpointGoal(
												ManipJointConstants.slamL4,
												ManipJointConstants.error)
												//is it lte or gte? my mind is not minding rn
												|| ManipJointConstants.slamL4.lte(
														Rotations.of(manipJoint.getMotorPosition()))),
								manipulator.setManipulatorCommand(ManipulatorModes.SLOWSCORE),
								elevator.runElevatorCommand(ElevatorPositions.CORAL_L2),
								new WaitCommand(0.2),
								manipJoint.runManipJointCommand(ManipJointPositions.CORAL_L2),
								manipulator.setManipulatorCommand(ManipulatorModes.IDLE)),

						//L3 Sequence
						new ConditionalCommand(
								new SequentialCommandGroup(
										manipJoint.runManipJointCommand(ManipJointPositions.SLAM_L3),
										new WaitCommand(0.2),
										manipulator.setManipulatorCommand(ManipulatorModes.SLOWSCORE),
										new WaitCommand(0.2).andThen(
												elevator.runElevatorCommand(ElevatorPositions.SLAM_L3)),
										new WaitCommand(4)),

								//L2 Sequence
								new ConditionalCommand(
										new SequentialCommandGroup(
												manipJoint.runManipJointCommand(ManipJointPositions.SLAM_L2),
												new WaitCommand(0.2),
												manipulator.runManipulatorCommand(ManipulatorModes.SLOWSCORE)),
										new PrintCommand("Elevator Position Not Scoring"),
										(() -> elevator.getPosition() == ElevatorPositions.CORAL_L2)),
								(() -> elevator.getPosition() == ElevatorPositions.CORAL_L3)),
						(() -> elevator.getPosition() == ElevatorPositions.CORAL_L4))),
				candle.changeCANdle(AnimationTypes.SCORE)
		);

		addRequirements(elevator, manipJoint, manipulator);

	}

}
