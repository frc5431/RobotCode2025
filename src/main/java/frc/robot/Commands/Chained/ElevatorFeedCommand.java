package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorFeedCommand extends SequentialCommandGroup {

	/**
	 * @param elevator
	 * @param manipJoint
	 */
	public ElevatorFeedCommand(Elevator elevator, ManipJoint manipJoint) {

		addCommands(
				new ConditionalCommand(
						new PrintCommand("Rise Skipped"),
						elevator.runElevatorCommand(ElevatorPositions.SAFESWING),
						() -> elevator.isSwingSafe()),
				new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.safeSwing,
						ElevatorConstants.error)),
				elevator.runElevatorCommand(ElevatorPositions.FEED),
				manipJoint.runManipJointCommand(ManipJointPositions.FEED),
				new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.feed,
						ManipJointConstants.error)));

		addRequirements(elevator, manipJoint);
	}

}
