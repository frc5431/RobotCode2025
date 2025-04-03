package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorFeedCommand extends SequentialCommandGroup {

	/**
	 * @param elevator
	 * @param manipJoint
	 */
	public ElevatorFeedCommand(Elevator elevator, ManipJoint manipJoint) {

		addCommands(
				new ConditionalCommand(
					new PrintCommand("********************************\nRise Skipped\n***************************"),
					new SequentialCommandGroup(
						elevator.runElevatorCommand(ElevatorPositions.SAFESWING),
						new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.safeSwing,
						ElevatorConstants.error))),
						() -> elevator.isSwingSafe()),
	
				elevator.runElevatorCommand(ElevatorPositions.FEED),
				manipJoint.runManipJointCommand(ManipJointPositions.PREEFEED),
				new WaitUntilCommand(() -> Math.abs(manipJoint.getMotorPosition()) >= 13.0),
				manipJoint.runManipJointCommand(ManipJointPositions.FEED)

				);

		addRequirements(elevator, manipJoint);
	}

}
