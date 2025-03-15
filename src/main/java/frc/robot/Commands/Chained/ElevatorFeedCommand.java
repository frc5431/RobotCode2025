package frc.robot.Commands.Chained;

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

		switch (elevator.getPosition()) {
			// if higher than safeswing, dont get any higher
			case EJECET:
			case CORALL4:
			case SAFESWING:
			case CORALL3:	
			case FEED:
				addCommands(
						// manip runs to stow position only if the elevator is at the setpoint goal
						manipJoint.runManipJointCommand(ManipJointPositions.PREEFEED),
						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.prefeed,
								ManipJointConstants.tightError)),
						// since its sequential, this lowers once the manip is
						// when prev commands finish (instantaly since its RunCommands)
						// sets elevator to stow angle only if the manipulator is near the stow angle
						// this ensures that the manip doesnt hit anything while the elevator goes down
						elevator.runElevatorCommand(ElevatorPositions.FEED),
						manipJoint.runManipJointCommand(ManipJointPositions.FEED),

						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.feed,
								ManipJointConstants.tightError)));
				break;
			// if lower than stow, raise elevator first
			case CLEANL2:
			case CORALL2:
			case CORALL1:
			case STOW:
			default:
				addCommands(
						elevator.runElevatorCommand(ElevatorPositions.SAFESWING),
						new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.safeSwing,
								ElevatorConstants.error)),
						// manip runs to stow position only if the elevator is at the setpoint goal
						manipJoint.runManipJointCommand(ManipJointPositions.PREEFEED),
						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.prefeed,
								ManipJointConstants.error)),
								elevator.runElevatorCommand(ElevatorPositions.FEED),
						manipJoint.runManipJointCommand(ManipJointPositions.FEED),
						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.feed,
								ManipJointConstants.error)));
						// since its sequential, this lowers once the manip is
						// when prev commands finish (instantaly since its RunCommands)
						// sets elevator to stow angle only if the manipulator is near the stow angle
						// this ensures that the manip doesnt hit anything while the elevator goes down
				break;
		}

		addRequirements(elevator, manipJoint);
	}

}
