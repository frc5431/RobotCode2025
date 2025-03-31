package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorStowCommand extends SequentialCommandGroup {

	public ElevatorStowCommand(Elevator elevator, ManipJoint manipJoint) {

		addCommands(
				// rises to l4 so manip can safely move
				elevator.runElevatorCommand(ElevatorPositions.RISE),
				new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.rise,
						ElevatorConstants.error)),
				// manip runs to stow position only if the elevator is at the setpoint goal

				manipJoint.runManipJointCommand(ManipJointPositions.STOW),
				new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.processorAngle,
						ManipJointConstants.error)),
				// since its sequential, this lowers once the manip is
				elevator.runElevatorCommand(ElevatorPositions.STOW));

		addRequirements(elevator, manipJoint);
	}

}
