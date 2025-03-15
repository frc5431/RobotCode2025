package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends SequentialCommandGroup {

	public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {

		switch (elevator.getPosition()) {
			case CORALL1:
			case STOW:
			case CORALL2:
			case CORALL3:
			case CORALL4:
			case EJECET:
				addCommands(manipJoint.runManipJointCommand(position.getJointMode()).alongWith(
						elevator.runElevatorCommand(position.getElevatorMode())));
				break;
			case FEED:
			case SAFESWING:
			default:
				addCommands(
				new ElevatorStowCommand(true, elevator, manipJoint),
				manipJoint.runManipJointCommand(position.getJointMode()).alongWith(
					elevator.runElevatorCommand(position.getElevatorMode())));
				break;

		}
		addRequirements(elevator, manipJoint);
	}

}
