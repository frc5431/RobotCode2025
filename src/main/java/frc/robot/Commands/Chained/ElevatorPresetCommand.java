package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends SequentialCommandGroup {

	public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {

		switch (elevator.getPosition()) {
			case CORAL_L1:
			case STOW:
			case CORAL_L2:
			case CLEAN_L2:
			case CLEAN_L3:
			case CORAL_L3:
			case CORAL_L4:
			case EJECT:
				addCommands(manipJoint.runManipJointCommand(position.getJointMode()).alongWith(
						elevator.runElevatorCommand(position.getElevatorMode())));
				break;
			case FEED:
			case SAFESWING:
			default:
				addCommands(
				new ElevatorStowCommand( elevator, manipJoint),
				manipJoint.runManipJointCommand(position.getJointMode()).alongWith(
					elevator.runElevatorCommand(position.getElevatorMode())));
				break;

		}
		addRequirements(elevator, manipJoint);
	}

}
