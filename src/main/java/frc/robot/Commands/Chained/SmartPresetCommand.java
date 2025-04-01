package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.PresetPosition;

public class SmartPresetCommand extends SequentialCommandGroup {

	public SmartPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {

		addCommands(
				new ConditionalCommand(
						new SequentialCommandGroup(elevator.riseElevatorCommand(),
								new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.rise,
										ElevatorConstants.error))),
						new PrintCommand("********************************\nRise Skipped\n***************************"),
						(() -> !(elevator.isSwingSafe() && manipJoint.isSwingSafe()))),
				// rises to RISE so manip can safely move

				// manip runs to stow position only if the elevator is at the setpoint goal

				manipJoint.runManipJointCommand(position.getJointMode()),

				new ConditionalCommand(
					new SequentialCommandGroup(
						manipJoint.runManipJointCommand(position.getJointMode()),
						// This should be Angle Angle not ManipJointPositions Anlge
					new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(position.getJointMode().position,
					ManipJointConstants.error))),
						new PrintCommand("*********************\nSwing Out Skipped\n************************"),
						(() -> !(manipJoint.isSwingSafe()))),
			
				// since its sequential, this lowers once the manip is
				elevator.runElevatorCommand(position.getElevatorMode()));

		addRequirements(elevator, manipJoint);

	}

}
