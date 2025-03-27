package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.PresetPosition;

public class SmartPresetCommand extends SequentialCommandGroup {

    public SmartPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {

            addCommands(
                	// rises to RISE so manip can safely move
				elevator.runElevatorCommand(ElevatorPositions.RISE),
				new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.rise,
						ElevatorConstants.error)),
				// manip runs to stow position only if the elevator is at the setpoint goal

				manipJoint.runManipJointCommand(position.getJointMode()),
				new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.stow,
						ManipJointConstants.error)),
				// since its sequential, this lowers once the manip is
				elevator.runElevatorCommand(position.getElevatorMode()));

        addRequirements(elevator, manipJoint);
    
    }

}
