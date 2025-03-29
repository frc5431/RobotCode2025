package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class PickCoralCommand extends SequentialCommandGroup {
    
    public PickCoralCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator){
        addCommands(
            manipJoint.runManipJointCommand(ManipJointPositions.FEED),
            new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.feed,
            ManipJointConstants.error)),
            manipulator.setManipulatorCommand(ManipulatorModes.FEED),
            elevator.runElevatorCommand(ElevatorPositions.PICKUP),
            new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorPositions.PICKUP.rotation, ElevatorConstants.superTightError)),
            new WaitCommand(0.4),
            elevator.runElevatorCommand(ElevatorPositions.FEED)
        );
        addRequirements(elevator, manipulator);    
    }
}
