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
    
    public PickCoralCommand(Elevator elevator, Manipulator manipulator){
        addCommands(
            elevator.runElevatorCommand(ElevatorPositions.PICKUP),
            manipulator.setManipulatorCommand(ManipulatorModes.FEED),
            new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorPositions.PICKUP.rotation, ElevatorConstants.tightError)),
            manipulator.setManipulatorCommand(ManipulatorModes.FEED),
            new WaitCommand(0.5),
            elevator.runElevatorCommand(ElevatorPositions.FEED),
            manipulator.setManipulatorCommand(ManipulatorModes.IDLE)
        );
        addRequirements(elevator, manipulator);    
    }
}
