package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class ElevatorLebron extends SequentialCommandGroup {

    /**
     * @param elevator
     * @param manipulator
     * @param manipJoint
     * This does cool stuff to shoot in barge
     */
    public ElevatorLebron(Elevator elevator, Manipulator manipulator, ManipJoint manipJoint) {
        addCommands(
            // elevator.runElevatorCommand(ElevatorPositions.CORAL_L3),
            manipJoint.runManipJointCommand(ManipJointPositions.STRAIGHT_UP),

            // new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointPositions.STRAIGHT_UP.position, ManipJointConstants.error)),
            elevator.runElevatorCommand(ElevatorPositions.CORAL_L4),
            
            new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorPositions.LEBRON_SHOOT.rotation, ElevatorConstants.error)),
            manipulator.runManipulatorCommand(ManipulatorModes.SCORE)
        );
        addRequirements(elevator, manipulator, manipJoint);        
    }

}
