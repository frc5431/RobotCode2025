package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class SmartScoreCommand extends SequentialCommandGroup {

    /**
     * If scoring L2/L3, auto stow
     * In cases when scoring L4, I dont wanna climb the reef
     * 
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public SmartScoreCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {
        switch (manipJoint.getMode()) {
            case CORAL_L2:
            case CORAL_L3:
                addCommands(
                        manipulator.runManipulatorCommand(ManipulatorModes.SCORE),
                        new WaitUntilCommand(() -> !manipulator.hasCoral()),
                        new ElevatorStowCommand(elevator, manipJoint)
                                .alongWith(manipulator.runManipulatorCommand(ManipulatorModes.SCORE).withTimeout(0.3)));
                break;
            default:
                addCommands(

                        manipulator.runManipulatorCommand(ManipulatorModes.SCORE));
                break;
        }

        addRequirements(elevator, manipJoint, manipulator);

    }

}
