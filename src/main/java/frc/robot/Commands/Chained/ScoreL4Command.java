package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class ScoreL4Command extends SequentialCommandGroup {

    /**
     * If scoring L2/L3, auto stow
     * In cases when scoring L4, I dont wanna climb the reef
     * 
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public ScoreL4Command(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {
        addCommands(
            new ParallelCommandGroup(
                manipulator.runManipulatorCommand(ManipulatorModes.SLOWSCORE),
                elevator.runElevatorCommand(ElevatorPositions.RISE)
            )
        );
        

        addRequirements(elevator, manipJoint, manipulator);

    }

}
