package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class ScoreL4Command extends ParallelRaceGroup {

    /**
     * We got that 2056 manip
     * 
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public ScoreL4Command(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {
        addCommands(
                manipulator.runManipulatorCommand(ManipulatorModes.SCORE),
                new WaitCommand(0.3).andThen(
                elevator.runElevatorCommand(ElevatorPositions.RISE))

        );

        addRequirements(elevator, manipJoint, manipulator);

    }

}
