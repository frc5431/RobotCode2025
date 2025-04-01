package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class EjectCoralCommand extends ParallelCommandGroup {

    /**
     * @param intake
     * @param feeder
     * @param manipulator
     */
    public EjectCoralCommand(Intake intake, Feeder feeder, Manipulator manipulator) {
        addCommands(
                intake.runIntakeCommand(IntakeModes.OUTTAKE),
                feeder.runFeederCommand(FeederModes.REVERSE));
        addRequirements(intake, feeder, manipulator);

    }

}
