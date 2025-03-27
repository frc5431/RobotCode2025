package frc.robot.Commands.Chained;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.RangeAligner.RangeAligner;

public class SimpleAlignToReefCommand extends SequentialCommandGroup{

    private Drivebase drivebase;
    private RangeAligner rangeAligner;

    public SimpleAlignToReefCommand(Drivebase drivebase, RangeAligner rangeAligner) {
        this.drivebase = drivebase;
        this.rangeAligner = rangeAligner;

        // addCommands(
        //     drivebase.driveAuton(new ChassisSpeeds(0, -0.5, 0)),
        //     new WaitUntilCommand()
        // );

        addRequirements(drivebase);
    }
    
}