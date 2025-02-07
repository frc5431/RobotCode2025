package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;

public class Field {

    public static final double netAimAngle = 0;
    public static final double processorAimAngle = 0;
    public static final double intakeAlgeaAngle = 0;



    /**
     * @return returns true if drivestation is on blue alliance
     */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    
    /**
     * @return returns true if drivestation is on red alliance
     */
    public static boolean isRed() {
        return !isBlue();
    }



    
}
