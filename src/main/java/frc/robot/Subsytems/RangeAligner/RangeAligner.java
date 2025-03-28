package frc.robot.Subsytems.RangeAligner;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.RangeConstants;

public class RangeAligner extends SubsystemBase{
    public CANrange leftCaNrange;
    public CANrange rightCaNrange;

    public RangeAligner(CANrange leftCaNrange, CANrange rightCaNrange) {
        this.leftCaNrange = leftCaNrange;
        this.rightCaNrange = rightCaNrange;
        leftCaNrange.getConfigurator().apply(RangeConstants.proximityConfig);
        rightCaNrange.getConfigurator().apply(RangeConstants.proximityConfig);

    }

    public Distance Range(Boolean isRight){
        if(isRight){
            return rightCaNrange.getDistance().getValue();
        } else {
            return leftCaNrange.getDistance().getValue();
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("Right Dist", rightCaNrange.getDistance().getValue().abs(Inches));
        SmartDashboard.putNumber("Left Dist", leftCaNrange.getDistance().getValue().abs(Inches));
        SmartDashboard.putNumber("Right Detect", rightCaNrange.getIsDetected().getValueAsDouble());
        SmartDashboard.putNumber("Left Detect", leftCaNrange.getIsDetected().getValueAsDouble());
    
    }

    

}

