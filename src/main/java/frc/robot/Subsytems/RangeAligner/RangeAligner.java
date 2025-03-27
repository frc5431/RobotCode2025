package frc.robot.Subsytems.RangeAligner;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeAligner extends SubsystemBase{
    public CANrange leftCaNrange;
    public CANrange rightCaNrange;

    public RangeAligner(CANrange leftCaNrange, CANrange rightCaNrange) {
        this.leftCaNrange = leftCaNrange;
        this.rightCaNrange = rightCaNrange;
    }

    public Distance Range(Boolean isRight){
        if(isRight){
            return rightCaNrange.getDistance().getValue();
        } else{
            return leftCaNrange.getDistance().getValue();
        }
    }

}

