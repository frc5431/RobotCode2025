package frc.robot.Subsytems.Limelight;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.team5431.titan.core.vision.Limelight;

public class VisionUtil {
    



    public static class VisionCommandConfig {
        
        public Limelight limelight;
        public int pipeline;
        public double p;
        public double maxOutput;
        public Distance distanceTolerance;
        public Angle angleTolerance;
        
        public Distance v_distanceSetpoint;
        public Distance h_distanceSetpoint;

        public void setLimeLight(Limelight limelight, int pipeline) {
            this.limelight = limelight;
            this.pipeline = pipeline;
        }

        public void setP(double p) {
            this.p = p;
        }

        public void setMaximumOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void setDistanceTolerance(Distance tolerance) {
            this.distanceTolerance = tolerance;
        }

        public void setAngleTolerance(Angle tolerance) {
            this.angleTolerance = tolerance;
        }

        public void setVerticalDistanceSetpoint(Distance tolerance) {
            this.v_distanceSetpoint = tolerance;
        }

        public void setHorizontalDistanceSetpoint(Distance tolerance) {
            this.h_distanceSetpoint = tolerance;
        }





    }

}
