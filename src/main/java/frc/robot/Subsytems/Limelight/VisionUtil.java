package frc.robot.Subsytems.Limelight;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper.PhysicalConfig;
import frc.robot.Util.Constants.VisionConstants;
import frc.team5431.titan.core.vision.Limelight;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper;

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

    public static final class VisionConfig {
        /* Limelight Configuration */

        public static final String LEFT_LL = VisionConstants.cameraName;
        public static final PhysicalConfig centerConfig = new PhysicalConfig()
                .withTranslation(VisionConstants.centLLForwardOffset.in(Units.Meters), VisionConstants.centLLRightOffset.in(Units.Meters), VisionConstants.centLLUpOffset.in(Units.Meters))
                .withRotation(VisionConstants.centLLRollOffset.in(Units.Degrees), VisionConstants.centLLPitchOffset.in(Units.Degrees), VisionConstants.centLLYawOffset.in(Units.Degrees));

        // public static final String RIGHT_LL = "limelight-right";
        // public static final PhysicalConfig RIGHT_CONFIG = new PhysicalConfig()
        //         .withTranslation(VisionConstants.rightLLForwardOffset.in(Units.Meters), VisionConstants.rightLLRightOffset.in(Units.Meters), VisionConstants.rightLLUpOffset.in(Units.Meters))
        //         .withRotation(VisionConstants.rightLLRollOffset.in(Units.Degrees), VisionConstants.rightLLPitchOffset.in(Units.Degrees), VisionConstants.rightLLYawOffset.in(Units.Degrees));


        // Increase these numbers to trust global measurements from vision less.
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix = VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y,
                VISION_STD_DEV_THETA);

        /* Vision Command Configs */

    }

    public static class LimelightLogger {
        @SuppressWarnings("unused")
        private String name;
        public VisionHelper visionHelper;

        public LimelightLogger(String name, VisionHelper visionHelper) {
            this.visionHelper = visionHelper;
            this.name = name;
        }

        @AutoLogOutput(key = "Vision/{name}/ConnectionStatus")
        public boolean getCameraConnection() {
            return visionHelper.isCameraConnected();
        }

        @AutoLogOutput(key = "Vision/{name}/Integrating")
        public boolean getIntegratingStatus() {
            return visionHelper.isIntegrating;
        }

        @AutoLogOutput(key = "Vision/{name}/LogStatus")
        public String getLogStatus() {
            return visionHelper.logStatus;
        }

        @AutoLogOutput(key = "Vision/{name}/TagStatus")
        public String getTagStatus() {
            return visionHelper.tagStatus;
        }

        @AutoLogOutput(key = "Vision/{name}/Pose")
        public Pose2d getPose() {
            return visionHelper.getRawPose3d().toPose2d();
        }

        @AutoLogOutput(key = "Vision/{name}/MegaPose")
        public Pose2d getMegaPose() {
            return visionHelper.getMegaPose2d();
        }

        @AutoLogOutput(key = "Vision/{name}/PoseX")
        public double getPoseX() {
            return getPose().getX();
        }

        @AutoLogOutput(key = "Vision/{name}/PoseY")
        public double getPoseY() {
            return getPose().getY();
        }

        @AutoLogOutput(key = "Vision/{name}/TagCount")
        public double getTagCount() {
            return visionHelper.getTagCountInView();
        }

        @AutoLogOutput(key = "Vision/{name}/TargetSize")
        public double getTargetSize() {
            return visionHelper.getTargetSize();
        }
    }


}
