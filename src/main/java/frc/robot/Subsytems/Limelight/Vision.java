package frc.robot.Subsytems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsytems.Limelight.LimelightHelpers.LimelightResults;
import frc.robot.Subsytems.Limelight.LimelightHelpers.RawFiducial;

import java.text.DecimalFormat;

public class Vision {

    /* Limelight Configuration */

    /** Must match to the name given in LL dashboard */
    public final String CAMERA_NAME;

    public String logStatus = "";
    public String tagStatus = "";
    public boolean isIntegrating;
    /** Physical Config */
    private PhysicalConfig physicalConfig;

    /* Debug */
    private final DecimalFormat df = new DecimalFormat();

    public Vision(String cameraName) {
        this.CAMERA_NAME = cameraName;
        physicalConfig = new PhysicalConfig();
        logStatus = "Not started";
        tagStatus = "Not started";
        isIntegrating = false;
    }

    public Vision(String cameraName, int pipeline) {
        this(cameraName);
        setLimelightPipeline(pipeline);
    }

    public Vision(String cameraName, int pipeline, PhysicalConfig physicalConfig) {
        this(cameraName, pipeline);
        this.physicalConfig = physicalConfig;
        // LimelightHelpers.setCameraPose_RobotSpace(
        //         this.CAMERA_NAME,
        //         physicalConfig.forward,
        //         physicalConfig.right,
        //         physicalConfig.up,
        //         physicalConfig.roll,
        //         physicalConfig.pitch,
        //         physicalConfig.yaw);
    }

    /*
     *
     * Frequently Used Methods
     *
     *
     */

    /* ::: Basic Information Retrieval ::: */

    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2:
     *     -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(CAMERA_NAME);
    }

    /**
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(CAMERA_NAME);
    }

    /** @return Whether the LL has any valid targets (apriltags or other vision targets) */
    public boolean targetInView() {
        return LimelightHelpers.getTV(CAMERA_NAME);
    }

    /** @return whether the LL sees multiple tags or not */
    public boolean multipleTagsInView() {
        return getTagCountInView() > 1;
    }

    public double getTagCountInView() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(CAMERA_NAME).tagCount;

        // if (retrieveJSON() == null) return 0;

        // return retrieveJSON().targetingResults.targets_Fiducials.length;
    }

    /**
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     */
    public double getClosestTagID() {
        return LimelightHelpers.getFiducialID(CAMERA_NAME);
    }

    public double getTargetSize() {
        return LimelightHelpers.getTA(CAMERA_NAME);
    }

    /* ::: Pose Retrieval ::: */

    /** @return the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java */
    public Pose3d getRawPose3d() {
        return LimelightHelpers.getBotPose3d_wpiBlue(
                CAMERA_NAME); // 2024: all alliances use blue as 0,0
    }

    /** @return the corresponding LL Pose3d (MEGATAG2) for the alliance in DriverStation.java */
    public Pose2d getMegaPose2d() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAMERA_NAME)
                .pose; // 2024: all alliances use blue as 0,0
    }

    public boolean hasAccuratePose() {
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /** @return the distance of the 2d vector from the camera to closest apriltag */
    public double getDistanceToTagFromCamera() {
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(CAMERA_NAME).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(CAMERA_NAME).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public RawFiducial[] getRawFiducial() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(CAMERA_NAME).rawFiducials;
    }

    /**
     * Returns the timestamp of the MEGATAG1 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
    public double getRawPoseTimestamp() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(CAMERA_NAME).timestampSeconds;
    }

    /**
     * Returns the timestamp of the MEGATAG2 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
    public double getMegaPoseTimestamp() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAMERA_NAME).timestampSeconds;
    }

    /**
     * Returns the latency of the pose estimation from the Limelight camera.
     *
     * @return The latency of the pose estimation in seconds.
     */
    @Deprecated(forRemoval = true)
    public double getPoseLatency() {
        return Units.millisecondsToSeconds(LimelightHelpers.getBotPose_wpiBlue(CAMERA_NAME)[6]);
    }

    /*
     *
     * Custom Helpers
     *
     *
     */

    /**
     * get distance in meters to a target
     *
     * @param targetHeight meters
     * @return
     */
    public double getDistanceToTarget(double targetHeight) {
        return (targetHeight - physicalConfig.up)
                / Math.tan(Units.degreesToRadians(physicalConfig.roll + getVerticalOffset()));
    }

    public void sendValidStatus(String message) {
        isIntegrating = true;
        logStatus = message;
    }

    public void sendInvalidStatus(String message) {
        isIntegrating = false;
        logStatus = message;
    }

    /*
     *
     * Utility Wrappers
     *
     *
     */

    /** @return The latest LL results as a LimelightResults object. */
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(CAMERA_NAME);
    }

  
    public void setLimelightPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(CAMERA_NAME, pipelineIndex);
    }

    /** */
    public void setRobotOrientation(double degrees) {
        LimelightHelpers.SetRobotOrientation(CAMERA_NAME, degrees, 0, 0, 0, 0, 0);
    }

    public void setRobotOrientation(double degrees, double angularRate) {
        LimelightHelpers.SetRobotOrientation(CAMERA_NAME, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * Sets the LED mode of the LL.
     *
     * @param enabled true to enable the LED mode, false to disable it
     */
    public void setLEDMode(boolean enabled) {
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(CAMERA_NAME);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(CAMERA_NAME);
        }
    }

    /**
     * Set LL LED's to blink
     *
     * @return
     */
    public void blinkLEDs() {
        LimelightHelpers.setLEDMode_ForceBlink(CAMERA_NAME);
    }

    /** Checks if the camera is connected by looking for an empty botpose array from camera. */
    public boolean isCameraConnected() {
        try {
            var rawPoseArray =
                    LimelightHelpers.getLimelightNTTableEntry(CAMERA_NAME, "botpose_wpiblue")
                            .getDoubleArray(new double[0]);
            if (rawPoseArray.length < 6) {
                return false;
            }
            return true;
        } catch (Exception e) {
            System.err.println("Avoided crashing statement in Limelight.java: isCameraConnected()");
            return false;
        }
    }

    /** Prints the vision, estimated, and odometry pose to SmartDashboard */
    public void printDebug() {
        Pose3d botPose3d = getRawPose3d();
        SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
        SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
        SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
        SmartDashboard.putString(
                "LimelightRoll", df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
        SmartDashboard.putString(
                "LimelightPitch",
                df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
        SmartDashboard.putString(
                "LimelightYaw", df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
    }


    /**
     * Specify the location of your Limelight relative to the center of your robot. (meters,
     * degrees)
     */
    public static class PhysicalConfig {
        public double forward, right, up; // meters
        public double roll, pitch, yaw; // degrees

        /**
         * Specify the location of your Limelight relative to the center of your robot. (meters,
         * degrees)
         */
        public PhysicalConfig() {}

        /**
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return
         */
        public PhysicalConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return
         */
        public PhysicalConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }
}