package frc.robot.Subsytems.Limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Drivebase.Drivebase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsytems.Limelight.LimelightHelpers.RawFiducial;
import frc.robot.Subsytems.Limelight.LimelightHelpers.Trio;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper;
import frc.robot.Subsytems.Limelight.VisionUtil.VisionConfig;
import frc.robot.Util.Field;
import lombok.Getter;
import lombok.Setter;
import frc.robot.Util.Constants.VisionConstants;
import frc.team5431.titan.core.misc.Calc;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.text.DecimalFormat;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.Systems;

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision}
     * {@code allLimelights} &
     * {@code poseLimelights}
     */

    private @Getter Trigger reefAlignment = new Trigger(
            Systems.getDriver().rightBumper().or(Systems.getDriver().leftBumper()));

    /* Pose Estimation Constants */ // 2.3;

    // Increase these numbers to trust global measurements from vision less.
    public static double VISION_STD_DEV_X = 0.5;
    public static double VISION_STD_DEV_Y = 0.5;
    public static double VISION_STD_DEV_THETA = 99999999;

    public static final Matrix<N3, N1> visionStdMatrix = VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y,
            VISION_STD_DEV_THETA);

    // TODO: deal with this
    private Drivebase drivebase = Systems.getDrivebase();

    // TODO: we only have one for now
    /* Limelights */
    public final VisionHelper centLL3 = new LimelightHelpers().new VisionHelper(
            VisionConfig.LEFT_LL, VisionConstants.centerTagPipeline, VisionConfig.centerConfig);

    // public final LimelightLogger leftLogger = new LimelightLogger("Left",
    // centLL3);
    // public final VisionHelper rightLL = new LimelightHelpers().new VisionHelper(
    // VisionConfig.RIGHT_LL,
    // VisionConstants.rightTagPipeline,
    // VisionConfig.RIGHT_CONFIG);
    // public final LimelightLogger rightLogger = new LimelightLogger("Right",
    // rightLL);
    public final VisionHelper[] allLimelights = { centLL3 };
    public final VisionHelper[] poseLimelights = {
            centLL3
    };

    private final DecimalFormat df = new DecimalFormat();

    @AutoLogOutput(key = "Vision/is_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    private @Setter boolean isAligning = false;

    public Vision() {
        setName("Vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (VisionHelper visionHelper : allLimelights) {
            visionHelper.setLEDMode(false);
        }
    }

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("Reef Tag SCan", this.OnlyIfNullChecker()); // Yaw should be 0 when intake faces red
                                                                              // alliance and manip faces blue
        // Yaw should be 180 when intake faces blue alliance and manip faces red
        double yaw = Systems.getDrivebase().getOperatorForwardDirection().getMeasure().plus(Degrees.of(180))
                .in(Degrees);
        for (VisionHelper visionHelper : poseLimelights) {
            visionHelper.setRobotOrientation(yaw);

            if (DriverStation.isAutonomousEnabled() && visionHelper.targetInView()) {
                Pose3d botpose3D = visionHelper.getRawPose3d();
                Pose2d megaPose2d = visionHelper.getMegaPose2d();
                double timeStamp = visionHelper.getRawPoseTimestamp();
                Pose2d integratablePose = new Pose2d(megaPose2d.getTranslation(), botpose3D.toPose2d().getRotation());
                autonPoses.add(Trio.of(botpose3D, integratablePose, timeStamp));
            }
        }

        try {
            isIntegrating = false;
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled() && VisionConstants.useVisionPeriodic) {

                // choose LL with best view of tags and integrate from only that camera
                VisionHelper bestLimelight = centLL3;
                addFilteredVisionInput(bestLimelight);
                // for (VisionHelper visionHelper : poseLimelights) {
                // if (Field.isReef((bestLimelight.getClosestTagID()))) {
                // addFilteredVisionInput(bestLimelight);
                // } else {
                // visionHelper.sendInvalidStatus("SAD!: Apriltag is not matched Reef ID");
                // }
                // isIntegrating |= visionHelper.isIntegrating;
                // }

            }
        } catch (Exception e) {
            System.out.println("Vision pose not present but tried to access it [vision.periodic()]");
        }
    }

    private void addFilteredVisionInput(VisionHelper ll) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            @SuppressWarnings("unused")
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getRawPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = ll.getMegaPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeeds = Systems.getDrivebase().getChassisSpeeds();

            /* rejections */
            // reject pose if individual tag ambiguity is too high
            ll.tagStatus = "";
            for (RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2) {
                    highestAmbiguity = tag.ambiguity;
                } else if (tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // log ambiguities
                ll.tagStatus += "Tag " + tag.id + ": " + tag.ambiguity;
                // ambiguity rejection check
                if (tag.ambiguity > 0.4) {
                    ll.sendInvalidStatus("vision: ambiguity rejection {vision:179ish}");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                ll.sendInvalidStatus("vision: bound rejection {vision:185ish}");
                return;
            } else if (Math.abs(robotSpeeds.omegaRadiansPerSecond) >= VisionConstants.maxRadPerSec
                    .in(RadiansPerSecond)) {
                // reject if we are rotating more than an amount of radians per second
                ll.sendInvalidStatus("vision: robot rotation too fast");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("vision: roll/pitch rejection");
                return;
            } else if (targetSize <= VisionConstants.minSizeRejection) {
                ll.sendInvalidStatus("vision: size rejection");
                return;
            }
            /* CONFIDENCE (integration/standard devation) Evaluation */
            // if almost stationary and extremely close to tag
            else if (targetSize >= 3) {
                ll.sendValidStatus("Vision: Stationary close integration");
                xyStds = VisionConstants.servicableTrustStds;
                degStds = VisionConstants.dismalTrustStds;
            
            } else {
                //System.out.println("Rejected, get better");
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = VisionConstants.dismalTrustStds;
            }

            if (robotSpeeds.omegaRadiansPerSecond >= VisionConstants.lowTrustRadPerSec.in(RadiansPerSecond)) {
                degStds = VisionConstants.dismalTrustStds;
            }
            degStds = 9999;

            //SmartDashboard.putNumber("HIGHEST AMBIG", highestAmbiguity);
            SmartDashboard.putNumber("target size", targetSize);
            //SmartDashboard.putNumber("pose error", poseError);
            //SmartDashboard.putBoolean("multi tag", multiTags);
            SmartDashboard.putNumber("xystds", xyStds);
            SmartDashboard.putNumber("theteaSTDS", degStds);

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;
            degStds = 9999;

            Systems.getDrivebase().setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            xyStds,
                            xyStds,
                            degStds));

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());
            Systems.getDrivebase().addVisionMeasurement(integratedPose, timeStamp);
            // Systems.getDrivebase().resetPose(Systems.getEstimator().getCurrentPose());
            // Systems.getEstimator().getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(
            // xyStds,
            // xyStds,
            // degStds));

            // Systems.getEstimator().getPoseEstimator().addVisionMeasurement(integratedPose,
            // timeStamp);

        } else {
            ll.tagStatus = "no tags";
            ll.sendInvalidStatus("Vision: no tag found rejection");
            // System.out.println("********\nNoTag In View\n*********************");
        }
    }

    public void resetPoseToVision() {
        VisionHelper ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d robotPose = Systems.getDrivebase().getRobotPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                System.out.println(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE BAD POSE");
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) {
                System.out.println(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE OUT OF FIELD");
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                System.out.println(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE IN AIR");
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                System.out.println(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE TILTED");
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;

            System.out.println(
                    "ResetPoseToVision: Old Pose X: "
                            + robotPose.getX()
                            + " Y: "
                            + robotPose.getY()
                            + " Theta: "
                            + robotPose.getRotation().getDegrees());
            Systems.getDrivebase().setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Systems.getDrivebase().addVisionMeasurement(integratedPose, poseTimestamp);
            // robotpose after vision mesurments have been added
            robotPose = Systems.getDrivebase().getRobotPose();
            System.out.println(
                    "ResetPoseToVision: New Pose X: "
                            + robotPose.getX()
                            + " Y: "
                            + robotPose.getY()
                            + " Theta: "
                            + robotPose.getRotation().getDegrees());
            System.out.println("ResetPoseToVision: SUCCESS");
            return true;
        }
        return false; // target not in view
    }

    public VisionHelper getBestLimelight() {
        return centLL3;
    }

    @AutoLogOutput(key = "Vision/BestLimelight")
    public String logBestLimelight() {
        return getBestLimelight().CAMERA_NAME;
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (VisionHelper visionHelper : poseLimelights) {
            if (visionHelper.hasAccuratePose())
                return true;
        }
        return false;
    }

    public Distance getCameraXDistance() {
        return LimelightHelpers.getCameraPose3d_TargetSpace(this.getBestLimelight().CAMERA_NAME).getMeasureX();
    }

    public Distance getCameraYDistance() {
        return LimelightHelpers.getCameraPose3d_TargetSpace(this.getBestLimelight().CAMERA_NAME).getMeasureZ();
    }

    public boolean getPipeAlignDist(boolean rightTrue) {
        return Calc.approxEquals(getCameraXDistance().in(Inches),
                rightTrue ? VisionConstants.rightPipeOffset.in(Inches) : VisionConstants.leftPipeOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean getPipeScoreDist() {
        return Calc.approxEquals(getCameraYDistance().in(Inches),
                VisionConstants.pipeScoreOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean leftOfTag() {
        return getCameraXDistance().in(Inches) < VisionConstants.centerOffset.in(Inches);
    }

    public boolean isCentered() {
        return Calc.approxEquals(getCameraXDistance().in(Inches),
                VisionConstants.centerOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean getCenterScoreDistance() {
        return Calc.approxEquals(getCameraYDistance().in(Inches),
                VisionConstants.centerScoreOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (VisionHelper visionHelper : allLimelights) {
            visionHelper.setLimelightPipeline(pipeline);
        }
    }

    public boolean OnlyIfNullChecker() {
        try {
            return (Field.isRedTag(getBestLimelight().getClosestTagID()) == Field.isRed())
                    && Field.isReef(getBestLimelight().getClosestTagID());
        } catch (Exception e) {
            return false;
        }
    }

    /** Set all LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                () -> {
                    for (VisionHelper visionHelper : allLimelights) {
                        visionHelper.blinkLEDs();
                    }
                },
                () -> {
                    for (VisionHelper visionHelper : allLimelights) {
                        visionHelper.setLEDMode(false);
                    }
                })
                        .withName("Vision.blinkLimelights");
    }

    /** Set left LL to blink */
    public Command solidLimelight() {
        return startEnd(
                () -> {
                    centLL3.setLEDMode(true);
                },
                () -> {
                    centLL3.setLEDMode(false);
                })
                        .withName("Vision.blinkLimelights");
    }

}