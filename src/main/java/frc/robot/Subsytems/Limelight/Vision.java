package frc.robot.Subsytems.Limelight;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsytems.Limelight.LimelightHelpers.RawFiducial;
import frc.robot.Subsytems.Limelight.LimelightHelpers.Trio;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper;
import frc.robot.Subsytems.Limelight.VisionUtil.LimelightLogger;
import frc.robot.Subsytems.Limelight.VisionUtil.VisionConfig;
import frc.robot.Util.Field;
import frc.robot.Util.Constants.VisionConstants;
import frc.team5431.titan.core.vision.Limelight;

import java.text.DecimalFormat;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;

//TODO: DO NOT DELETE ANY IMPORTS, it's unused the stuff below is commented

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision} {@code allLimelights} &
     * {@code poseLimelights}
     */
    

        /* Pose Estimation Constants */ // 2.3;

        // Increase these numbers to trust global measurements from vision less.
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs */

            
        

        
    

    /* Limelights */
    public final VisionHelper leftLL =
            new LimelightHelpers().new VisionHelper(
                    VisionConfig.LEFT_LL, VisionConstants.leftTagPipeline, VisionConfig.LEFT_CONFIG);
    public final LimelightLogger leftLogger = new LimelightLogger("Left", leftLL);
    public final VisionHelper rightLL =
            new LimelightHelpers().new VisionHelper(
                    VisionConfig.RIGHT_LL,
                    VisionConstants.rightTagPipeline,
                    VisionConfig.RIGHT_CONFIG);
    public final LimelightLogger rightLogger = new LimelightLogger("Right", rightLL);
    public final VisionHelper[] allLimelights = {leftLL, rightLL};
    public final VisionHelper[] poseLimelights = {
        leftLL, rightLL
    }; 

    private final DecimalFormat df = new DecimalFormat();

    @AutoLogOutput(key = "Vision/a_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    private boolean isAiming = false;

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
        double yaw = 0; /* = Robot.swerve.getRotation().getDegrees();*/ //TODO fix this assign actual value
        for (VisionHelper visionHelper : poseLimelights) {
            visionHelper.setRobotOrientation(yaw);

            if (DriverStation.isAutonomousEnabled() && visionHelper.targetInView()) {
                Pose3d botpose3D = visionHelper.getRawPose3d();
                Pose2d megaPose2d = visionHelper.getMegaPose2d();
                double timeStamp = visionHelper.getRawPoseTimestamp();
                Pose2d integratablePose =
                        new Pose2d(megaPose2d.getTranslation(), botpose3D.toPose2d().getRotation());
                autonPoses.add(Trio.of(botpose3D, integratablePose, timeStamp));
            }
        }

        try {
            isIntegrating = false;
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled()) {
                // if the front camera sees tag and we are aiming, only use that camera
                // if (isAiming && speakerLL.targetInView()) {
                //     for (Limelight limelight : limelights) {
                //         if (limelight.CAMERA_NAME == speakerLL.CAMERA_NAME) {
                //             addFilteredVisionInput(limelight);
                //         } else {
                //             limelight.sendInvalidStatus("speaker only rejection");
                //         }
                //         isIntegrating |= limelight.isIntegrating;
                //     }
                // } else {
                // choose LL with best view of tags and integrate from only that camera
                VisionHelper bestLimelight = getBestLimelight();
                for (VisionHelper visionHelper : poseLimelights) {
                    if (visionHelper.CAMERA_NAME == bestLimelight.CAMERA_NAME) {
                        addFilteredVisionInput(bestLimelight);
                    } else {
                        visionHelper.sendInvalidStatus("not best rejection");
                    }
                    isIntegrating |= visionHelper.isIntegrating;
                }
                // }
            }
        } catch (Exception e) {
            System.out.println("Vision pose not present but tried to access it (manmade error)");
        }
    }

    private void addFilteredVisionInput(VisionHelper ll) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getRawPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = ll.getMegaPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            // ChassisSpeeds robotSpeed /* = Robot.swerve.getVelocity(true); */; //TODO: fix this assign value

            // distance from current pose to vision estimated pose
            double poseDifference = 1; /* =
                    Robot.swerve.getPose().getTranslation().getDistance(botpose.getTranslation()); */ //TODO: Fix this

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
                if (tag.ambiguity > 0.9) {
                    ll.sendInvalidStatus("ambiguity rejection");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                ll.sendInvalidStatus("bound rejection");
                return;
             } /*else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
            //     // reject if we are rotating more than 0.5 rad/s
            //     ll.sendInvalidStatus("rotation rejection");
            //     return; }*/ //TODO: when robotSpeed has value uncomment this
            else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                ll.sendInvalidStatus("height rejection");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("roll/pitch rejection");
                return;
            } else if (targetSize <= 0.025) {
                ll.sendInvalidStatus("size rejection");
                return;
            }
            /* integrations */
            // if almost stationary and extremely close to tag
            /*else if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1; //TODO: uncomment when robotSpeed has value
            } */ else if (multiTags && targetSize > 0.05) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
                if (targetSize > 0.09) {
                    ll.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                    degStds = 0.1;
                }
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 16;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                ll.sendValidStatus("Proximity integration");
                xyStds = 2.0;
                degStds = 999999;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 0.5;
                degStds = 999999;
            } else {
                System.out.println("Rejected, get better");
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            // if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
            //     degStds = 15;
            // } //TODO: Uncomment when robotSpeed has value

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            // TODO: Fix this
            // Robot.swerve.setVisionMeasurementStdDevs(
            //         VecBuilder.fill(
            //                 VisionConfig.VISION_STD_DEV_X,
            //                 VisionConfig.VISION_STD_DEV_Y,
            //                 VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());
            // Robot.swerve.addVisionMeasurement(integratedPose, timeStamp); //TODO: Fix this
        } else {
            ll.tagStatus = "no tags";
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    /**
     * REQUIRES ACCURATE POSE ESTIMATION. Uses trigonometric functions to calculate the angle
     * between the robot heading and the angle required to face the speaker center.
     *
     * @return angle between robot heading and speaker in degrees
     */
    

    

    /** Returns the distance from the speaker in meters, adjusted for the robot's movement. */

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take, adjusted
     * for the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedTargetPos(Translation2d targetPose) {
        double NORM_FUDGE = 0.075;
        double tunableNoteVelocity = 1;
        double tunableNormFudge = 0;
        double tunableStrafeFudge = 1;
        double tunableSpeakerYFudge = 0.0;
        double tunableSpeakerXFudge = 0.0;

        Translation2d robotPos; /* = Robot.swerve.getPose().getTranslation(); */ //TODO: fix this
        targetPose = Field.flipXifRed(targetPose);
        // double xDifference = Math.abs(robotPos.getX() - targetPose.getX()); //TODO: uncomment when robotPos has value
        // double spinYFudge =
        //         (xDifference < 5.8)
        //                 ? 0.05
        //                 : 0.8; // change spin fudge for score distances vs. feed distances //TODO: uncomment when xDifference has value

        // ChassisSpeeds robotVel; /* = Robot.swerve.getVelocity(true);*/ // TODO: fix this

        // double distance = robotPos.getDistance(targetPose); //TODO: uncomment when robotPos has value
        // double normFactor =
        //         Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
        //                 ? 0.0
        //                 : Math.abs(
        //                         MathUtil.angleModulus(
        //                                         robotPos.minus(targetPose).getAngle().getRadians()
        //                                                 - Math.atan2(
        //                                                         robotVel.vyMetersPerSecond,
        //                                                         robotVel.vxMetersPerSecond))
        //                                 / Math.PI); //TODO: uncomment when robotVel has value

        double x =
                targetPose.getX() + (Field.isBlue() ? tunableSpeakerXFudge : -tunableSpeakerXFudge);
        // - (robotVel.vxMetersPerSecond * (distance / tunableNoteVelocity));
        //      * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                targetPose.getY()
                        // + (Field.isBlue() ? -spinYFudge : spinYFudge) //TODO: uncomment when spinYFudge has value
                        + tunableSpeakerYFudge;
        // - (robotVel.vyMetersPerSecond * (distance / tunableNoteVelocity));
        //       * tunableStrafeFudge);

        return new Translation2d(x, y);
    }

    public void autonResetPoseToVision() {
        boolean reject = true;
        boolean firstSuccess = false;
        double batchSize = 5;
        for (int i = autonPoses.size() - 1; i > autonPoses.size() - (batchSize + 1); i--) {
            Trio<Pose3d, Pose2d, Double> poseInfo = autonPoses.get(i);
            boolean success =
                    resetPoseToVision(
                            true, poseInfo.getFirst(), poseInfo.getSecond(), poseInfo.getThird());
            if (success) {
                if (i == autonPoses.size() - 1) {
                    firstSuccess = true;
                }
                reject = false;
                System.out.println(
                        "AutonResetPoseToVision succeeded on " + (autonPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            System.out.println(
                    "AutonResetPoseToVision failed after "
                            + batchSize
                            + " of "
                            + autonPoses.size()
                            + " possible tries");
           
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
            Pose2d robotPose; /* = Robot.swerve.getPose();*/ //TODO: fix this
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

            // System.out.println(
            //         "ResetPoseToVision: Old Pose X: "
            //                 + robotPose.getX()
            //                 + " Y: "
            //                 + robotPose.getY()
            //                 + " Theta: "
            //                 + robotPose.getRotation().getDegrees()); //TODO: Uncomment when robotPose has value
            // Robot.swerve.setVisionMeasurementStdDevs(
            //         VecBuilder.fill(
            //                 VisionConfig.VISION_STD_DEV_X,
            //                 VisionConfig.VISION_STD_DEV_Y,
            //                 VisionConfig.VISION_STD_DEV_THETA)); //TODO: fix this

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            // Robot.swerve.addVisionMeasurement(integratedPose, poseTimestamp); //TODO: fix this
            // robotPose = Robot.swerve.getPose(); */ //TODO: fix this
            // System.out.println(
            //         "ResetPoseToVision: New Pose X: "
            //                 + robotPose.getX()
            //                 + " Y: "
            //                 + robotPose.getY()
            //                 + " Theta: "
            //                 + robotPose.getRotation().getDegrees()); //TODO: Uncomment when robotPose has value
            System.out.println("ResetPoseToVision: SUCCESS");
            return true;
        }
        return false; // target not in view
    }

    public VisionHelper getBestLimelight() {
        VisionHelper bestLimelight = rightLL;
        double bestScore = 0;
        for (VisionHelper visionHelper : poseLimelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += visionHelper.getTagCountInView();
            score += visionHelper.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = visionHelper;
            }
        }
        return bestLimelight;
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
            if (visionHelper.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (VisionHelper visionHelper : allLimelights) {
            visionHelper.setLimelightPipeline(pipeline);
        }
    }

    /** Set both LLs to blink */
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
                            leftLL.setLEDMode(true);
                        },
                        () -> {
                            leftLL.setLEDMode(false);
                        })
                .withName("Vision.blinkLimelights");
    }

    public void setAiming() {
        isAiming = true;
    }

    public void setNotAiming() {
        isAiming = false;
    }

    /** Logging */
}