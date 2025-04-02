package frc.robot.Commands.Chained;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Util.Field;
import frc.robot.Util.Constants.DrivebaseConstants;
import frc.robot.Util.Field.BranchSide;
import frc.robot.Util.Field.ReefSide;

public class AlignToReef {

    private final Drivebase drivebase;

    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();

    public boolean isPIDLoopRunning = false;

    public AlignToReef(Drivebase drivebase, AprilTagFieldLayout field) {
        this.drivebase = drivebase;

        Arrays.stream(Field.blueReefAprilTags).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                        p.getMeasureX(),
                        p.getMeasureY(),
                        p.getRotation().toRotation2d()));
            });
        });

        Arrays.stream(Field.redReefAprilTags).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                        p.getMeasureX(),
                        p.getMeasureY(),
                        p.getRotation().toRotation2d()));
            });
        });

        Arrays.stream(Field.reefAprilTags).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                        p.getMeasureX(),
                        p.getMeasureY(),
                        p.getRotation().toRotation2d()));
            });
        });
    }

    /**
     * this is an enum that represents if the branch is on the left or right side
     * ofthe field, instead of relative to the tag
     */
    public enum FieldBranchSide {
        LEFT(BranchSide.LEFT), RIGHT(BranchSide.RIGHT), MIDDLE(BranchSide.MIDDLE);

        public BranchSide branchSide;

        public FieldBranchSide getOpposite() {
            switch (this) {
                case LEFT:
                    return FieldBranchSide.RIGHT;
                case RIGHT:
                    return FieldBranchSide.LEFT;
                case MIDDLE:
                    return FieldBranchSide.MIDDLE;
            }
            System.out.println("Error, switch case failed to catch the field branch side");
            return this;
        }

        private FieldBranchSide(BranchSide internal) {
            this.branchSide = internal;
        }
    }

    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging")
            .getStructTopic("desired branch", Pose2d.struct).publish();

    private PathConstraints pathConstraints = DrivebaseConstants.constraints;

    public void changePathConstraints(PathConstraints newPathConstraints) {
        this.pathConstraints = newPathConstraints;
    }

    public Command generateCommand(FieldBranchSide side) {
        return new SequentialCommandGroup(Commands.none(),
        Commands.defer(() -> {
            var branch = getClosestBranch(side, drivebase);
            drivebase.resetPose(Systems.getEstimator().getCurrentPose());
            desiredBranchPublisher.accept(branch);
            Command command = new DriveToPoseCommand(drivebase, () -> Systems.getEstimator().getCurrentPose(), branch, 1);
            // Commansd command = getPathFromWaypoint(getWaypointFromBranch(branch));
            // command.addRequirement(drivebase);
            return command;
        }, Set.of()));
    }

    public Command generateCommand(final ReefSide reefTag, BranchSide side) {
        return new SequentialCommandGroup(Commands.none(),
        Commands.defer(() -> {
            var branch = getBranchFromTag(reefTag.getCurrent(), side);
            desiredBranchPublisher.accept(branch);

            Command command = getPathFromWaypoint(getWaypointFromBranch(branch));
            command.addRequirements(drivebase);
            return command;
        }, Set.of(drivebase)));
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drivebase.getRobotPose().getTranslation(),
                        getPathVelocityHeading(drivebase.getChassisSpeeds(), waypoint)),
                waypoint);

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return Commands.sequence(
                    Commands.print("start position PID loop"),
                    PositionPIDCommand.generateCommand(drivebase, waypoint, DrivebaseConstants.autoAlignAdjustTimeout),
                    Commands.print("end position PID loop"));
        }

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                DriverStation.isAutonomous() ? pathConstraints : DrivebaseConstants.constraints,
                new IdealStartingState(getVelocityMagnitude(drivebase.getChassisSpeeds()), drivebase.getRobotHeading()),
                new GoalEndState(0.0, waypoint.getRotation()));

        path.preventFlipping = true;

        drivebase.resetPose(Systems.getEstimator().getCurrentPose());

        return (AutoBuilder.followPath(path).andThen(
                Commands.print("start position PID loop"),
                PositionPIDCommand
                        .generateCommand(drivebase, waypoint,
                                (DriverStation.isAutonomous() ? DrivebaseConstants.autoAlignAdjustTimeout
                                        : DrivebaseConstants.teleopAlignAdjustTimeout))
                        .beforeStarting(Commands.runOnce(() -> {
                            isPIDLoopRunning = true;
                        }))
                        .finallyDo(() -> {
                            isPIDLoopRunning = false;
                        }),
                Commands.print("end position PID loop"))).finallyDo((interupt) -> {
                    if (interupt) { // if this is false then the position pid would've X braked & called the same
                                    // method
                        drivebase.stopRobotCentric();
                    }
                });
    }

    /**
     * @param cs
     *            field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
        if (getVelocityMagnitude(cs).in(Units.MetersPerSecond) < 0.25) {
            System.out.println("approach: straight line");
            var diff = target.getTranslation().minus(drivebase.getRobotPose().getTranslation());
            System.out.println("diff calc: \nx: " + diff.getX() + "\ny: " + diff.getY() + "\nDoT: "
                    + diff.getAngle().getDegrees());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();// .rotateBy(Rotation2d.k180deg);
        }

        System.out.println("approach: compensating for velocity");

        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);

        System.out.println(
                "velocity calc: \nx: " + cs.vxMetersPerSecond + "\ny: " + cs.vyMetersPerSecond + "\nDoT: " + rotation);

        return rotation;
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
        return Units.MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    /**
     * @return Pathplanner waypoint with direction of travel away from the
     *         associated reef side
     */
    private Pose2d getWaypointFromBranch(Pose2d branch) {
        return new Pose2d(
                branch.getTranslation(),
                branch.getRotation());
    }

    /**
     * @return target rotation for the robot when it reaches the final waypoint
     */
    private Rotation2d getBranchRotation(Drivebase drivebase) {
        return getClosestReefAprilTag(drivebase.getRobotPose()).getRotation()
                // WARNING: could cause an rotation issue down below
                .rotateBy(Rotation2d.kZero);
    }

    public static Pose2d getClosestBranch(FieldBranchSide fieldSide, Drivebase drivebase) {
        Pose2d robotPose = drivebase.predict(DrivebaseConstants.autoAlignPredict);

        Pose2d tag = getClosestReefAprilTag(robotPose);

        BranchSide tagSide = fieldSide.branchSide;

        if (robotPose.getX() > 4.500
                &&
                robotPose.getX() < 13) {
            tagSide = fieldSide.getOpposite().branchSide;
        }

        return getBranchFromTag(tag, tagSide);
    }

    private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side) {
        var translation = tag.getTranslation().plus(
                new Translation2d(
                        side.tagOffset.getY(),
                        side.tagOffset.getX()).rotateBy(tag.getRotation()));

        return new Pose2d(
                translation.getX(),
                translation.getY(),
                tag.getRotation());
    }

    /**
     * get closest reef april tag pose to given position
     * 
     * @param pose field relative position
     * @return
     */
    public static Pose2d getClosestReefAprilTag(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        
        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else{
            reefPoseList = Field.isBlue() ? 
                blueReefTagPoses :
                redReefTagPoses;
        }

        
        return pose.nearest(reefPoseList);

    }

}