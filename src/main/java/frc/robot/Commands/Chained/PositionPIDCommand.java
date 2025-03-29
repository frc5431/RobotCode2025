package frc.robot.Commands.Chained;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Util.Constants.DrivebaseConstants;

public class PositionPIDCommand extends Command {

    public Drivebase drivebase;
    public final Pose2d goalPose;
    private PPHolonomicDriveController alignController = DrivebaseConstants.alignController;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final Timer timer = new Timer();

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging")
            .getBooleanTopic("PositionPIDEndTrigger").publish();
    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging")
            .getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging")
            .getDoubleTopic("Y Error").publish();

    private PositionPIDCommand(Drivebase drivebase, Pose2d goalPose) {
        this.drivebase = drivebase;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = drivebase.getRobotPose().relativeTo(goalPose);

            var rotation = MathUtil.isNear(
                    0.0,
                    diff.getRotation().getRotations(),
                    DrivebaseConstants.rotationTolerance.getRotations(),
                    0.0,
                    1.0);

            var position = diff.getTranslation().getNorm() < DrivebaseConstants.positionTolerance.in(Units.Meter);

            var speed = drivebase.getSpeed() < DrivebaseConstants.speedTolerance.in(Units.MetersPerSecond);

            // System.out.println("end trigger conditions R: "+ rotation + "\tP: " +
            // position + "\tS: " + speed);

            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(0.1);
    }

    public static Command generateCommand(Drivebase drivebase, Pose2d goalPose, Time timeout) {
        return new PositionPIDCommand(drivebase, goalPose).withTimeout(timeout).finallyDo(() -> {
            drivebase.stopRobotCentric();
            // TODO LOCK MODULES HERE
            // UNLESS IT DOES THAT FOR US
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        drivebase.driveAlign(
                alignController
                .calculateRobotRelativeSpeeds(
                        drivebase.getRobotPose(), goalState));

        xErrLogger.accept(drivebase.getRobotPose().getX() - goalPose.getX());
        yErrLogger.accept(drivebase.getRobotPose().getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        timer.stop();

        Pose2d diff = drivebase.getRobotPose().relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
                + "\nPosition offset: " + Units.Inches.convertFrom(diff.getTranslation().getNorm(), Units.Meters)
                + " in"
                + "\nRotation offset: " + diff.getRotation().getMeasure().in(Units.Degrees) + " deg"
                + "\nVelocity value: " + drivebase.getChassisSpeeds() + "m/s");
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}