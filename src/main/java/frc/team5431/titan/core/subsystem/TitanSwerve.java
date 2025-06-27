package frc.team5431.titan.core.subsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.misc.GameField;
import lombok.Getter;
import lombok.Setter;

/**
 * @author {blame} Natalie Daleo
 *         <h4>
 *         TitanUtil Class that utilizes Pheonix 6 Swerve
 *         </h4>
 *         <body>
 *         Class that extends the Phoenix 6 SwerveDrivetrain class and
 *         implements
 *         Subsystem so it can easily be used in command-based projects.
 */
public class TitanSwerve extends PheonixTunerSwerve implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private @Getter SwerveRequest.ForwardPerspectiveValue perspectiveValue = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * This should be tuned to your individual robot
     */
    private @Getter @Setter PIDConstants translationPID = new PIDConstants(1, 0, 0.5);
    /**
     * This should be tuned to your individual robot
     */
    private @Getter @Setter PIDConstants rotationPID = new PIDConstants(2, 0, 0);

    private @Getter SwerveRequest.RobotCentric visionRobotCentric = new RobotCentric();

    private @Getter SwerveRequest.FieldCentric driverFieldControl = new SwerveRequest.FieldCentric()
            .withForwardPerspective(perspectiveValue);

    private @Getter SwerveRequest.RobotCentric driverRobotControl = new SwerveRequest.RobotCentric();

    private @Getter SwerveRequest.RobotCentric alignControl = new SwerveRequest.RobotCentric();

    SwerveModuleState[] states = this.getState().ModuleStates;
    StructArrayPublisher<SwerveModuleState> modulePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Robot Pose", Pose2d.struct).publish();

    StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Chassis Speed", ChassisSpeeds.struct).publish();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *            Drivetrain-wide constants for the swerve drive
     * @param modules
     *            Constants for each specific module
     */
    public TitanSwerve(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *            Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency
     *            The frequency to run the odometry loop. If
     *            unspecified or set to 0 Hz, this is 250 Hz on
     *            CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation
     *            The standard deviation for odometry calculation
     *            in the form [x, y, theta]ᵀ, with units in meters
     *            and radians
     * @param visionStandardDeviation
     *            The standard deviation for vision calculation
     *            in the form [x, y, theta]ᵀ, with units in meters
     *            and radians
     * @param modules
     *            Constants for each specific module
     */
    public TitanSwerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        configureAutoBuilder();

        Logger.recordOutput("Robot Pose", this.getRobotPose());
        Logger.recordOutput("Chassis Speeds", this.getChassisSpeeds());
        Logger.recordOutput("Robot Heading", this.getRobotHeading());

    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPoses,
                    this::getChassisSpeeds,
                    (speeds) -> driveAuton(speeds),
                    new PPHolonomicDriveController(
                            translationPID,
                            rotationPID),
                    RobotConfig.fromGUISettings(),
                    () -> false,
                    this);

        } catch (Exception e) {
            DriverStation.reportError("Auton Config Issue", e.getStackTrace());
        }
    }

    /**
     * @return the robot's pose derived from the swerve module states
     */
    public Pose2d getRobotPose() {
        return getState().Pose;
    }

    /**
     * @return the robot's heading derived from the swerve module states
     */
    public Rotation2d getRobotHeading() {
        return getState().Pose.getRotation();
    }

    /**
     * @return the robot's chassis speeds derived from kinematics
     */
    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    public double getAverageDriveVoltage() {
        return ((this.getModule(0).getDriveMotor().getMotorVoltage().getValueAsDouble() +
                this.getModule(1).getDriveMotor().getMotorVoltage().getValueAsDouble() +
                this.getModule(2).getDriveMotor().getMotorVoltage().getValueAsDouble() +
                this.getModule(3).getDriveMotor().getMotorVoltage().getValueAsDouble()) / 4);
    }

    public boolean faceTargetEvaluate(double setpoint) {
        return Calc.approxEquals(this.getRobotPose().getRotation().getRadians(),
                Units.degreesToRadians(setpoint), 0.1);
    }

    public void resetGyro() {
        this.resetRotation(kBlueAlliancePerspectiveRotation);
    }

    /*
     * // Keep the robot on the field
     * public Pose2d keepPoseOnField(Pose2d pose) {
     * 
     * double halfBot =
     * double x = pose.getX();
     * double y = pose.getY();
     * 
     * // WARNING: IF ANTHING BAD IS EVER HAPPENING, IM NOT SURE THIS IS RIGHT
     * double newX = MathUtil.clamp(x, halfBot, (Field.getFieldLength().in(Meters) -
     * halfBot));
     * double newY = MathUtil.clamp(y, halfBot, (Field.getFieldLength().in(Meters) -
     * halfBot));
     * 
     * if (x != newX || y != newY) {
     * pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
     * }
     * return pose;
     * }
     */

    public Pose2d predict(Time inTheFuture, Pose2d currentPose) {

        var cs = getChassisSpeeds();

        return new Pose2d(
                currentPose.getX() + cs.vxMetersPerSecond * inTheFuture.in(Seconds),
                currentPose.getY() + cs.vyMetersPerSecond * inTheFuture.in(Seconds),
                currentPose.getRotation()
                        .plus(Rotation2d.fromRadians(cs.omegaRadiansPerSecond * inTheFuture.in(Seconds))));
    }

    public void resetPoses(Pose2d pose) {
        this.resetPose(pose);
        // Systems.getEstimator().setCurrentPose(pose);
    }

    public void driveAuton(ChassisSpeeds chassisSpeeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public void driveAuton(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
                .withSteerRequestType(SteerRequestType.Position)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public void driveAlign(ChassisSpeeds chassisSpeeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public void driveAlign(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
                .withSteerRequestType(SteerRequestType.Position)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    /**
     * Instant Command that resets Rotation2d to foward facing value
     */
    public Command zeroGyro() {
        return new InstantCommand(() -> resetGyro(), this);
    }

    public Command stopRobotCentric() {
        return new InstantCommand(() -> this
                .setControl(new SwerveRequest.RobotCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)));
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request
     *            Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command alignWheelsCommand(Rotation2d direction) {
        return run(() -> new SwerveRequest.PointWheelsAt().withModuleDirection(direction));
    }

    /**
     * Default Periodic Loop, should be called if the periodic function is
     * overridden
     */
    public void defaultPeriodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        SmartDashboard.putNumber("Drivebase Rotation", this.getRotation3d().getMeasureZ().baseUnitMagnitude());

        modulePublisher.set(states);
        posePublisher.set(getRobotPose());
        speedsPublisher.set(getChassisSpeeds());
        updateSimState(0.02, getAverageDriveVoltage()); // Added so I can use swerve in simulation

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    GameField.isBlue()
                            ? kBlueAlliancePerspectiveRotation
                            : kRedAlliancePerspectiveRotation);
            hasAppliedOperatorPerspective = true;
        }

    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        SmartDashboard.putNumber("Drivebase Rotation", this.getRotation3d().getMeasureZ().baseUnitMagnitude());

        modulePublisher.set(states);
        posePublisher.set(getRobotPose());
        speedsPublisher.set(getChassisSpeeds());
        updateSimState(0.02, getAverageDriveVoltage()); // Added so I can use swerve in simulation

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    GameField.isBlue()
                            ? kBlueAlliancePerspectiveRotation
                            : kRedAlliancePerspectiveRotation);
            hasAppliedOperatorPerspective = true;
        }

    }

}