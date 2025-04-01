package frc.robot.Subsytems.Drivebase;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Systems;
import frc.robot.Util.Field;
import frc.robot.Util.SwerveConstants;
import frc.robot.Util.Constants.AutonConstants;
import frc.robot.Util.Constants.DrivebaseConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util.SwerveConstants.TunerSwerveDrivetrain;
import frc.team5431.titan.core.misc.Calc;
import lombok.Getter;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Drivebase extends TunerSwerveDrivetrain implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    // TODO: GIVE DRIVER DPAD ALIGNMENT

    private @Getter SwerveRequest.ForwardPerspectiveValue perspectiveValue = ForwardPerspectiveValue.OperatorPerspective;

    private @Getter double driverFOCInverter = (Field.isBlue()) ? 1 : -1;

    private @Getter SwerveRequest.RobotCentric visionRobotCentric = new RobotCentric()
            .withRotationalDeadband(DrivebaseConstants.VisionAngularDeadzone);

    private @Getter SwerveRequest.FieldCentric driverFOControl = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.kSpeedAt12Volts.times(0.1))
            .withRotationalDeadband(DrivebaseConstants.AngularDeadzone) // Add a 10%
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private @Getter SwerveRequest.RobotCentric driverROControl = new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.kSpeedAt12Volts.times(0.1))
            .withRotationalDeadband(DrivebaseConstants.AngularDeadzone) // Add a 10%
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private @Getter SwerveRequest.RobotCentric alignControl = new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.kSpeedAt12Volts.times(0.1))
            .withRotationalDeadband(DrivebaseConstants.AngularDeadzone) // Add a 10%
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private @Getter TitanFieldCentricFacingAngle facingRequest = new TitanFieldCentricFacingAngle()
            .withPID(new PIDController(6, 0.01, 0.008));

    SwerveModuleState[] states = this.getState().ModuleStates;
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
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
    public Drivebase(
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
    public Drivebase(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        configureAutoBuilder();
    }

    public void resetGyro() {
        this.resetRotation(kBlueAlliancePerspectiveRotation );
    }

    public Command zeroGyro() {
        return new InstantCommand(() -> resetGyro(), this);
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPoses,
                    this::getChassisSpeeds,
                    (speeds) -> driveAuton(speeds),
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            AutonConstants.translationPID,
                            AutonConstants.rotationPID),
                    RobotConfig.fromGUISettings(),
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

        } catch (Exception e) {
            DriverStation.reportError("Auton Config Issue", e.getStackTrace());
        }
    }

    /**
     * The function `getRobotPose` returns the robot's pose after checking and
     * updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after
     *         calling the
     *         `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        return getState().Pose;
    }

    public Rotation2d getRobotHeading() {
        return getState().Pose.getRotation();

    }

    // Keep the robot on the field
    public Pose2d keepPoseOnField(Pose2d pose) {

        double halfBot = DrivebaseConstants.robotLength.div(2).in(Meters);
        double x = pose.getX();
        double y = pose.getY();

        // WARNING: IF ANTHING BAD IS EVER HAPPENING, IM NOT SURE THIS IS RIGHT
        double newX = MathUtil.clamp(x, halfBot, (Field.getFieldLength().in(Meters) - halfBot));
        double newY = MathUtil.clamp(y, halfBot, (Field.getFieldLength().in(Meters) - halfBot));

        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
        }
        return pose;
    }

    public Pose2d predict(Time inTheFuture) {

        Pose2d currPose = Systems.getEstimator().getCurrentPose();

        var cs = getChassisSpeeds();

        return new Pose2d(
                currPose.getX() + cs.vxMetersPerSecond * inTheFuture.in(Seconds),
                currPose.getY() + cs.vyMetersPerSecond * inTheFuture.in(Seconds),
                currPose.getRotation()
                        .plus(Rotation2d.fromRadians(cs.omegaRadiansPerSecond * inTheFuture.in(Seconds))));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    public double getSpeed() {
        ChassisSpeeds robotVelocity = getChassisSpeeds();
        return Math.sqrt(robotVelocity.vxMetersPerSecond * robotVelocity.vxMetersPerSecond
                + robotVelocity.vyMetersPerSecond * robotVelocity.vyMetersPerSecond);
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

    public void resetPoses(Pose2d pose) {
        this.resetPose(pose);
        Systems.getEstimator().setCurrentPose(pose);
        Systems.getLayout().setOrigin(new Pose3d(pose));
    }

    /**
     * @param chassisSpeeds
     * @return
     */
    public Command driveRobotCentric(ChassisSpeeds chassisSpeeds) {
        return run(() -> this.setControl(visionRobotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)));
    }

    public void driveAuton(ChassisSpeeds chassisSpeeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    public void driveAlign(ChassisSpeeds chassisSpeeds) {
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

    public Command stopRobotCentric() {
        return new InstantCommand(() -> this
                .setControl(new SwerveRequest.RobotCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)));
    }

    public boolean faceTargetEvaluate(double setpoint) {
        return Calc.approxEquals(this.getRobotPose().getRotation().getRadians(),
                Units.degreesToRadians(setpoint), 0.1);
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
        // SmartDashboard.putData("Swerve Pose", (Sendable) this.getRobotPose());

        publisher.set(states);
        posePublisher.set(getRobotPose());
        speedsPublisher.set(getChassisSpeeds());
        updateSimState(0.02, 12.0); // Added so I can use swerve in simulation

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    Field.isBlue()
                            ? kBlueAlliancePerspectiveRotation
                            : kRedAlliancePerspectiveRotation);
            hasAppliedOperatorPerspective = true;
        }

    }

}