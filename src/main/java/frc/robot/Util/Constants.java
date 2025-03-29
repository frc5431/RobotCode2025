package frc.robot.Util;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import lombok.Setter;

public final class Constants {

    public static final String canbus = "Omnivore";

    public static class ControllerConstants {
        public enum ControlStates {
            CLEAN, CORAL,
        }

        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final boolean using8BitDo = false;

        public static final double deadzone = 0.15;
        public static final double triggerThreshold = 0.5;

        public static final PresetPosition StowPosition = new PresetPosition(
                ElevatorPositions.STOW, ManipJointPositions.STOW);

        public static final PresetPosition FeedCoralPosition = new PresetPosition(
                ElevatorPositions.FEED, ManipJointPositions.FEED);

        public static final PresetPosition ScoreL2Position = new PresetPosition(ElevatorPositions.CORAL_L2,
                ManipJointPositions.CORAL_L2);

        public static final PresetPosition ScoreL3Position = new PresetPosition(ElevatorPositions.CORAL_L3,
                ManipJointPositions.CORAL_L3);

        public static final PresetPosition ScoreProcessorPosition = new PresetPosition(ElevatorPositions.PROCESSOR,
                ManipJointPositions.PROCESSOR);

        public static final PresetPosition LolipopPosition = new PresetPosition(ElevatorPositions.LOLIPOP,
                ManipJointPositions.LOLIPOP);

        public static final PresetPosition CleanL2Position = new PresetPosition(ElevatorPositions.CLEAN_L2,
                ManipJointPositions.CLEAN_L2);

        public static final PresetPosition CleanL3Position = new PresetPosition(ElevatorPositions.CLEAN_L3,
                ManipJointPositions.CLEAN_L3);

        public static final PresetPosition ScoreL4Position = new PresetPosition(ElevatorPositions.CORAL_L4,
                ManipJointPositions.CORAL_L4);

        public static final PresetPosition EjectL4Position = new PresetPosition(ElevatorPositions.EJECT,
                ManipJointPositions.EJECT);

    }

    public static class AutonConstants {
        // The PID values from last year
        public static final PIDConstants translationPID = new PIDConstants(1.6, 0, 0.1);
        public static final PIDConstants rotationPID = new PIDConstants(2, 0, .0);
    }

    public static class RangeConstants {
        public static final int rightID = 40;
        public static final int leftID = 41;

        public static final int minDist = 2500;
        public static final Distance hyst = Units.Inches.of(0.01);
        public static final Distance threshold = Units.Inches.of(0.01);

        public static final ProximityParamsConfigs proximityConfig = new ProximityParamsConfigs()
                .withMinSignalStrengthForValidMeasurement(minDist)
                .withProximityHysteresis(hyst)
                .withProximityThreshold(threshold);

    }

    public static class IntakeConstants {

        public enum IntakeStates {
            IDLE, INTAKING, FEEDING, OUTTAKING, STUCK,
        }

        public static final boolean attached = true;
        public static final boolean isInverted = true;
        public static final boolean useRpm = false;
        public static final int id = 21;
        public static final double gearRatio = 1;
        public static final Current supplyLimit = Units.Amps.of(30);
        public static final Current stallLimit = Units.Amps.of(45);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0.00065;
        public static final double i = 0.0008;
        public static final double d = 0.00003;
        public static final double maxIAccum = 2 * i;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(3000);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum IntakeModes {
            IDLE(idleSpeed, 0.0), INTAKE(intakeSpeed, 0.95), FEED(feedSpeed, 0.2), OUTTAKE(outtakeSpeed, -0.4);

            public AngularVelocity speed;
            public double output;

            IntakeModes(AngularVelocity speed, double output) {
                this.speed = speed;
                this.output = output;
            }

        }

    }

    public static class CleanerConstants {

        public enum CleanerStates {
            IDLE, INTAKING, OUTTAKING
        }

        public static final boolean attached = false;
        public static final int id = 26;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(30);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;
        public static final double maxIAccum = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);
        public static final AngularVelocity intakeSpeed = Units.RPM.of(70);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);

        public enum CleanerModes {
            IDLE(idleSpeed), INTAKE(intakeSpeed), OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            CleanerModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class CleanPivotConstants {

        public enum CleanPivotStates {
            STOW, L2, L3, NET,
        }

        public static final int id = 25;
        public static final boolean attached = false;
        public static final Angle softLimitForwardMax = Units.Rotation.of(0);
        public static final boolean softLimitEnabled = true;
        public static final Angle intakeAngle = Units.Rotation.of(0);
        public static final Angle softLimitReverseMax = Units.Rotation.of(0);
        public static final Angle stowAngle = Units.Rotation.of(0);
        public static final Angle l2Angle = Units.Rotation.of(0);
        public static final Angle l3Angle = Units.Rotation.of(0);
        public static final Angle netAngle = Units.Rotation.of(0);
        public static final boolean isInverted = false;
        public static final Angle zeroOffset = Units.Rotation.of(0);
        public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public enum CleanPivotModes {
            INTAKE(intakeAngle), STOW(stowAngle), L2(l2Angle), L3(l3Angle), NET(netAngle);

            public Angle angle;

            CleanPivotModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class ManipulatorConstants {

        public enum ManipulatorStates {
            EMPTY, LOCKED, INTAKING, OUTTAKING,
        }

        public static final boolean attached = true;
        public static final int id = 24;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(40);
        public static final Current stallLimit = Units.Amps.of(50);
        public static final double stallCurrent = 37;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = true;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;

        public static final double p = 1;
        public static final double i = 0.01;
        public static final double d = 0.3;
        public static final double maxIAccum = 0.2;

        public static final AngularVelocity scoreSpeed = Units.RPM.of(0);
        public static final AngularVelocity reverseSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipulatorModes {
            IDLE(idleSpeed, 0), SCORE(scoreSpeed, -1), FEED(feedSpeed, 1), MANUAL(feedSpeed,
                    0.4), REVERSE(reverseSpeed, 0.2), SLOWSCORE(scoreSpeed, -0.3);

            public AngularVelocity speed;
            public double output;

            ManipulatorModes(AngularVelocity speed, double output) {
                this.speed = speed;
                this.output = output;
            }

        }

    }

    public static class ElevatorConstants {

        public enum ElevatorStates {
            STOWED, FEED, PROCESSOR, L1, L2, L3, L4, NET,
        }

        public static final boolean attached = true;
        public static final boolean leaderInvert = true;
        public static final boolean follwerInvert = false;
        public static final boolean gravityType = false;
        public static final boolean breakType = true;

        public static final boolean canRangeAttached = false;
        public static final boolean canCoderAttached = false;

        public static final int leftId = 15;
        public static final int rightId = 14;
        public static final int canCoderId = 16;
        public static final double gearRatio = 1 / 1;

        public static final Current forwardTorqueLimit = Units.Amps.of(80);
        public static final Current reverseTorqueLimit = Units.Amps.of(80);

        public static final boolean useStallLimit = true;
        public static final boolean useSupplyLimit = true;
        public static final Current stallLimit = Units.Amps.of(80);
        public static final Current supplyLimit = Units.Amps.of(60);

        public static final double maxForwardOutput = 0.5;
        public static final double maxReverseOutput = -0.5;

        public static final boolean useFMaxRotation = true;
        public static final boolean useRMaxRotation = true;
        public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
        public static final Angle maxFowardRotation = Units.Rotation.of(5);

        public static final FeedbackSensorSourceValue feedbackSensor = FeedbackSensorSourceValue.FusedCANcoder;

        // static voltage needed to hold position
        public static final double s = 0.65;

        // public static final double p = 0.6;
        public static final double p = 1.5;
        public static final double i = 0;
        // do not add d please
        public static final double d = 0;
        public static final double maxIAccum = 0.2;

        public static final Angle error = Units.Rotation.of(.51);
        public static final Angle tightError = Units.Rotation.of(.1);
        public static final Angle lolipop = Units.Rotation.of(0.2);

        public static final Angle coralL1 = Units.Rotation.of(0.5);
        public static final Angle processor = Units.Rotation.of(0.45);

        public static final Angle stow = Units.Rotation.of(0.7);
        public static final Angle feed = Units.Rotation.of(2.12);
        public static final Angle pickup = Units.Rotation.of(1.7);

        public static final Angle cleanL3 = Units.Rotation.of(2.2);

        public static final Angle cleanl2 = Units.Rotation.of(2.12);
        public static final Angle coralL2 = Units.Rotation.of(0.7);
        public static final Angle slamL3 = Units.Rotation.of(1.2);
        public static final Angle coralL3 = Units.Rotation.of(1.8);
        // 2.7
        // lower this to l3 increase speed of feeding
        public static final Angle safeSwing = Units.Rotation.of(2.7);
        public static final Angle rise = Units.Rotation.of(3);

        public static final Angle coralL4 = Units.Rotation.of(4.65);
        public static final Angle lebronShoot = Units.Rotation.of(3.65);
        public static final Angle eject = Units.Rotation.of(4.7);

        public static final Angle coralStation = Units.Rotation.of(3);

        public enum ElevatorPositions {
            STOW(stow), FEED(feed), PROCESSOR(processor), LOLIPOP(lolipop), PICKUP(pickup), CORAL_L1(coralL1), CLEAN_L2(
                    cleanl2), CLEAN_L3(cleanL3), SLAM_L3(slamL3), CORAL_L2(coralL2), CORAL_L3(
                            coralL3), CORAL_L4(coralL4), SAFESWING(safeSwing), RISE(rise), EJECT(eject), LEBRON_SHOOT(lebronShoot);

            public Angle rotation;

            ElevatorPositions(Angle rotation) {
                this.rotation = rotation;
            }

        }

    }

    public static class IntakePivotConstants {

        public enum IntakePivotStates {
            STOW, INTAKING,
        }

        public static final int id = 20;
        public static final boolean attached = true;
        public static final boolean isInverted = true;
        public static final boolean ecoderInverted = false;

        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final Current supplyCurrent = Units.Amp.of(40);
        public static final Current stallCurrent = Units.Amp.of(80);
        public static final double maxForwardOutput = 0.1;
        public static final double maxReverseOutput = -0.3;

        public static final Angle zeroOffset = Units.Rotation.of(0.32);
        public static final Angle softLimitReverseMax = Units.Rotation.of(0.39);
        public static final Angle softLimitForwardMax = Units.Rotation.of(0.8);
        public static final boolean softLimitEnabled = true;

        public static final double ff = 0.1;
        public static final double p = 0.3;
        public static final double i = 0.01;
        public static final double d = 0.3;
        public static final double maxIAccum = 0.1;

        public static final Angle stowAngle = Units.Rotation.of(0.4);
        public static final Angle deployAngle = Units.Rotation.of(0.75);
        public static final Angle scoreL1 = Units.Rotation.of(0.56);

        public enum IntakePivotModes {
            STOW(stowAngle), DEPLOY(deployAngle), L1(scoreL1), NONE(stowAngle);

            public Angle angle;

            IntakePivotModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class DrivebaseConstants {

        public static final PathConstraints constraints = new PathConstraints(0.75, 1, 0.5 * Math.PI, 0.5 * Math.PI);
        public static final PPHolonomicDriveController alignController = new PPHolonomicDriveController(
                new PIDConstants(0.01, 0, 0.6),
                new PIDConstants(0.5, 0, 0.0));
        public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.5);
        public static final Distance robotLength = Units.Inches.of(28);
        public static final AngularVelocity AngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final AngularVelocity VisionAngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final AngularVelocity FaceTargetAngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final LinearVelocity FaceTargetAVelocityDeadzone = TunerConstants.kSpeedAt12Volts.times(0.05);

        public static final Rotation2d rotationTolerance = Rotation2d.fromDegrees(5.0);
        public static final Distance positionTolerance = Units.Inches.of(1);
        public static final LinearVelocity speedTolerance = Units.InchesPerSecond.of(10);
        public static final Time autoAlignPredict = Units.Seconds.of(0.0);
        public static final Time teleopAlignAdjustTimeout = Units.Seconds.of(1);
        public static final Time autoAlignAdjustTimeout = Units.Seconds.of(1);

        public static final AngularVelocity AutoAngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final AngularVelocity AutonMaxAngularRate = RotationsPerSecond.of(0.5);

    }

    public static class VisionConstants {

        public static final boolean useVisionPeriodic = true;

        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;
        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
        // to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI));

        public static final String cameraName = "limelight";
        public static final Distance centLLForwardOffset = Units.Inches.of(-14);
        public static final Distance centLLRightOffset = Units.Inches.of(6.75);
        public static final Distance centLLUpOffset = Units.Inches.of(19);
        public static final Angle centLLRollOffset = Units.Degrees.of(0);
        public static final Angle centLLPitchOffset = Units.Degrees.of(0);
        public static final Angle centLLYawOffset = Units.Degrees.of(180);

        public static final Distance rightLLForwardOffset = Units.Inches.of(0);
        public static final Distance rightLLRightOffset = Units.Inches.of(0);
        public static final Distance rightLLUpOffset = Units.Inches.of(0);
        public static final Angle rightLLRollOffset = Units.Degrees.of(0);
        public static final Angle rightLLPitchOffset = Units.Degrees.of(0);
        public static final Angle rightLLYawOffset = Units.Degrees.of(0);

        public static final int centerTagPipeline = 0;
        public static final int rightTagPipeline = 0;

        public static final Distance rightPipeOffset = Units.Inches.of(6);
        public static final Distance leftPipeOffset = Units.Inches.of(6);
        public static final Distance pipeScoreOffset = Units.Inches.of(6);
        public static final Distance centerScoreOffset = Units.Inches.of(2);
        public static final Distance centerOffset = Units.Inches.of(2);
        public static final Distance allowedError = Units.Inches.of(1);

        public static final LinearVelocity alignYVelocity = Units.FeetPerSecond.of(1 / 1);
        public static final LinearVelocity alignXVelocity = Units.FeetPerSecond.of(1 / 1);
        public static final AngularVelocity alignThetaVelocity = Units.RadiansPerSecond.of(0.0);
        public static final ChassisSpeeds alignXSpeed = new ChassisSpeeds(alignXVelocity, Units.FeetPerSecond.of(0),
                alignThetaVelocity);
        public static final ChassisSpeeds alignYSpeed = new ChassisSpeeds(Units.FeetPerSecond.of(0), alignYVelocity,
                alignThetaVelocity);

        public static final double highTrustStds = 0.1;
        public static final double servicableTrustStds = 0.25;
        public static final double defaultTrustStds = 0.5;
        public static final double decreasedTrustStds = 2;

        public static final double lowTrustStds = 5;
        public static final double badTrustStds = 8;
        public static final double dismalTrustStds = 15;
        public static final double abysmalTrustStds = 16;
        public static final double noTrustStds = 9999;

        /**
         * idk what unit this is
         * spectrum did 0.025
         */
        public static final double minSizeRejection = 0.025;
        public static final AngularVelocity maxRadPerSec = Units.RadiansPerSecond.of(1);
        public static final AngularVelocity lowTrustRadPerSec = Units.RadiansPerSecond.of(0.5);

        public static final Distance visionRejectDistance = Units.Meters.of(1);
        /**
         * Meters per Second
         */
        public static final double velocityLowTrustThreshold = 0.2;

    }

    public static class ManipJointConstants {

        public enum ManipJointStates {
            STOWED, FEED, HUMAN, L1, L2, L3, L4
        }

        public static final boolean attached = true;
        public static final int id = 23;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(40);
        public static final Current stallLimit = Units.Amps.of(80);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = true;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kAlternateOrExternalEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -0.08;

        // calculate WITH coral,
        // 0.15 output units
        public static final double s = 0.3; // 0.15 holds arm at 90 degree position, when gravity's pull is strongest

        public static final double p = 2.5;
        public static final double i = 0.001;
        public static final double d = 1.7;
        public static final double maxIAccum = 0.005;

        public static final Angle eject = Units.Rotation.of(-0.8);
        public static final Angle stow = Units.Rotations.of(-2);
        public static final Angle scoreL1 = Units.Rotations.of(-3.5);
        public static final Angle scoreL2 = Units.Rotations.of(-3.2);
        public static final Angle slamL2 = Units.Rotations.of(-3.6);
        public static final Angle scoreL3 = Units.Rotations.of(-1.5);
        public static final Angle slamL3 = Units.Rotations.of(-4);
        public static @Setter Angle adjustAngle = Units.Rotations.of(-2);
        public static final Angle lolipop = Units.Rotations.of(-7);

        public static final Angle cleanAlgea = Units.Rotations.of(-2.5);
        public static final Angle processorAngle = Units.Rotations.of(-7.6);
        public static final Angle safeSwing = Units.Rotations.of(-11);
        public static final Angle prefeed = Units.Rotations.of(-13);
        public static final Angle feed = Units.Rotations.of(-14);
        public static final Angle scoreL4 = Units.Rotations.of(-3.7);
        public static final Angle slamL4 = Units.Rotations.of(-5);

        public static final Angle straightUp = Units.Rotation.of(-0.5);
        public static final Angle error = Units.Rotations.of(0.3);
        public static final Angle tightError = Units.Rotations.of(0.1);

        public enum ManipJointPositions {
            STOW(stow), EJECT(eject), PREEFEED(prefeed), LOLIPOP(lolipop), FEED(feed), CLEAN_L2(cleanAlgea), CLEAN_L3(
                    cleanAlgea), PROCESSOR(processorAngle), CORAL_L1(scoreL1), CORAL_L2(scoreL2), SLAM_L2(slamL2), CORAL_L3(
                            scoreL3), SLAM_L3(slamL3), ADJUSTANGLE(adjustAngle), CORAL_L4(scoreL4), STRAIGHT_UP(straightUp);

            public Angle position;

            ManipJointPositions(Angle position) {
                this.position = position;
            }

        }

    }

    public static class FeederConstants {

        public enum FeederStates {
            IDLE, STUCK, REVERSE, FEEDING,
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final boolean useRPM = false;
        public static final int id = 22;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(40);
        public static final Current stallLimit = Units.Amps.of(80);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;

        public static final double p = 1;
        public static final double i = 0.00;
        public static final double d = 0.3;
        public static final double maxIAccum = 0.2;

        public static final AngularVelocity reverseSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final double idleOutput = 0.0;
        public static final double reverseOutput = -0.7;
        public static final double feedOutput = 0.8;
        public static final double slowFeedOutput = 0.3;

        public enum FeederModes {
            IDLE(idleSpeed, idleOutput), REVERSE(reverseSpeed, reverseOutput), FEED(feedSpeed,
                    feedOutput), SLOW(feedSpeed, slowFeedOutput);

            public AngularVelocity speed;
            public double output;

            FeederModes(AngularVelocity speed, Double output) {
                this.speed = speed;
                this.output = output;

            }

        }
    }

    public static class ClimberConstants {

        public enum ClimberStates {
            STOW, ALIGN, CLIMB
        }

        public static final int id = 27;
        public static final boolean attached = false;
        public static final boolean isInverted = false;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(30);
        public static final Current stallLimit = Units.Amps.of(80);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0.5;
        public static final double maxReverseOutput = 0.5;

        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 1;
        public static final double i = 0.01;
        public static final double d = 0.3;
        public static final double maxIAccum = 0.2;

        public static final AngularVelocity climbVelocity = Units.RPM.of(0);
        public static final Angle stow_angle = Units.Rotation.of(0);
        public static final Angle align_angle = Units.Rotation.of(0);
        public static final Angle climb_angle = Units.Rotation.of(0);
        public static final Angle error = Units.Rotation.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ClimberModes {
            STOW(stow_angle), ALIGN(align_angle), CLIMB(climb_angle);

            public Angle angle;

            ClimberModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class CANdleConstants {
        public static final boolean attached = true;
        public static final int id = 18;

        public static final double fast = 0.8;
        public static final double medium = 0.5;
        public static final double slow = 0.2;

        // Team Colors
        public static final Color darkBlue = new Color(0, 0, 200);
        public static final Color cyanish = new Color(13, 228, 252);
        public static final Color purple = new Color(270, 0, 217);

        // Game Piece Colors
        public static final Color algaeGreen = new Color(69, 206, 162);
        public static final Color coralWhite = new Color(255, 230, 220);

        // Indicator Colors
        public static final Color green = new Color(56, 209, 0);
        public static final Color blue = new Color(8, 32, 255);
        public static final Color red = new Color(255, 0, 0);
        public static final Color yellow = new Color(252, 186, 3);

        // Misc
        public static final Color black = new Color(0, 0, 0);
        public static final Color orange = new Color(255, 25, 0);

        // Animations
        public enum AnimationTypes {
            CORAL(coralWhite, medium), ALGAE(algaeGreen, medium), BOTH(cyanish, medium), SLOW_WHITE(coralWhite,
                    slow), FLASHING_ORANGE(orange, fast), STRESS_TIME(yellow,
                            medium), BLINK_RED(red, medium), BLINK_BLUE(blue, medium), FLASHING_GREEN(green, fast);

            public Color color;
            public double speed;

            AnimationTypes(Color color, double speed) {
                this.color = color;
                this.speed = speed;
            }
        }
    }

}
