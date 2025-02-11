package frc.robot.Util;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class Constants {

    public static final String canbus = "Omnivore";

    public static class ControllerConstants {

        public enum ControlStates {
            ALGAE,
            CORAL,
        }

        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final double deadzone = 0.15;

    }

    public static class IntakeConstants {

        public enum IntakeStates {
            IDLE,
            INTAKING,
            FEEDING,
            OUTTAKING,
            STUCK,
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final int id = 0;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum IntakeModes {
            IDLE(idleSpeed),
            INTAKE(intakeSpeed),
            FEED(feedSpeed),
            OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            IntakeModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class CleanerConstants {


        public enum CleanerStates {
            IDLE,
            INTAKING,
            OUTTAKING
        }

        public static final boolean attached = true;
        public static final int id = -124542;
        public static final double gearRatio = 0 / 0;
        public static final Current supplyLimit = Units.Amps.of(0);
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
        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);

        public enum CleanerModes {
            IDLE(idleSpeed),
            INTAKE(intakeSpeed),
            OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            CleanerModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class CleanPivotConstants {

        public enum CleanPivotStates {
            STOW,
            L2,
            L3,
            NET,
        }

        public static final int id = 32342370;
        public static final boolean attached = true;
        public static final Angle softLimitForwardMax = Units.Rotation.of(0);
        public static final boolean softLimitEnabled = true;
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
            STOW(stowAngle),
            L2(l2Angle),
            L3(l3Angle),
            NET(netAngle);

            public Angle angle;

            CleanPivotModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class ManipulatorConstants {

        public enum ManipulatorStates {
            IDLE,
            LOCKED,
            INTAKING,
            OUTTAKING,
        }

        public static final boolean attached = true;
        public static final int id = 1;
        public static final double gearRatio = 1/1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipulatorModes {
            IDLE(idleSpeed),
            INTAKE(intakeSpeed),
            FEED(feedSpeed),
            OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            ManipulatorModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class ElevatorConstants {

        public enum ElevatorStates {

        }

        public static final boolean attached = true;
        public static final boolean leaderInvert = false;
        public static final boolean follwerInvert = true;
        public static final boolean gravityType = false;
        public static final boolean breakType = true;
        public static final boolean useFMaxRotation = true;
        public static final boolean useRMaxRotation = true;
        public static final boolean canRangeAttached = true;

        public static final int leftId = 14;
        public static final int rightId = 15;
        public static final int canCoderId = 00;
        public static final int canRangeId = 16;
        public static final double gearRatio = 1 / 1;
        public static final Current forwardCurrentLimit = Units.Amps.of(0);
        public static final Current reverseCurrentLimit = Units.Amps.of(0);

        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;
        public static final Angle maxReverseRotation = Units.Rotation.of(0);
        public static final Angle maxFowardRotation = Units.Rotation.of(0);
        public static final Angle rotationOffset = Units.Rotation.of(0);

        public static final FeedbackSensorSourceValue feedbackSensor = FeedbackSensorSourceValue.FusedCANcoder;

        public static final double s = 0;
        public static final double v = 0;
        public static final double a = 0;
        public static final double g = 0;
        public static final double ff = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow = Units.Rotation.of(0);
        public static final Angle feed = Units.Rotation.of(0);
        public static final Angle algaeProcessor = Units.Rotation.of(0);
        public static final Angle coralL1 = Units.Rotation.of(0);
        public static final Angle coralL2 = Units.Rotation.of(0);
        public static final Angle algaeL2 = Units.Rotation.of(0);
        public static final Angle coralL3 = Units.Rotation.of(0);
        public static final Angle algaeL3 = Units.Rotation.of(0);
        public static final Angle coralL4 = Units.Rotation.of(0);
        public static final Angle coralStation = Units.Rotation.of(0);
        public static final Angle net = Units.Rotation.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ElevatorPositions {
            STOW(stow),
            FEED(feed),
            PROCESSOR(algaeProcessor),
            CORALL1(coralL1),
            CORALL2(coralL2),
            ALGAEL2(algaeL2),
            CORALL3(coralL3),
            ALGAEL3(algaeL3),
            CORALL4(coralL4),
            NET(net);

            public Angle rotation;

            ElevatorPositions(Angle rotation) {
                this.rotation = rotation;
            }

        }

    }

    public static class ManipJointConstants {

        public enum ManipJointStates {

        }

        public static final boolean attached = true;
        public static final int id = 1;
        public static final double gearRatio = 0 / 0;
        public static final Current supplyLimit = Units.Amps.of(30);
        public static final Current stallLimit = Units.Amps.of(50);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow = Units.Rotations.of(0);
        public static final Angle feed = Units.Rotations.of(0);
        public static final Angle scoreL1 = Units.Rotations.of(0);
        public static final Angle scoreL2 = Units.Rotations.of(0);
        public static final Angle scoreL3 = Units.Rotations.of(0);
        public static final Angle scoreL4 = Units.Rotations.of(0);
        public static final Angle error = Units.Rotations.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipJointPositions {
            STOW(stow),
            FEED(feed),
            SCOREL1(scoreL1),
            SCOREL2(scoreL2),
            SCOREL3(scoreL3),
            SCOREL4(scoreL4);

            public Angle position;

            ManipJointPositions(Angle position) {
                this.position = position;
            }

        }

    }

    public static class ClimberConstants {

        public enum ClimberStates {
            STOW,
            ALIGN,
            CLIMB
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final int id = 2;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow_angle = Units.Rotation.of(0);
        public static final Angle align_angle = Units.Rotation.of(0);
        public static final Angle climb_angle = Units.Rotation.of(0);
        public static final Angle error = Units.Rotation.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ClimberModes {
            STOW(stow_angle),
            ALIGN(align_angle),
            CLIMB(climb_angle);

            public Angle angle;

            ClimberModes(Angle angle) {
                this.angle = angle;
            }

        }

    }
}
