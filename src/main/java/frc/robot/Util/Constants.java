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
