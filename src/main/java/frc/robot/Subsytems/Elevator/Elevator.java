package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorStates;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends CTREMechanism {

    public static class ElevatorConfig extends Config {

        public ElevatorConfig() {
            super("Elevator", ElevatorConstants.leftId, Constants.canbus);
            configPIDGains(ElevatorConstants.p, ElevatorConstants.i,
                    ElevatorConstants.d);
            configNeutralBrakeMode(ElevatorConstants.breakType);
            configMotionMagicPosition(ElevatorConstants.s);
            configMotionMagic(ElevatorConstants.cruiseVelocity, ElevatorConstants.acceleration, ElevatorConstants.jerk);
            configFeedbackSensorSource(ElevatorConstants.feedbackSensor);
            configGearRatio(ElevatorConstants.gearRatio);
            configGravityType(ElevatorConstants.gravityType);
            configSupplyCurrentLimit(ElevatorConstants.supplyLimit, ElevatorConstants.useSupplyLimit);
            configStatorCurrentLimit(ElevatorConstants.stallLimit, ElevatorConstants.useStallLimit);
            configReverseSoftLimit(ElevatorConstants.maxReverseRotation.in(Rotation),
                    ElevatorConstants.useRMaxRotation);
            configForwardSoftLimit(ElevatorConstants.maxFowardRotation.in(Rotation), ElevatorConstants.useFMaxRotation);
        }

    }

    public TalonFX leader;
    private TalonFX follower;
    private CANcoder elevatorCANcoder;
    private boolean attached;

    @Getter
    private ElevatorPositions position;
    @Getter
    @Setter
    private ElevatorStates states;

    /**
     * @param leader
     *            json
     * @param follower
     *            zwach
     * @param attached
     *            is the mechanism in use
     */
    public Elevator(TalonFX leader, TalonFX follower, boolean attached) {
        super(leader, attached);

        this.leader = leader;
        this.follower = follower;
        this.attached = attached;
        config.talonConfig.Slot0.kS = ElevatorConstants.s;
        config.talonConfig.Feedback.withFeedbackRemoteSensorID(ElevatorConstants.canCoderId);
        config.applyTalonConfig(leader);
        elevatorCANcoder = new CANcoder(ElevatorConstants.canCoderId, Constants.canbus);
        follower.setControl(new Follower(ElevatorConstants.leftId, ElevatorConstants.follwerInvert));

        this.position = ElevatorPositions.STOW;
        this.states = ElevatorStates.STOWED;
        if (attached) {
            Logger.recordOutput("Elevator/Mode", getPosition());
            Logger.recordOutput("Elevator/States", getStates());
            Logger.recordOutput("Elevator/Setpoint", getPosition().rotation.in(Rotation));
            Logger.recordOutput("Elevator/Position", leader.getPosition().getValueAsDouble());
            Logger.recordOutput("Elevator/Voltage", leader.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Elevator/Output", leader.getMotorOutputStatus().getValueAsDouble());
            Logger.recordOutput("Elevator/Acceleration", leader.getAcceleration().getValueAsDouble());
            Logger.recordOutput("Elevator/Velocity", leader.getVelocity().getValueAsDouble());
        }

    }

    public void periodic() {
        if (attached) {
            SmartDashboard.putString("Elevator Mode", getPosition().toString());
            SmartDashboard.putString("Elevator State", getStates().toString());
            SmartDashboard.putNumber("Elevator Setpoint", getPosition().rotation.in(Rotation));
            SmartDashboard.putNumber("CANCoder Position", elevatorCANcoder.getPosition().getValueAsDouble());
            //SmartDashboard.putNumber("Follower Position", follower.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Voltage", leader.getMotorVoltage().getValueAsDouble());
            //SmartDashboard.putNumber("Elevator Refrence", leader.getClosedLoopReference().getValueAsDouble());
            SmartDashboard.putBoolean("Elevator Goal",
                    getPositionSetpointGoal(getPosition().rotation, ElevatorConstants.error));
            SmartDashboard.putNumber("Elevator Output", leader.getClosedLoopOutput().getValueAsDouble());
            //SmartDashboard.putNumber("Elevator Leader Output", leader.getBridgeOutput().getValueAsDouble());
            SmartDashboard.putBoolean("Elevator Safe Swing", isSwingSafe());

            //SmartDashboard.putNumber("Elevator Acceleration", leader.getAcceleration().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Velocity", leader.getVelocity().getValueAsDouble());

            // if (ElevatorConstants.canCoderAttached) {
            //     SmartDashboard.putNumber("Elevator/CANCoderPosition",
            //             elevatorCANcoder.getPosition().getValueAsDouble());
            // }

        }
    }

    /**
     * Checks if the motor is reaching the rotational setpoint
     * 
     * @param target
     *            the target rotation angle
     * @param error
     *            allowed error in rotations (keep SMALL)
     * @return true if the motor's angle position is within the error of the target
     *         angle position
     */
    public boolean getPositionSetpointGoal(Angle target, Angle error) {
        if (attached) {
            if (Calc.approxEquals(elevatorCANcoder.getPosition().getValue().in(Rotation), target.in(Rotation),
                    error.in(Rotation))) {
                return true;
            }
        }
        return false;
    }

    public Angle getElevatorPosition() {
        return elevatorCANcoder.getPosition().getValue();
    }

    /**
     * @return if above or equal to the safe swing distance in rotations
     */
    public boolean isSwingSafe() {
        if (attached) {
            return leader.getRotorPosition().getValue().gte(ElevatorConstants.rise);
        }
        return false;
    }

    public void stupid() {

    }

    public void runEnum(ElevatorPositions position) {
        this.position = position;
        leader.setControl(new MotionMagicExpoDutyCycle(position.rotation));
        // setMMPosition(position.rotation);
    }

    public void runEnumMM(ElevatorPositions position) {
        this.position = position;
        setMMPosition(position.rotation);
    }

    public void runEnumFOC(ElevatorPositions position) {
        this.position = position;
        setMMPositionFOC(position.rotation);
    }

    public void riseAboveFriction() {
        this.position = ElevatorPositions.STOW;
        setMMPositionFOC(position.rotation);
    }

    public Command zeroElevatorCommand() {
        return new InstantCommand(() -> setZero(), this);
    }

    protected void stop() {
        if (attached) {
            leader.stopMotor();
            follower.stopMotor();
        }
    }

    protected void setZero() {
        if (attached) {
            motor.setPosition(Revolutions.of(0));
        }
    }

    public Command runElevatorCommand(ElevatorPositions position) {
        return new InstantCommand(() -> runEnum(position), this)
                .withName("Elevator.runEnum");
    }

    public Command riseElevatorCommand() {
        Command rise = new InstantCommand(() -> runEnum(ElevatorPositions.RISE), this);
        return rise;
    }

    public Command runElevatorCommandMM(ElevatorPositions position) {
        return new InstantCommand(() -> runEnumMM(position), this)
                .withName("Elevator.runEnumMM");
    }

    public Command runElevatorCommandFOC(ElevatorPositions position) {
        return new InstantCommand(() -> runEnumFOC(position), this)
                .withName("Elevator.runEnumFOC");
    }

    public Command killElevatorCommand() {
        return new RunCommand(() -> stop(), this)
                .withName("KILL ELEVATOR");
    }

    public Command stopElevatorCommand(ElevatorPositions position) {
        return new InstantCommand(() -> stop(), this)
                .withName("Elevator.STOP");
    }

    @Override
    protected Config setConfig() {
        this.config = new ElevatorConfig();
        return config;
    }

}
