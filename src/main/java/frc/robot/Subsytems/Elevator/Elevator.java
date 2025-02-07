package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorStates;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class Elevator extends CTREMechanism {

    public static class ElevatorConfig extends Config {

        public ElevatorConfig() {
            super("Elevator", ElevatorConstants.leftId, Constants.canbus);
            configGravityType(ElevatorConstants.gravityType);
            configFeedForwardGains(ElevatorConstants.s, ElevatorConstants.v, ElevatorConstants.a, ElevatorConstants.g);
            configReverseSoftLimit(ElevatorConstants.maxReverseRotation.in(Rotation),
                    ElevatorConstants.useRMaxRotation);
            configForwardTorqueCurrentLimit(ElevatorConstants.forwardCurrentLimit);
            configReverseTorqueCurrentLimit(ElevatorConstants.reverseCurrentLimit);
            configMotionMagicPosition(ElevatorConstants.ff);
            configFeedbackSensorSource(ElevatorConstants.feedbackSensor, ElevatorConstants.rotationOffset.in(Rotation));
            configGearRatio(ElevatorConstants.gearRatio);
            configNeutralBrakeMode(ElevatorConstants.breakType);
            configPIDGains(ElevatorConstants.p, ElevatorConstants.i, ElevatorConstants.d);
            configForwardSoftLimit(ElevatorConstants.maxFowardRotation.in(Rotation), ElevatorConstants.useFMaxRotation);
        }

    }

    private TalonFX leader;
    private TalonFX follower;

    private ElevatorConfig config;
    private boolean attached;

    private ElevatorPositions position;
    @SuppressWarnings("unused")
    private ElevatorStates states;

    /**
     * @param leader   json
     * @param follower zwach
     * @param attached is the mechanism in use
     */
    public Elevator(TalonFX leader, TalonFX follower, boolean attached) {
        super(leader, attached);
        this.leader = leader;
        this.follower = follower;
        this.attached = attached;
        ElevatorConfig config = new ElevatorConfig();
        this.config = config;
        this.position = ElevatorPositions.STOW;
        this.config.applyTalonConfig(leader);

        if (attached) {
            follower.setControl(new Follower(ElevatorConstants.leftId, ElevatorConstants.follwerInvert));
            Logger.recordOutput("Elevator/Mode", position.toString());
            Logger.recordOutput("Elevator/Setpoint", position.rotation.in(Rotation));
            Logger.recordOutput("Elevator/Setpoint", position.toString());
            Logger.recordOutput("Elevator/Position", leader.getPosition().getValueAsDouble());
            Logger.recordOutput("Elevator/Voltage", leader.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Elevator/Output", leader.getMotorOutputStatus().getValueAsDouble());
            Logger.recordOutput("Elevator/Acceleration", leader.getAcceleration().getValueAsDouble());
            Logger.recordOutput("Elevator/Velocity", leader.getVelocity().getValueAsDouble());

        }
    }

    public void periodic() {
        if (attached) {
            SmartDashboard.putString("Elevator Mode", position.toString());
            SmartDashboard.putNumber("Elevator Setpoint", position.rotation.in(Rotation));
            SmartDashboard.putNumber("Elevator Position", leader.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Position", follower.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Voltage", leader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Output", leader.getMotorOutputStatus().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Acceleration", leader.getAcceleration().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Velocity", leader.getVelocity().getValueAsDouble());
        }
    }

    public void runEnum(ElevatorPositions position) {
        this.position = position;
        setMotorPosition(position.rotation);
    }

    public void runEnumMM(ElevatorPositions position) {
        this.position = position;
        setMMPosition(position.rotation);
    }

    public void runEnumFOC(ElevatorPositions position) {
        this.position = position;
        setMMPositionFOC(position.rotation);
    }

    protected void stop() {
        if (attached) {
            leader.stopMotor();
        }
    }

    protected void setZero() {
        resetPosition();
    }

    public Command runElevatorCommand(ElevatorPositions position) {
        return new RunCommand(() -> runEnum(position), this)
                .withName("Elevator.runEnum");
    }

    public Command runElevatorCommandMM(ElevatorPositions position) {
        return new RunCommand(() -> runEnumMM(position), this)
                .withName("Elevator.runEnumMM");
    }

    public Command runElevatorCommandFOC(ElevatorPositions position) {
        return new RunCommand(() -> runEnumFOC(position), this)
                .withName("Elevator.runEnumFOC");
    }

    public Command stopElevatorCommand(ElevatorPositions position) {
        return new RunCommand(() -> stop(), this)
                .withName("Elevator.STOP");
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            config.applyTalonConfig(leader);
        }
        return this.config;
    }

}
