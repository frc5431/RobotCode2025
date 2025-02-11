package frc.robot.Subsytems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.ClimberConstants;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.ClimberConstants.ClimberStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Climber extends REVMechanism {

    private ClimberConfig config;
    private SparkMax motor;
    public boolean attached;
    public SysIdRoutine routine;

    private ClimberModes mode;
    private ClimberStates state;

    public static class ClimberConfig extends Config {

        public ClimberConfig() {
            super("Climber", ClimberConstants.id);
            configIdleMode(ClimberConstants.idleMode);
            configInverted(ClimberConstants.isInverted);
            configGearRatio(ClimberConstants.gearRatio);
            configMaxIAccum(ClimberConstants.maxIAccum);
            configMaxMotionPositionMode(ClimberConstants.mm_positionMode);
            configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
            configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
            configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
            configMaxMotion(ClimberConstants.mm_velocity, ClimberConstants.mm_maxAccel, ClimberConstants.mm_error);
        }
    }

    public Climber(SparkMax motor, boolean attached) {
        super(motor, attached);
        ClimberConfig config = new ClimberConfig();
        this.motor = motor;
        this.mode = ClimberModes.STOW;
        this.state = ClimberStates.STOW;
        config.applySparkConfig(motor);

        Logger.recordOutput("Climber/Mode", getMode());
		Logger.recordOutput("Climber/State", getClimberState());
		Logger.recordOutput("Climber/Setpoint", mode.angle.in(Rotations));
		Logger.recordOutput("Climber/Velocity", getMotorVelocity());
		Logger.recordOutput("Climber/Voltage", getMotorVoltage());
		Logger.recordOutput("Climber/Current", getMotorCurrent());
		Logger.recordOutput("Climber/Output", getMotorOutput());
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            config.applySparkConfig(motor);
        }
        return config;
    }
    @Override
    public void periodic() {
        SmartDashboard.putString("Climber Mode", this.getMode());
        SmartDashboard.putString("Climber State", getClimberState());
		SmartDashboard.putNumber("Climber Setpoint", mode.angle.in(Rotations));
		SmartDashboard.putNumber("Climber Velocity", getMotorVelocity());
		SmartDashboard.putNumber("Climber Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Climber Current", getMotorCurrent());
		SmartDashboard.putNumber("Climber Output", getMotorOutput());

        switch (this.mode) {
            case STOW:
                setClimberState(ClimberStates.STOW);
                break;
            case ALIGN:
                setClimberState(ClimberStates.ALIGN);
                break;
            case CLIMB:
                setClimberState(ClimberStates.CLIMB);
                break;
        }

    }

    public void setClimberState(ClimberStates ClimberState) {
        this.state = ClimberState;
    }

    public String getClimberState() {
        return this.state.toString();
    }

    public String getMode() {
        return this.mode.toString();
    }

    protected void runEnum(ClimberModes Climbermode) {
        this.mode = Climbermode;
        setMotorPosition(Climbermode.angle);
    }

    public Command runClimberCommand(ClimberModes climberModes) {
        // return new StartEndCommand(() -> this.runEnum(ClimberModes), () -> this.runEnum(ClimberModes), this).withName("Climber.runEnum");
        return run(() -> runEnum(climberModes)).withName("Climber.runEnum");
    }

    public void alignClimber(){
        runEnum(ClimberModes.ALIGN);
    }
}