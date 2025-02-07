package frc.robot.Subsytems.Cleaner;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.CleanerConstants;
import frc.robot.Util.Constants.CleanerConstants.CleanerModes;
import frc.robot.Util.Constants.CleanerConstants.CleanerStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Cleaner extends REVMechanism {

    private CleanerConfig config;

    private SparkMax motor;
    public SysIdRoutine routine;

    private CleanerModes mode;
    private CleanerStates state;

    public static class CleanerConfig extends Config {

        public CleanerConfig() {
            super("Cleaner", CleanerConstants.id);
            configIdleMode(CleanerConstants.idleMode);
            configInverted(CleanerConstants.isInverted);
            configGearRatio(CleanerConstants.gearRatio);
            configMaxIAccum(CleanerConstants.maxIAccum);
            configMaxMotionPositionMode(CleanerConstants.mm_positionMode);
            configPIDGains(CleanerConstants.p, CleanerConstants.i, CleanerConstants.d);
            configSmartCurrentLimit(CleanerConstants.stallLimit, CleanerConstants.supplyLimit);
            configPeakOutput(CleanerConstants.maxForwardOutput, CleanerConstants.maxReverseOutput);
            configMaxMotion(CleanerConstants.mm_velocity, CleanerConstants.mm_maxAccel, CleanerConstants.mm_error);
        }
    }

    public Cleaner(SparkMax motor, CleanerConfig config, boolean attached) {
        super(motor, attached);

        this.config = config;
        this.motor = motor;
        this.mode = CleanerModes.IDLE;
        this.state = CleanerStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Cleaner/Mode", mode.toString());
        Logger.recordOutput("Cleaner/Setpoint", mode.speed.in(RPM));
        Logger.recordOutput("Cleaner/Output", getMotorOutput());
        Logger.recordOutput("Cleaner/Current", getMotorCurrent());
        Logger.recordOutput("Cleaner/Voltage", getMotorVoltage());
        Logger.recordOutput("Cleaner/Velocity", getMotorVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Cleaner Mode", this.getMode());
        SmartDashboard.putNumber("Cleaner Setpoint", mode.speed.in(RPM));
        SmartDashboard.putNumber("Cleaner Output", getMotorOutput());
        SmartDashboard.putNumber("Cleaner Current", getMotorCurrent());
        SmartDashboard.putNumber("Cleaner Voltage", getMotorVoltage());
        SmartDashboard.putNumber("Cleaner Velocity", getMotorVelocity());

        switch (this.mode) {
            case IDLE:
                setCleanerState(CleanerStates.IDLE);
                break;
            case INTAKE:
                setCleanerState(CleanerStates.INTAKING);
                break;
            case OUTTAKE:
                setCleanerState(CleanerStates.OUTTAKING);
                break;
        }

    }

    public void setCleanerState(CleanerStates cleanerState) {
        this.state = cleanerState;
    }

    protected void runEnum(CleanerModes cleanermode) {
        this.mode = cleanermode;
        setVelocity(cleanermode.speed);
    }

    public Command runCleanerCommand(CleanerModes cleanerModes) {
        return new StartEndCommand(() -> this.runEnum(cleanerModes), () -> this.runEnum(CleanerModes.IDLE), this)
                .withName("Cleaner.runEnum");
    }

    public String getMode() {
        return this.mode.toString();
    }

    public String getCleanerState() {
        return this.state.toString();
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }
}
