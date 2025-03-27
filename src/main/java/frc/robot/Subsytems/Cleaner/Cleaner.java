package frc.robot.Subsytems.Cleaner;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Util.Constants.CleanerConstants;
import frc.robot.Util.Constants.CleanerConstants.CleanerModes;
import frc.robot.Util.Constants.CleanerConstants.CleanerStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

@Deprecated
public class Cleaner extends REVMechanism {

    @Getter private CleanerModes mode;
    @Getter @Setter private CleanerStates state;

    public static class CleanerConfig extends Config {

        public CleanerConfig() {
            super("Intake", CleanerConstants.id);
            configIdleMode(CleanerConstants.idleMode);
            configInverted(CleanerConstants.isInverted);
            configEncoderPosRatio(CleanerConstants.gearRatio);
            configMaxIAccum(CleanerConstants.maxIAccum);
            configMaxMotionPositionMode(CleanerConstants.mm_positionMode);
            configPIDGains(CleanerConstants.p, CleanerConstants.i, CleanerConstants.d);
            configSmartCurrentLimit(CleanerConstants.stallLimit, CleanerConstants.supplyLimit);
            configPeakOutput(CleanerConstants.maxForwardOutput, CleanerConstants.maxReverseOutput);
            configMaxMotion(CleanerConstants.mm_velocity, CleanerConstants.mm_maxAccel, CleanerConstants.mm_error);
        }
    }

    private CleanerConfig config = new CleanerConfig();

    public Cleaner(SparkMax motor, boolean attached){
        super(motor, attached);
        this.setConfig(config);
        
        this.motor = motor;
        this.mode = CleanerModes.IDLE;
        this.state = CleanerStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Cleaner/Mode", getMode());
        Logger.recordOutput("Cleaner/State", getState());
        Logger.recordOutput("Cleaner/Setpoint", getMode().speed.in(RPM));
        Logger.recordOutput("Cleaner/Output", getMotorOutput());
        Logger.recordOutput("Cleaner/Current", getMotorCurrent());
        Logger.recordOutput("Cleaner/Voltage", getMotorVoltage());
        Logger.recordOutput("Cleaner/Velocity", getMotorVelocity());
    }

    @Override
    public void periodic() {
        // SmartDashboard.putString("Cleaner Mode", getMode().toString());
        // SmartDashboard.putString("Cleaner State", getState().toString());
        // SmartDashboard.putNumber("Cleaner Setpoint", getMode().speed.in(RPM));
        // SmartDashboard.putNumber("Cleaner Output", getMotorOutput());
        // SmartDashboard.putNumber("Cleaner Current", getMotorCurrent());
        // SmartDashboard.putNumber("Cleaner Voltage", getMotorVoltage());
        // SmartDashboard.putNumber("Cleaner Velocity", getMotorVelocity());

        // switch (this.mode) {
        //     case IDLE:
        //         setState(CleanerStates.IDLE);
        //         break;
        //     case INTAKE:
        //         setState(CleanerStates.INTAKING);
        //         break;
        //     case OUTTAKE:
        //         setState(CleanerStates.OUTTAKING);
        //         break;
        // }

    }

    protected void runEnum(CleanerModes cleanermode) {
        this.mode = cleanermode;
        setVelocity(cleanermode.speed);
    }

    public Command runCleanerCommand(CleanerModes cleanerModes) {
        return new StartEndCommand(() -> this.runEnum(cleanerModes), () -> this.runEnum(CleanerModes.IDLE), this)
                .withName("Cleaner.runEnum");
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }
}
