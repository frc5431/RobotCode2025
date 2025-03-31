package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.FeederConstants;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.FeederConstants.FeederStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class Feeder extends REVMechanism {

    private SparkMax motor;
    public boolean attachted;
    public SysIdRoutine routine;

    private @Getter FeederModes mode;
    private @Getter @Setter FeederStates state;

    public static class FeederConfig extends Config {

        public FeederConfig() {
            super("Feeder", FeederConstants.id);
            configIdleMode(FeederConstants.idleMode);
            configInverted(FeederConstants.isInverted);
            configEncoderPosRatio(FeederConstants.gearRatio);
            configMaxIAccum(FeederConstants.maxIAccum);
            configPIDGains(FeederConstants.p, FeederConstants.i, FeederConstants.d);
            configSmartCurrentLimit(FeederConstants.stallLimit, FeederConstants.supplyLimit);
            configPeakOutput(FeederConstants.maxForwardOutput, FeederConstants.maxReverseOutput);
        }
    }
    
    private FeederConfig config = new FeederConfig();

    public Feeder(SparkMax motor, boolean attachted) {
        super(motor, attachted);
        FeederConfig config = new FeederConfig();
        this.motor = motor;
        this.mode = FeederModes.IDLE;
        this.state = FeederStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Feeder/Rollers/Mode", getMode());
        Logger.recordOutput("Feeder/Rollers/State", getState());
        Logger.recordOutput("Feeder/Rollers/Setpoint", getMode().speed.in(RPM));
        Logger.recordOutput("Feeder/Rollers/Velocity", getState());
        Logger.recordOutput("Feeder/Rollers/Velocity", getMotorVelocity());
        Logger.recordOutput("Feeder/Rollers/Voltage", getMotorVoltage());
        Logger.recordOutput("Feeder/Rollers/Current", getMotorCurrent());
        Logger.recordOutput("Feeder/Rollers/Output", getMotorOutput());
    }

    @Override
    public void periodic() {
        //SmartDashboard.putString("Feeder State", getState().toString());
        SmartDashboard.putString("Feeder Mode", getMode().toString());
        //SmartDashboard.putNumber("Feeder Setpoint", getMode().speed.in(RPM));
        SmartDashboard.putNumber("Feeder Output", getMotorOutput());
        // SmartDashboard.putNumber("Feeder Current", getMotorCurrent());
        // SmartDashboard.putNumber("Feeder Voltage", getMotorVoltage());
        // SmartDashboard.putNumber("Feeder Velocity", getMotorVelocity());

    }

    public void runEnum(FeederModes feederModes, boolean rpm) {
        this.mode = feederModes;
        if (rpm) {
            setVelocity(feederModes.speed);
        } else {
            setPercentOutput(feederModes.output);
        }
    }

    public Command runFeederCommand(FeederModes feederModes) {
        return new RunCommand(() -> this.runEnum(feederModes, FeederConstants.useRPM), this)
                .withName("Intake.runEnum");
    }

    public Command runFeederCommand(FeederModes feederModes, boolean rpm) {
        return new RunCommand(() -> this.runEnum(feederModes, rpm), this)
                .withName("Intake.runEnum");
    }

    protected void stop() {
        if (attached) {
            motor.stopMotor();
        }
    }
    
    @Override
    protected Config setConfig() {
        if (attachted) {
            setConfig(config);
       }
        return this.config;
    }

}
