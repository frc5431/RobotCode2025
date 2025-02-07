package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.RPM;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.IntakeConstants;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Intake extends REVMechanism {

    private IntakeConfig config;
    private SparkMax motor;
    public boolean attachted;
    public SysIdRoutine routine;

    private IntakeModes mode;
    private IntakeStates state;

    public static class IntakeConfig extends Config {

        public IntakeConfig() {
            super("Intake", IntakeConstants.id);
            configIdleMode(IntakeConstants.idleMode);
            configInverted(IntakeConstants.isInverted);
            configGearRatio(IntakeConstants.gearRatio);
            configMaxIAccum(IntakeConstants.maxIAccum);
            configMaxMotionPositionMode(IntakeConstants.mm_positionMode);
            configPIDGains(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d);
            configSmartCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.supplyLimit);
            configPeakOutput(IntakeConstants.maxForwardOutput, IntakeConstants.maxReverseOutput);
            configMaxMotion(IntakeConstants.mm_velocity, IntakeConstants.mm_maxAccel, IntakeConstants.mm_error);
        }
    }

    public Intake(SparkMax motor, boolean attachted) {
        super(motor, attachted);
        System.out.println("INTAKE IS ALIVE");
        IntakeConfig config = new IntakeConfig();
        this.motor = motor;
        this.mode = IntakeModes.IDLE;
        this.state = IntakeStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Intake/Rollers/Mode", getMode());
        Logger.recordOutput("Intake/Rollers/Setpoint", mode.speed.in(RPM));
        Logger.recordOutput("Intake/Rollers/Velocity", getIntakeState());
        Logger.recordOutput("Intake/Rollers/Velocity", getMotorVelocity());
        Logger.recordOutput("Intake/Rollers/Voltage", getMotorVoltage());
        Logger.recordOutput("Intake/Rollers/Current", getMotorCurrent());
        Logger.recordOutput("Intake/Rollers/Output", getMotorOutput());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Mode", getMode());
        SmartDashboard.putNumber("Intake Setpoint", mode.speed.in(RPM));
        SmartDashboard.putNumber("Intake Output", getMotorOutput());
        SmartDashboard.putNumber("Intake Current", getMotorCurrent());
        SmartDashboard.putNumber("Intake Voltage", getMotorVoltage());
        SmartDashboard.putNumber("Intake Velocity", getMotorVelocity());

        switch (this.mode) {
            case IDLE:
                setIntakeState(IntakeStates.IDLE);
                break;
            case INTAKE:
                setIntakeState(IntakeStates.INTAKING);
                break;
            case OUTTAKE:
                setIntakeState(IntakeStates.OUTTAKING);
                break;
            case FEED:
                setIntakeState(IntakeStates.FEEDING);
                break;
        }

    }

    public void setIntakeState(IntakeStates intakeState) {
        this.state = intakeState;
    }

    protected void runEnum(IntakeModes intakemode) {
        this.mode = intakemode;
        setVelocity(intakemode.speed);
    }

    public Command runIntakeCommand(IntakeModes intakeModes) {
        return new StartEndCommand(() -> this.runEnum(intakeModes), () -> this.runEnum(IntakeModes.IDLE), this)
                .withName("Intake.runEnum");
    }


    public String getMode() {
        return this.mode.toString();
    }

    public String getIntakeState() {
        return this.state.toString();
    }

    @Override
    protected Config setConfig() {
        if (attachted) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }

}