package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.IntakeConstants;
import frc.robot.Util.Constants.IntakePivotConstants;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotStates;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class IntakePivot extends REVMechanism {

	private SparkMax motor;
	private AbsoluteEncoder absoluteEncoder;
	private boolean attached;

	private @Getter IntakePivotModes mode;
	private @Getter @Setter IntakePivotStates state;

	public static class PivotConfig extends Config {

		public PivotConfig() {
			super("IntakePivot", IntakePivotConstants.id);
			configPeakOutput(IntakePivotConstants.maxForwardOutput, IntakePivotConstants.maxReverseOutput);
			configInverted(IntakePivotConstants.isInverted);
			configAbsoluteEncoderInverted(!IntakeConstants.isInverted);
			configIdleMode(IntakePivotConstants.idleMode);
			configSmartCurrentLimit(IntakePivotConstants.stallCurrent, IntakePivotConstants.supplyCurrent);
			configMaxIAccum(IntakePivotConstants.maxIAccum);
			configFeedbackSensorSource(IntakePivotConstants.feedbackSensor, IntakePivotConstants.zeroOffset);
			configPIDGains(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
		}
	}
	private PivotConfig config = new PivotConfig();

	public IntakePivot(SparkMax motor, boolean attached) {
		super(motor, attached);
		this.attached = attached;
		this.absoluteEncoder = motor.getAbsoluteEncoder();
		this.motor = motor;
		this.mode = IntakePivotModes.STOW;
		this.state = IntakePivotStates.STOW;

		config.applySparkConfig(motor);

		Logger.recordOutput("Intake/Pivot/Mode", getMode());
		Logger.recordOutput("Intake/Pivot/State", getState());
		Logger.recordOutput("Intake/Pivot/Setpoint", getMode().angle.in(Rotation));
		Logger.recordOutput("Intake/Pivot/Output", getMotorOutput());
		Logger.recordOutput("Intake/Pivot/Position", absoluteEncoder.getPosition());
		Logger.recordOutput("Intake/Pivot/Current", getMotorCurrent());
		Logger.recordOutput("Intake/Pivot/Voltage", getMotorVoltage());
		Logger.recordOutput("Intake/Pivot/Velocity", getMotorVelocity());
	}

	public void periodic() {
		SmartDashboard.putString("Intake Pivot Mode", getMode().toString());
		//SmartDashboard.putString("Intake Pivot State", getState().toString());
		//SmartDashboard.putBoolean("ManipJoint Goal", getAngleSetpointGoal(getMode().angle, ManipJointConstants.error));
		SmartDashboard.putNumber("Intake Pivot Setpoint", getMode().angle.in(Rotation));
		SmartDashboard.putNumber("Intake Pivot Output", getMotorOutput());
		SmartDashboard.putNumber("Intake Pivot Position", absoluteEncoder.getPosition());
		//SmartDashboard.putNumber("Intake Pivot Current", getMotorCurrent());
		//SmartDashboard.putNumber("Intake Pivot Voltage", getMotorVoltage());
		//SmartDashboard.putNumber("Intake Pivot Velocity", getMotorVelocity());

	}

	protected void runEnum(IntakePivotModes intakePivotModes) {
		this.mode = intakePivotModes;
		setMotorPosition(intakePivotModes.angle);
	}

	protected void runEnumMM(IntakePivotModes intakePivotModes) {
		this.mode = intakePivotModes;
		setMMPosition(intakePivotModes.angle);
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public Command runIntakePivotCommand(IntakePivotModes intakePivotModes) {
		return new InstantCommand(() -> this.runEnum(intakePivotModes), this)
				.withName("IntakePivot.runEnum");
	}

	public Command runIntakePivotCommandMM(IntakePivotModes intakePivotModes) {
		return new InstantCommand(() -> this.runEnumMM(intakePivotModes), this)
				.withName("IntakePivot.runEnumMM");
	}

	public Command killIntakeCommand() {
		return new RunCommand(() -> this.stop(), this)
				.withName("KILL INTAKE PIVOT");
	}

	public Command IntakePivotResetPositionCommand() {
		return new InstantCommand(() -> this.setZero(), this)
				.withName("IntakePivot.setZero");
	}

	@Override
	protected Config setConfig() {
		if (attached) {
			setConfig(config);
		}
		return this.config;
	}

}
