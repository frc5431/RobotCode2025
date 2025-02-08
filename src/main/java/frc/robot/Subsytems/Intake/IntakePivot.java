package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.IntakePivotConstants;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class IntakePivot extends REVMechanism {

	public SparkMax motor;
	public SparkClosedLoopController controller;
	public AbsoluteEncoder absoluteEncoder;
	public IntakePivotModes mode;
	public IntakePivotStates state;
	public double massKg;
	public boolean isShooter;

	public static class PivotConfig extends Config {

		public PivotConfig() {
			super("IntakePivot", IntakePivotConstants.id);
			configSoftLimit(IntakePivotConstants.softLimitEnabled, IntakePivotConstants.softLimitForwardMax,
					IntakePivotConstants.softLimitReverseMax);
			configInverted(IntakePivotConstants.isInverted);
			configFeedbackSensorSource(IntakePivotConstants.feedbackSensor, IntakePivotConstants.zeroOffset);
			configPIDGains(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
		}
	}

	public IntakePivot(PivotConfig config, SparkMax motor, boolean attached) {
		super(motor, attached);

		this.config = config;
		this.motor = motor;
		this.mode = IntakePivotModes.STOW;
		this.state = IntakePivotStates.STOWED;

		config.applySparkConfig(motor);

		Logger.recordOutput("Intakeer/Pivot/Mode", mode.toString());
		Logger.recordOutput("Intakeer/Pivot/Mode", mode.angle.in(Rotation));
		Logger.recordOutput("Intakeer/Pivot/Output", getMotorOutput());
		Logger.recordOutput("Intakeer/Pivot/Position", absoluteEncoder.getPosition());
		Logger.recordOutput("Intakeer/Pivot/Current", getMotorCurrent());
		Logger.recordOutput("Intakeer/Pivot/Voltage", getMotorVoltage());
		Logger.recordOutput("Intakeer/Pivot/Velocity", getMotorVelocity());
	}

	public void periodic() {
		SmartDashboard.putString("Intakeer Pivot Mode", mode.toString());
		SmartDashboard.putNumber("Intakeer Pivot Mode", mode.angle.in(Rotation));
		SmartDashboard.putNumber("Intakeer Pivot Output", getMotorOutput());
		SmartDashboard.putNumber("Intakeer Pivot Position", absoluteEncoder.getPosition());
		SmartDashboard.putNumber("Intakeer Pivot Current", getMotorCurrent());
		SmartDashboard.putNumber("Intakeer Pivot Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Intakeer Pivot Velocity", getMotorVelocity());
	}

	public void setManipJointState(IntakePivotStates IntakeerPivotStates) {
		this.state = IntakeerPivotStates;
	}

	protected void runEnum(IntakePivotModes IntakePivotModes) {
		this.mode = IntakePivotModes;
		setMotorPosition(IntakePivotModes.angle);
	}

	protected void runEnumMM(IntakePivotModes IntakePivotModes) {
		this.mode = IntakePivotModes;
		setMMPosition(IntakePivotModes.angle);
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public Command runIntakeerPivotCommand(IntakePivotModes IntakePivotModes) {
		return new RunCommand(() -> this.runEnum(IntakePivotModes), this)
				.withName("IntakePivot.runEnum");
	}

	public Command runIntakeerPivotCommandMM(IntakePivotModes IntakePivotModes) {
		return new RunCommand(() -> this.runEnumMM(IntakePivotModes), this)
				.withName("IntakePivot.runEnumMM");
	}

	public Command stopIntakeerPivotCommand() {
		return new RunCommand(() -> this.stop(), this)
				.withName("IntakePivot.STOP");
	}

	public Command IntakeerPivotResetPositionCommand() {
		return new RunCommand(() -> this.setZero(), this)
				.withName("IntakePivot.setZero");
	}

	public String getMode() {
		return this.mode.toString();
	}

	public String getManipJointState() {
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
