package frc.robot.Subsytems.Cleaner;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.CleanPivotConstants;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

@Deprecated
public class CleanPivot extends REVMechanism {

	public SparkMax motor;
	public SparkClosedLoopController controller;
	public AbsoluteEncoder absoluteEncoder;
	public RelativeEncoder relativeEncoder;
	public double massKg;
	public boolean isShooter;
	public boolean attached;

	@Getter public CleanPivotModes mode;
	@Getter @Setter public CleanPivotStates state;
	public static class PivotConfig extends Config {

		public PivotConfig() {
			super("CleanPivot", CleanPivotConstants.id);
			configSoftLimit(CleanPivotConstants.softLimitEnabled, CleanPivotConstants.softLimitForwardMax,
					CleanPivotConstants.softLimitReverseMax);
			configInverted(CleanPivotConstants.isInverted);
			configFeedbackSensorSource(CleanPivotConstants.feedbackSensor, CleanPivotConstants.zeroOffset);
			configPIDGains(CleanPivotConstants.p, CleanPivotConstants.i, CleanPivotConstants.d);
		}
	}

	private PivotConfig config = new PivotConfig();

	public CleanPivot(SparkMax motor, boolean attached) {
		super(motor, attached);
		this.motor = motor;
		this.attached = attached;
		this.absoluteEncoder = motor.getAbsoluteEncoder();
		this.relativeEncoder = motor.getEncoder();
		this.mode = CleanPivotModes.STOW;
		this.state = CleanPivotStates.STOW;
		config.applySparkConfig(motor);

		Logger.recordOutput("Cleaner/Pivot/Mode", getMode());
		Logger.recordOutput("Cleaner/Pivot/State", getState());
		Logger.recordOutput("Cleaner/Pivot/Setpoint", getMode().angle.in(Rotation));
		Logger.recordOutput("Cleaner/Pivot/Output", getMotorOutput());
		Logger.recordOutput("Cleaner/Pivot/Position", absoluteEncoder.getPosition());
		Logger.recordOutput("Cleaner/Pivot/Current", getMotorCurrent());
		Logger.recordOutput("Cleaner/Pivot/Voltage", getMotorVoltage());
		Logger.recordOutput("Cleaner/Pivot/Velocity", getMotorVelocity());
	}

	public void periodic() {
	// 	SmartDashboard.putString("Cleaner Pivot Mode", getMode().toString());
	// 	SmartDashboard.putString("Cleaner Pivot Mode", getState().toString());
	// 	SmartDashboard.putNumber("Cleaner Pivot Setpoint", getMode().angle.in(Rotation));
	// 	SmartDashboard.putNumber("Cleaner Pivot Output", getMotorOutput());
	// 	SmartDashboard.putNumber("Cleaner Pivot Position", absoluteEncoder.getPosition());
	// 	SmartDashboard.putNumber("Cleaner Pivot Current", getMotorCurrent());
	// 	SmartDashboard.putNumber("Cleaner Pivot Voltage", getMotorVoltage());
	// 	SmartDashboard.putNumber("Cleaner Pivot Velocity", getMotorVelocity());
	 }

	public void runEnum(CleanPivotModes cleanPivotModes) {
		this.mode = cleanPivotModes;
		setMotorPosition(cleanPivotModes.angle);
	}

	protected void runEnumMM(CleanPivotModes cleanPivotModes) {
		this.mode = cleanPivotModes;
		setMMPosition(cleanPivotModes.angle);
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public Command runCleanerPivotCommand(CleanPivotModes cleanPivotModes) {
		return new RunCommand(() -> this.runEnum(cleanPivotModes), this)
				.withName("CleanPivot.runEnum");
	}

	public Command runCleanerPivotCommandMM(CleanPivotModes cleanPivotModes) {
		return new RunCommand(() -> this.runEnumMM(cleanPivotModes), this)
				.withName("CleanPivot.runEnumMM");
	}

	public Command stopCleanerPivotCommand() {
		return new RunCommand(() -> this.stop(), this)
				.withName("CleanPivot.STOP");
	}

	public Command cleanerPivotResetPositionCommand() {
		return new RunCommand(() -> this.setZero(), this)
				.withName("CleanPivot.setZero");
	}

	@Override
	protected Config setConfig() {
		if (attached) {
			setConfig(config);
		}
		return this.config;
	}

}
