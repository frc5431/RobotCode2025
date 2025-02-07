package frc.robot.Subsytems.Cleaner;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.CleanPivotConstants;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class CleanPivot extends REVMechanism {

	public SparkMax motor;
	public SparkClosedLoopController controller;
	public AbsoluteEncoder absoluteEncoder;
	public CleanPivotModes mode;
	public CleanPivotStates state;
	public double massKg;
	public boolean isShooter;

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

	public CleanPivot(PivotConfig config, SparkMax motor, boolean attached) {
		super(motor, attached);

		this.config = config;
		this.motor = motor;
		this.mode = CleanPivotModes.STOW;
		this.state = CleanPivotStates.STOW;

		config.applySparkConfig(motor);

		Logger.recordOutput("Cleaner/Pivot/Mode", mode.toString());
		Logger.recordOutput("Cleaner/Pivot/Mode", mode.angle.in(Rotation));
		Logger.recordOutput("Cleaner/Pivot/Output", getMotorOutput());
		Logger.recordOutput("Cleaner/Pivot/Position", absoluteEncoder.getPosition());
		Logger.recordOutput("Cleaner/Pivot/Current", getMotorCurrent());
		Logger.recordOutput("Cleaner/Pivot/Voltage", getMotorVoltage());
		Logger.recordOutput("Cleaner/Pivot/Velocity", getMotorVelocity());
	}

	public void periodic() {
		SmartDashboard.putString("Cleaner Pivot Mode", mode.toString());
		SmartDashboard.putNumber("Cleaner Pivot Mode", mode.angle.in(Rotation));
		SmartDashboard.putNumber("Cleaner Pivot Output", getMotorOutput());
		SmartDashboard.putNumber("Cleaner Pivot Position", absoluteEncoder.getPosition());
		SmartDashboard.putNumber("Cleaner Pivot Current", getMotorCurrent());
		SmartDashboard.putNumber("Cleaner Pivot Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Cleaner Pivot Velocity", getMotorVelocity());
	}

	public void setManipJointState(CleanPivotStates cleanerPivotStates) {
		this.state = cleanerPivotStates;
	}

	protected void runEnum(CleanPivotModes cleanPivotModes) {
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
