package frc.robot.Subsytems.Climber;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Constants.ClimberConstants;
import frc.robot.Util.Constants.ClimberConstants.ClimberPositions;
import frc.robot.Util.Constants.ClimberConstants.ClimberStates;
import frc.robot.Util.Constants.ManipulatorConstants;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class Climber extends REVMechanism  {

  private ClimberConfig config = new ClimberConfig();
	private SparkMax motor;
	public boolean attached;

  @Getter private ClimberPositions mode;
	@Getter @Setter private ClimberStates state;

  private static class ClimberConfig extends Config {
	  public ClimberConfig() {
			super("Climber", ClimberConstants.id);
			configIdleMode(ClimberConstants.idleMode);
			configInverted(ClimberConstants.isInverted);
			configEncoderPosRatio(ClimberConstants.gearRatio);
			configPositionWrapping(false);
			configAbsoluteEncoderInverted(false);
			configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
			configMaxIAccum(ClimberConstants.maxIAccum);
			configPIDGains(ManipulatorConstants.p, ManipulatorConstants.i, ManipulatorConstants.d);
			configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
			configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
		}
	}

  public Climber(SparkMax motor, boolean attached) {
		super(motor, attached);
		this.motor = motor;
		this.attached = attached;
		this.mode = ClimberPositions.STOW;
		this.state = ClimberStates.STOWED;
		config.applySparkConfig(motor);

		Logger.recordOutput("ClimberMode", getMode());
		Logger.recordOutput("Climber/Setpoint", getMode().position.in(Rotation));
		Logger.recordOutput("Climber/State", getState());
		Logger.recordOutput("Climber/Velocity", getMotorVelocity());
		Logger.recordOutput("Climber/Voltage", getMotorVoltage());
		Logger.recordOutput("Climber/Current", getMotorCurrent());
		Logger.recordOutput("Climber/Output", getMotorOutput());
		Logger.recordOutput("Climber/Velocity", getMotorPosition());
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Climber Mode", this.getMode().toString());
		SmartDashboard.putNumber("Climber Setpoint", getMode().position.in(Rotations));
		SmartDashboard.putBoolean("Climber Goal",
				getPositionSetpointGoal(getMode().position, ClimberConstants.error));
		SmartDashboard.putNumber("Climber Output", this.getMotorOutput());
		SmartDashboard.putNumber("Climber Current", this.getMotorCurrent());
		SmartDashboard.putNumber("Climber Voltage", this.getMotorVoltage());
		SmartDashboard.putNumber("Climber Position", this.getMotorPosition());

	}
	
	@Override
	public boolean getPositionSetpointGoal(Angle target, Angle error) {
		if (attached) {
			if (Calc.approxEquals(motor.getEncoder().getPosition(), target.in(Rotations),
					error.in(Rotations))) {
				return true;
			}
		}
		return false;
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public void runEnum(ClimberPositions Climbermode) {
		this.mode = Climbermode;
		setMotorPosition(Climbermode.position);
	}

	// Stole from ManipJoint, seems like it was never used when I searched but whatever.
	protected void runEnumMM(ClimberPositions Climbermode) {
		this.mode = Climbermode;
		setMMPosition(Climbermode.position);
	}

	public Command runClimberCommand(ClimberPositions Climbermode) {
		return new InstantCommand(() -> this.runEnum(Climbermode), this)
				.withName("Climber.runEnum");
	}

	// May comment out these two but who cares ig
	public Command runClimberCommandMM(ClimberPositions Climbermode) {
		return new InstantCommand(() -> this.runEnumMM(Climbermode), this)
				.withName("Climber.runEnumMM");
	}

	// Also was never used but whatever
	public Command killClimberCommand() {
		return new InstantCommand(() -> this.stop(), this)
				.withName("KILL Climber COMMAND");
	}

	// ALSO ALSO Never used?
	public Command ClimberResetPositionCommand() {
		return new InstantCommand(() -> this.setZero(), this)
				.withName("Climber.setZero");
	}

	public double getMotorPosition() {
		if (attached) {
			return motor.getEncoder().getPosition();
		}

		return 0;
	}
  @Override
  protected Config setConfig() {
    if (attached) {
      config.applySparkConfig(motor);
    }
    return this.config;
  }
}
