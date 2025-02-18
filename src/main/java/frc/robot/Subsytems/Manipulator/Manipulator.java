package frc.robot.Subsytems.Manipulator;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipulatorConstants;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class Manipulator extends REVMechanism {

	private ManipulatorConfig config = new ManipulatorConfig();
	private SparkMax motor;
	public boolean attached;

	@Getter private ManipulatorModes mode;
	@Getter @Setter private ManipulatorStates state;

	public static class ManipulatorConfig extends Config {

		public ManipulatorConfig() {
			super("Manipulator", ManipulatorConstants.id);
			configIdleMode(ManipulatorConstants.idleMode);
			configInverted(ManipulatorConstants.isInverted);
			configEncoderPosRatio(ManipulatorConstants.gearRatio);
			configMaxIAccum(ManipulatorConstants.maxIAccum);
			configMaxMotionPositionMode(ManipulatorConstants.mm_positionMode);
			configPIDGains(ManipulatorConstants.p, ManipulatorConstants.i, ManipulatorConstants.d);
			configSmartCurrentLimit(ManipulatorConstants.stallLimit, ManipulatorConstants.supplyLimit);
			configPeakOutput(ManipulatorConstants.maxForwardOutput, ManipulatorConstants.maxReverseOutput);
			configMaxMotion(ManipulatorConstants.mm_velocity, ManipulatorConstants.mm_maxAccel,
					ManipulatorConstants.mm_error);
		}
	}

	public Manipulator(SparkMax motor, boolean attached) {
		super(motor, attached);
		motor.getForwardLimitSwitch().isPressed();
		this.motor = motor;
		attached = ManipJointConstants.attached;
		this.mode = ManipulatorModes.IDLE;
		this.state = ManipulatorStates.EMPTY;
		config.applySparkConfig(motor);

		Logger.recordOutput("Manipulator/Rollers/Mode", getMode());
		Logger.recordOutput("Manipulator/Rollers/State", getState());
		Logger.recordOutput("Mainpulator/Rollers/Setpoint", getMode().speed.in(RPM));
		Logger.recordOutput("Manipulator/Rollers/Velocity", getMotorVelocity());
		Logger.recordOutput("Manipulator/Rollers/Voltage", getMotorVoltage());
		Logger.recordOutput("Manipulator/Rollers/Current", getMotorCurrent());
		Logger.recordOutput("Manipulator/Rollers/Output", getMotorOutput());

	}

	@Override
	protected Config setConfig() {
		if (attached) {
			config.applySparkConfig(motor);
		}
		return this.config;
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Mainpulator Mode", getMode().toString());
		SmartDashboard.putNumber("Mainpulator Setpoint", getMode().speed.in(RPM));
		SmartDashboard.putString("Manipulator State", getState().toString());
		SmartDashboard.putNumber("Mainpulator Output", getMotorOutput());
		SmartDashboard.putNumber("Mainpulator Current", getMotorCurrent());
		SmartDashboard.putNumber("Mainpulator Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Mainpulator Velocity", getMotorVelocity());
		SmartDashboard.putBoolean("ManipJoint Beambreak Status", getBeambreakStatus());

		switch (this.mode) {
			case IDLE:
				setState(ManipulatorStates.EMPTY);
				break;
			case SCORE:
				setState(ManipulatorStates.INTAKING);
				break;
			case REVERSE:
				setState(ManipulatorStates.OUTTAKING);
				break;
			case FEED:
				setState(ManipulatorStates.INTAKING);
				break;
		}
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public void runEnum(ManipulatorModes manipulatorMode) {
		this.mode = manipulatorMode;
		setVelocity(manipulatorMode.speed);
	}

	   public Command runManipulatorCommand(ManipulatorModes modes) {
        return new StartEndCommand(() -> this.runEnum(modes), () -> this.runEnum(ManipulatorModes.IDLE), this)
                .withName("Cleaner.runEnum");
    }

	public boolean getBeambreakStatus() {
		return motor.getForwardLimitSwitch().isPressed();
	}

}
