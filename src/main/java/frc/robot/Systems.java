package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.CANdle.CANdleSystem;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.CANdle.CANdleSystem.AnimationTypes;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Util.TitanBitDoController;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Limelight.Vision;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants;
import frc.team5431.titan.core.joysticks.TitanController;
import frc.robot.Util.SwerveConstants;
import frc.robot.Util.Constants.*;
import lombok.Getter;

public class Systems {

    private static @Getter TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
    private static @Getter TitanController operator = new TitanController(ControllerConstants.operatorPort,
            ControllerConstants.deadzone);
    private static @Getter TitanBitDoController operator8BitDo = new TitanBitDoController(ControllerConstants.operatorPort);
	private static @Getter AprilTagFieldLayout layout;


    private MotorType brushless = MotorType.kBrushless;

    private @Getter Intake intake;
    private @Getter IntakePivot intakePivot;
    private @Getter Feeder feeder;
    private @Getter ManipJoint manipJoint;
    private @Getter Manipulator manipulator;
    private @Getter Elevator elevator;
    private static @Getter CANdleSystem titanCANdle;
    private static @Getter Vision vision = new Vision();
    private @Getter Climber climber;
    private static @Getter Drivebase drivebase = new Drivebase(
        SwerveConstants.DrivetrainConstants,
        SwerveConstants.FrontLeft, SwerveConstants.FrontRight,
        SwerveConstants.BackLeft, SwerveConstants.BackRight);

    /* Kraken X60s */
    private TalonFX elevatorLeft;
    private TalonFX elevatorRight;

    /* Neo 1.1s */
    private SparkMax intakeMotor;
    private SparkMax manipJointMotor;
    private SparkMax feederMotor;
    private SparkMax intakePivotMotor;
    private SparkMax climberMotor;

    /* Neo 550s */
    private SparkMax manipulatorMotor;

    public Systems() {

        try {
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape-welded.json"));
        } catch (IOException ioE) {
            System.out.println("Apriltag Field Layout Path Bad");
        }

        if (IntakeConstants.attached) {
            intakeMotor = new SparkMax(IntakeConstants.id, brushless);
            intake = new Intake(intakeMotor, IntakeConstants.attached);
        }

        if (IntakePivotConstants.attached) {
            intakePivotMotor = new SparkMax(IntakePivotConstants.id, brushless);
            intakePivot = new IntakePivot(intakePivotMotor, IntakePivotConstants.attached);
        }

        if (FeederConstants.attached) {
            feederMotor = new SparkMax(FeederConstants.id, brushless);
            feeder = new Feeder(feederMotor, FeederConstants.attached);
        }

        if (ManipulatorConstants.attached) {
            manipulatorMotor = new SparkMax(ManipulatorConstants.id, brushless);
            manipulator = new Manipulator(manipulatorMotor, ManipulatorConstants.attached);
        }

        if (ManipJointConstants.attached) {
            manipJointMotor = new SparkMax(ManipJointConstants.id, brushless);
            manipJoint = new ManipJoint(manipJointMotor, ManipJointConstants.attached);
        }

        if (ElevatorConstants.attached) {
            elevatorLeft = new TalonFX(ElevatorConstants.leftId, Constants.canbus);
            elevatorRight = new TalonFX(ElevatorConstants.rightId, Constants.canbus);
            elevator = new Elevator(elevatorLeft, elevatorRight, ElevatorConstants.attached);
        }

        if (ClimberConstants.attached) {
            climberMotor = new SparkMax(ClimberConstants.id, brushless);
            climber = new Climber(climberMotor, ClimberConstants.attached);
        }

        if (CANdleConstants.attached) {
            titanCANdle = new CANdleSystem();
            titanCANdle.changeAnimation(AnimationTypes.SPIRIT);
        }

        
    }
}
