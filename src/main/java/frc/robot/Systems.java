package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.*;

public class Systems {

    private Intake intake;
    private Elevator elevator;
    private Manipulator manipulator;
    private Climber climber;


    /* Kraken X60s */
    private TalonFX elevatorLeft;
    private TalonFX elevatorRight;

    /* Neo 1.1s */
    private SparkMax intakeMotor;
    private SparkMax climberMotor;

    /* Neo 550s */
    private SparkMax manipulatorMotor;
    

    public Systems() {

        /* Kraken X60s */
        elevatorLeft = new TalonFX(ElevatorConstants.leftId, Constants.canbus);
        elevatorRight = new TalonFX(ElevatorConstants.rightId, Constants.canbus);

        /* Neo 1.1s */
        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);
        climberMotor = new SparkMax(ClimberConstants.id, MotorType.kBrushless);

        /* Neo 550s */
        manipulatorMotor = new SparkMax(ManipulatorConstants.id, MotorType.kBrushless);
        
        /*----------*/
        intake = new Intake(intakeMotor, IntakeConstants.attached);
        climber = new Climber(climberMotor, ClimberConstants.attached);
        elevator = new Elevator(elevatorLeft, elevatorRight, ElevatorConstants.attached);
        manipulator = new Manipulator(manipulatorMotor, ManipulatorConstants.attached);   



    }

    public Intake getIntake() {
        return intake;
    }

    public Climber getClimber() {
        return climber;
    }
    
    public Elevator getElevator() {
        return elevator;
    }
    
    public Manipulator getManipulator() {
        return manipulator;
    }

}
  