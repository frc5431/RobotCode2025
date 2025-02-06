package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.*;

public class Systems {

    private Intake intake;
    private Elevator elevator;

    /* Kraken X60s */
    private TalonFX elevatorLeft;
    private TalonFX elevatorRight;

    /* Neo 1.1s */
    private SparkMax intakeMotor;

    /* Neo 550s */
    

    public Systems() {

        /* Kraken X60s */
        elevatorLeft = new TalonFX(ElevatorConstants.leftId, Constants.canbus);
        elevatorRight = new TalonFX(ElevatorConstants.rightId, Constants.canbus);

        /* Neo 1.1s */
        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);
        
        /*----------*/
        intake = new Intake(intakeMotor, IntakeConstants.attached);
        elevator = new Elevator(elevatorLeft, elevatorRight, ElevatorConstants.attached);



    }

    public Intake getIntake() {
        return intake;
    }

    public Elevator getElevator() {
        return elevator;
    }

}
  