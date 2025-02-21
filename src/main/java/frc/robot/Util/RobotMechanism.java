package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class RobotMechanism extends SubsystemBase {

    @Getter
    @Setter
    private Double angle = 0.0;
    @Getter
    @Setter
    private Double length = 0.0;

    @Getter
    public Mechanism2d elevator = new Mechanism2d(2, 20, new Color8Bit(Color.kYellow));
    @Getter
    public MechanismRoot2d elevatorBase = elevator.getRoot("Elevator", 1, 9);
    @Getter
    public MechanismLigament2d elevatorExtention = new MechanismLigament2d("ElevatorExtention", 0, 90, 76,
            new Color8Bit(Color.kPurple));
    @Getter
    public MechanismRoot2d broPlsBase = elevator.getRoot("WristBase", 1, 9);
    @Getter
    public MechanismLigament2d wrist = new MechanismLigament2d("Wrist", 6, angle, 76, new Color8Bit(Color.kWheat));
    @Getter
    public MechanismRoot2d manipulatorBase = elevator.getRoot("ManipultorBase", 1, 9);
    @Getter
    public MechanismLigament2d manipulator = new MechanismLigament2d("Wrist", 3, wrist.getAngle(), 76,
            new Color8Bit(Color.kAzure));

    public RobotMechanism() {
        elevatorBase.append(elevatorExtention);
        broPlsBase.append(wrist);
        manipulatorBase.append(manipulator);
        wrist.append(manipulator);
        double elevatorEndX = 1;
        double elevatorEndY = 9 + elevatorExtention.getLength();

        // Calculate the end position of the wrist relative to the elevator extension
        double wristEndX = elevatorEndX + wrist.getLength() * Math.cos(Math.toRadians(wrist.getAngle()));
        double wristEndY = elevatorEndY + wrist.getLength() * Math.sin(Math.toRadians(wrist.getAngle()));

        // Set the position of broPlsBase to the end position of the wrist
        manipulatorBase.setPosition(wristEndX, wristEndY);

    }

    public Object Test() {
        broPlsBase.setPosition(1, elevatorExtention.getLength() + 9);
        elevatorExtention.setLength(elevatorExtention.getLength() + .01);
        wrist.setAngle(wrist.getAngle() + 1);
        manipulator.setAngle(wrist.getAngle());

        return elevator;
    }
    public Command ChangeWristAngleCommand(Double getAngle){
        return this.runOnce(() -> this.setAngle(getAngle));
    }
    public Command ChangeElevatorHeightCommand(Double getLength){
            return new StartEndCommand(() -> this.setLength(getLength),() -> setLength(getLength), this);
    }
}