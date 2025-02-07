package frc.robot.Util;

import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class PresetPosition {

    private CleanPivotModes pivotMode;
    private ManipJointPositions jointMode;
    private ElevatorPositions elevatorMode;

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode, CleanPivotModes pivotMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = pivotMode;
    }

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = CleanPivotModes.STOW;
    }

    public PresetPosition(ElevatorPositions elevatorMode, CleanPivotModes pivotMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = ManipJointPositions.STOW;
        this.pivotMode = pivotMode;
    }

    public ElevatorPositions getElevatorMode() {
        return elevatorMode;
    }

    public void setElevatorMode(ElevatorPositions elevatorMode) {
        this.elevatorMode = elevatorMode;
    }

    public void setPivotMode(CleanPivotModes pivotMode) {
        this.pivotMode = pivotMode;
    }

    public void setJointMode(ManipJointPositions jointMode) {
        this.jointMode = jointMode;
    }

    public CleanPivotModes getPivotMode() {
        return pivotMode;
    }

    public ManipJointPositions getJointMode() {
        return jointMode;
    }

}
