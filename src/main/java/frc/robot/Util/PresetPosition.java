package frc.robot.Util;

import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import lombok.*;
public class PresetPosition {

    private @Getter @Setter ManipJointPositions jointMode;
    private @Getter @Setter ElevatorPositions elevatorMode;

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
    }

}
