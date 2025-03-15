package frc.robot.Subsytems.CANdle;

import java.time.LocalTime;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.CANdleConstants;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;

public class TitanCANdle extends SubsystemBase {
    // Code heavily inspired from
    // https://github.com/FRC2539/javabot-2023/blob/main/src/main/java/frc/robot/subsystems/LightsSubsystem.java

    private static CANdle candle = new CANdle(CANdleConstants.id, Constants.canbus);

    
        enum TitanPrideColor {
            CYAN, BLUE, PURPLE
        }
    
        TitanPrideColor currentPRIDE = TitanPrideColor.BLUE;
    
        private boolean hasRunThisSecond = false;
    
        public TitanCANdle() {
            CANdleConfiguration config = new CANdleConfiguration();
            config.statusLedOffWhenActive = true;
            config.disableWhenLOS = false;
            config.stripType = LEDStripType.RGB;
            config.brightnessScalar = 1.0;
            config.vBatOutputMode = VBatOutputMode.Modulated;
            candle.configAllSettings(config, 100);
    
        }
    
        public void setBrightness(double percent) {
            candle.configBrightnessScalar(percent, 100);
        }
    
        public void titanPride() {
            int timeSec = LocalTime.now().getSecond();
    
            if (timeSec % 2 == 0 && !hasRunThisSecond) {
                hasRunThisSecond = true;
                switch (currentPRIDE) {
                    case BLUE:
                        LEDSegment.MainStrip.setFlowAnimation(CANdleConstants.purple, 1);
                    case PURPLE:
                        LEDSegment.MainStrip.setFlowAnimation(CANdleConstants.cyanish, 1);
                    case CYAN:
                        LEDSegment.MainStrip.setFlowAnimation(CANdleConstants.darkBlue, 1);
                }
    
            } else {
                hasRunThisSecond = false;
            }
        }
    
        public Command clearSegmentCommand(LEDSegment segment) {
            return runOnce(() -> {
                segment.clearAnimation();
                segment.disableLEDs();
            });
        }
    
        public Command titanCommand() {
            return new RunCommand(() -> titanPride(), this);
        }
    
        public Command changeAnimationCommand(AnimationTypes type) {
            return new RunCommand(() -> changeAnimation(type), this);
        }
    
        public void changeAnimation(AnimationTypes type) {
            LEDSegment.MainStrip.setStrobeAnimation(type.color, type.speed);
        }
    
        public static enum LEDSegment {
            BatteryIndicator(0, 2, 0), DriverStationIndicator(2, 1, -1), MainStrip(3, 10, 1);
    
            public final int startIndex;
            public final int segmentSize;
            public final int animationSlot;
    
            private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
                this.startIndex = startIndex;
                this.segmentSize = segmentSize;
                this.animationSlot = animationSlot;
            }
    
            public void setColor(Color color) {
                clearAnimation();
                candle.setLEDs(((int) color.red), ((int) color.green), ((int) color.blue), 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(CANdleConstants.black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    ((int) color.red), ((int) color.green), ((int) color.blue), 0, speed, segmentSize,
                    Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(((int) color.red), ((int) color.green), ((int) color.blue), 0, speed,
                            segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    ((int) color.red), ((int) color.green), ((int) color.blue), 0, speed, segmentSize, BounceMode.Front,
                    3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(((int) color.red), ((int) color.green), ((int) color.blue), 0, speed,
                    segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
    }
}