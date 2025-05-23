// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.CANdleConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANdleConstants.id, "rio");
    private final int LedCount = 113;

    enum TitanPrideColor {
        CYAN, BLUE, PURPLE
    }

    TitanPrideColor currentPRIDE = TitanPrideColor.BLUE;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow, SCORE, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, ELEVATOR_GOAL, INTAKE, SPIRIT, STRESS_TIME, BLINK_BLUE, BLINK_RED, SetAll
    }

    @SuppressWarnings("unused")
    private AnimationTypes m_currentAnimation;

    public CANdleSystem() {
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }
    
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch (toChange) {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case SCORE:
            m_toAnimate = new RainbowAnimation(1, 1, LedCount);
            break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                m_toAnimate.setLedOffset(8);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                m_toAnimate.setLedOffset(8);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(1, 0.8, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                m_toAnimate.setLedOffset(8);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                m_toAnimate.setLedOffset(8);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                m_toAnimate.setLedOffset(8);
                break;
            case SPIRIT:
                m_toAnimate = new TwinkleAnimation(100, 10, 200, 0, 0.8, LedCount, TwinklePercent.Percent64);
                m_toAnimate.setLedOffset(8);
                break;
            case BLINK_BLUE:
                m_toAnimate = new StrobeAnimation(20, 10, 180, 0, 98.0 / 256.0, LedCount);
                m_toAnimate.setLedOffset(8);
                break;
            case ELEVATOR_GOAL:
                m_toAnimate = new StrobeAnimation(50, 50, 250, 0, 1, LedCount);
                break;
            case INTAKE:
                m_toAnimate = new LarsonAnimation(20, 170, 250, 0, 1, LedCount, BounceMode.Front, 20);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
            case BLINK_RED:
            m_toAnimate = new StrobeAnimation(20, 180, 10, 0, 98.0 / 256.0, LedCount);
            m_toAnimate.setLedOffset(8);
                break;
            case STRESS_TIME:
            m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
            m_toAnimate.setLedOffset(8);
                break;
            default:
                break;
        }
        // System.out.println("Changed to " + m_currentAnimation.toString());
    }

    public Command changeCANdle(AnimationTypes animationTypes) {
        return new RunCommand(() -> changeAnimation(animationTypes), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_toAnimate == null) {
            m_candle.setLEDs((int) (0),
                    (int) (0),
                    (int) (255));
        } else {
            m_candle.animate(m_toAnimate);
        }

    }

}