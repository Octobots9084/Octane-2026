package frc.robot.subsystems.Lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Vision;

public class Lights extends SubsystemBase{
/**
   * The current state of the lights, which determines the color of the lights
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Lights.LightAnimations#DEFAULT DEFAULT}
   * @param LightAnimations The light states contain color and strobe settings - {@link frc.robot.subsystems.Lights.LightAnimations LightAnimations}
   */
    public LightAnimations lightsCurrentState = LightAnimations.DEFAULT;
  /**
   * The wanted state of the lights, which the subsystem attempts to set the {@link #lightsCurrentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Lights.LightAnimations#DEFAULT DEFAULT}
   * @param LightAnimations The light states contain color and strobe settings - {@link frc.robot.subsystems.Lights.LightAnimations LightAnimations}
   *
   */
    public LightAnimations lightsWantedState = LightAnimations.DEFAULT;
    public LightAnimations lastState = lightsCurrentState;
    public static Lights currentLightInstance;
    public Shooter shooter = Shooter.getInstance();
    public static LightsIOSystem device;

    public void periodic() {
        Logger.recordOutput("lightCurrentState", this.lightsCurrentState);
        lightStateTransitions();
        if(shooter.cantShoot()){
            lightsWantedState = LightAnimations.CANTSHOOT;
        }
        applyStates();
    }

    public Lights() {
        currentLightInstance = this;
        device = new LightsIOSystem();
    }

    public static Lights getLightInstance() {
        if (currentLightInstance == null) {
            setLightInstance(new Lights());
        }
        return currentLightInstance;
    }

    public static void setLightInstance(Lights instance) {
        currentLightInstance = instance;
    }

    public void lightStateTransitions() {
        switch (lightsWantedState) {
             case ZEROED:
                lightsCurrentState = LightAnimations.ZEROED;//implemented
                break;
             case ZEROFIX:
                lightsCurrentState = LightAnimations.ZEROFIX;//implemented
                break;
             case ZEROFIXSTALL:
                lightsCurrentState = LightAnimations.ZEROFIXSTALL;//implemented
                break;
            case REVERSEINTAKING:
                lightsCurrentState = LightAnimations.REVERSEINTAKING;//implemented
                break;
            case CANTSHOOT:
                if(lightsWantedState != LightAnimations.REVERSEINTAKING)
                lightsCurrentState = LightAnimations.CANTSHOOT;
                break;
            case SHOOTHUB:
                if(lightsWantedState != LightAnimations.REVERSEINTAKING)
                lightsCurrentState = LightAnimations.SHOOTHUB;//implemented
                break;
            case SHOOTFERRY:
                if(lightsWantedState != LightAnimations.REVERSEINTAKING)
                lightsCurrentState = LightAnimations.SHOOTFERRY;//implemented
                break;
             case INTAKING:
                if(lightsWantedState != LightAnimations.REVERSEINTAKING || lightsWantedState != LightAnimations.CANTSHOOT || lightsWantedState != LightAnimations.SHOOTFERRY || lightsWantedState != LightAnimations.SHOOTHUB)
                lightsCurrentState = LightAnimations.INTAKING;//implemented
                break;
            case DISABLED:
                lightsCurrentState = LightAnimations.DISABLED;
                break;
            case DISCONNECTEDCAMERA:
                lightsCurrentState = LightAnimations.DISCONNECTEDCAMERA;
            break;
             default:
                lightsCurrentState = LightAnimations.DEFAULT;//implemented
                break;
        }
    }

    public LightAnimations getWantedLightState() {
        return this.lightsWantedState;
    }

    public void applyStates() {
        if (lightsCurrentState == LightAnimations.ZEROFIX) {
            device.candle.setControl(LightAnimations.strobeSlow);
        } else if (lightsCurrentState == LightAnimations.ZEROFIXSTALL) {
            device.candle.setControl(LightAnimations.strobeFast);
        } else {
            device.candle.setControl(lightsCurrentState.color);
        }
    }
}
       
