package frc.robot.subsystems.Lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Vision;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;

public enum LightAnimations {

    DEFAULT(new SolidColor(0,65).withColor(new RGBWColor(255, 200, 0, 0))), // Yellow
    CANTSHOOT(new SolidColor(0,65).withColor(new RGBWColor(250, 30, 0, 0))), // Orange
    SHOOTFERRY(new SolidColor(0,65).withColor(new RGBWColor(0, 255, 100, 0))), // Tealish
    SHOOTHUB(new SolidColor(0,65).withColor(new RGBWColor(0, 255, 0, 0))), // Green
    ZEROED(new SolidColor(0,65).withColor(new RGBWColor(0, 0, 250, 100))),//light blue   /**
    ZEROFIX(new SolidColor(0,65).withColor(new RGBWColor(0, 0, 250, 100))),//light blue   /**
    ZEROFIXSTALL(new SolidColor(0,65).withColor(new RGBWColor(0, 0, 250, 100))),//light blue   /**
    DISABLED(new SolidColor(0,65).withColor(new RGBWColor(250, 0, 250, 0))),//pink   /**
    DISCONNECTEDCAMERA(new SolidColor(0,65).withColor(new RGBWColor(255, 0, 0, 0))),//violently red

    //NOT USING
    INTAKING(new SolidColor(0,65).withColor(new RGBWColor(250, 0, 250, 100))), // Blue
    REVERSEINTAKING(new SolidColor(0,65).withColor(new RGBWColor(250, 0, 250, 100))); // The hottest pink



    /*  The strobe animation
>>>>>>> Stashed changes
   */

    public static StrobeAnimation strobeSlow = new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 0, 0)).withFrameRate(10);
    public static StrobeAnimation strobeFast = new StrobeAnimation(0, 65).withColor(new RGBWColor(255, 0, 0, 0)).withFrameRate(2);
    public static RainbowAnimation rainbowAnim = new RainbowAnimation(0, 65);
       /**
       * The time
       */
        public double time;
        
        SolidColor color;
            
            
                private LightAnimations(SolidColor color) {
                    this.color = color;
        }

    public static void Lights(){
        if(Vision.getInstance().io.CamerasConnected()){
            if(!Robot.zeroingLights) {
                if (!Robot.zeroingFIXLights) {
                    if (SwerveSubsystem.getInstance().isInAllianceZone()) {
                        if (Shooter.getInstance().isAimedAtHub && Shooter.getInstance().isHubActive()) {
                            Lights.getLightInstance().lightsWantedState = SHOOTHUB;
                        } else {
                            Lights.getLightInstance().lightsWantedState = CANTSHOOT;
                        }
                    } else {
                        if (Shooter.getInstance().isAimedAtFerry) {
                            Lights.getLightInstance().lightsWantedState = SHOOTFERRY;
                        } else {
                            Lights.getLightInstance().lightsWantedState = CANTSHOOT;

                        }
                    }
                } else {
                    Lights.getLightInstance().lightsWantedState = ZEROFIX;
                }
                } else {
                    Lights.getLightInstance().lightsWantedState = ZEROED;
            }
        }
        }
    }

