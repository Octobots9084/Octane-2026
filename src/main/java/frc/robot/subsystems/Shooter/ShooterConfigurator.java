package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ShooterConfigurator {
    public TalonFXConfiguration flyWheelRightConfig;
    public TalonFXConfiguration turretConfig;
    public TalonFXConfiguration hoodConfig;
    public TalonFXConfiguration spindexerConfig;
    public TalonFXConfiguration verticalFeederConfig;
    public TalonFXConfiguration gateFeederConfig;
    public TalonFXConfiguration topRollerConfig;

    public ShooterConfigurator() {
        flyWheelRightConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((1)));
        turretConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.turretGearRatio));
        hoodConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.hoodGearRatio));
        spindexerConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.spindexerGearRatio));
        verticalFeederConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.feederGearRatio));
        gateFeederConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.gateGearRatio));
        // fly wheel right config
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimit = 20;
        flyWheelRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimit = 60;
        flyWheelRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        flyWheelRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flyWheelRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // create PID gains
        flyWheelRightConfig.Slot0.kP = 0.3;
        flyWheelRightConfig.Slot0.kI = 0.0;
        flyWheelRightConfig.Slot0.kD = 0.0;
        flyWheelRightConfig.Slot0.kA = 0.0;
        flyWheelRightConfig.Slot0.kV = 0.13;
        flyWheelRightConfig.Slot0.kS = 0.0;
        flyWheelRightConfig.Slot0.kG = 0.0;

        flyWheelRightConfig.MotionMagic.MotionMagicAcceleration = 100;
        flyWheelRightConfig.MotionMagic.MotionMagicJerk = 100;
        flyWheelRightConfig.MotionMagic.MotionMagicCruiseVelocity = 1000;

        
        // turret config
        turretConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = 60;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // create PID gains
        turretConfig.Slot0.kP = 1000; //1125;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kV = 1;
        turretConfig.Slot0.kS = 0; //0.5;
        turretConfig.Slot0.kG = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = 1500;
        turretConfig.MotionMagic.MotionMagicJerk = 0;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 7000;

        // hood config
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 60;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // create PID gains
        hoodConfig.Slot0.kP = 800;//100
        hoodConfig.Slot0.kI = 0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kA = 0.0;
        hoodConfig.Slot0.kV =
         0.0;
        hoodConfig.Slot0.kS = 0.0;
        hoodConfig.Slot0.kG = 0.0;

        hoodConfig.MotionMagic.MotionMagicAcceleration = 8;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 8;

        // spindexer config(four lane highway)
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 60;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // create PID gains
        spindexerConfig.Slot0.kP = 2;
        spindexerConfig.Slot0.kI = 0.0;
        spindexerConfig.Slot0.kD = 0.0;
        spindexerConfig.Slot0.kA = 0.0;
        spindexerConfig.Slot0.kV = 0.5;
        spindexerConfig.Slot0.kS = 0.0;
        spindexerConfig.Slot0.kG = 0.0;

        
        // spindexerConfig.MotionMagic.MotionMagicAcceleration = 10;
        // spindexerConfig.MotionMagic.MotionMagicJerk = 100;
        // spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 2.75;
        

        // vertical Feeder config(rural road)
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimit = 20;
        verticalFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimit = 80;
        verticalFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        verticalFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        verticalFeederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // create PID gains
        verticalFeederConfig.Slot0.kP = 3;
        verticalFeederConfig.Slot0.kI = 0.0;
        verticalFeederConfig.Slot0.kD = 0.0;
        verticalFeederConfig.Slot0.kA = 0.0;
        verticalFeederConfig.Slot0.kV = 0.82;
        verticalFeederConfig.Slot0.kS = 0.0;
        verticalFeederConfig.Slot0.kG = 0.0;

        verticalFeederConfig.MotionMagic.MotionMagicAcceleration = 1;
        verticalFeederConfig.MotionMagic.MotionMagicJerk = 0;
        verticalFeederConfig.MotionMagic.MotionMagicCruiseVelocity = 0.04;

        // gate Feeder config(rural road)
        gateFeederConfig.CurrentLimits.SupplyCurrentLimit = 20;
        gateFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        gateFeederConfig.CurrentLimits.StatorCurrentLimit = 80;
        gateFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        gateFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        gateFeederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // create PID gains
        gateFeederConfig.Slot0.kP = 1;
        gateFeederConfig.Slot0.kI = 0.0;
        gateFeederConfig.Slot0.kD = 0.0;
        gateFeederConfig.Slot0.kA = 0.0;
        gateFeederConfig.Slot0.kV = 0.5;
        gateFeederConfig.Slot0.kS = 0.0;
        gateFeederConfig.Slot0.kG = 0.0;

        gateFeederConfig.MotionMagic.MotionMagicAcceleration = 1;
        gateFeederConfig.MotionMagic.MotionMagicJerk = 0;
        gateFeederConfig.MotionMagic.MotionMagicCruiseVelocity = 0.04;
    }
}
