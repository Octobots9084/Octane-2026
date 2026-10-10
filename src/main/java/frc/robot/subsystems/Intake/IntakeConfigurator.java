package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IntakeConfigurator {
    public TalonFXConfiguration intakeRollerConfig;
    public TalonFXConfiguration intakeRightPivotConfig;
    public TalonFXConfiguration intakeLeftPivotConfig;

    public IntakeConfigurator() {
        intakeRollerConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.intakeRollerGearRatio));
        intakeRightPivotConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.intakePivotGearRatio));
        intakeLeftPivotConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.intakePivotGearRatio));

        // intake roller config
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRollerConfig.CurrentLimits.StatorCurrentLimit = 59.99;
        intakeRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // create PID gains
        intakeRollerConfig.Slot0.kP = 0.25;
        intakeRollerConfig.Slot0.kI = 0.0;
        intakeRollerConfig.Slot0.kD = 0.0;
        intakeRollerConfig.Slot0.kA = 0.0;
        intakeRollerConfig.Slot0.kV = 0.25;
        intakeRollerConfig.Slot0.kS = 0.0;
        intakeRollerConfig.Slot0.kG = 0.0;


        // intake pivot right config
        intakeRightPivotConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakeRightPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRightPivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakeRightPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        intakeRightPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//BRAKE NORMALLY, BUT LANE IS GONNA FART
        intakeRightPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // create PID gains
        intakeRightPivotConfig.Slot0.kP = 8.0;
        intakeRightPivotConfig.Slot0.kI = 0.1;
        intakeRightPivotConfig.Slot0.kD = 0.0;
        intakeRightPivotConfig.Slot0.kA = 0.0;
        intakeRightPivotConfig.Slot0.kV = 0.0;
        intakeRightPivotConfig.Slot0.kS = 0.0;
        intakeRightPivotConfig.Slot0.kG = 0.0;

        intakeRightPivotConfig.MotionMagic.MotionMagicAcceleration = 64*2;
        intakeRightPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 128*2;

        

        // intake pivot right config
        intakeLeftPivotConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakeLeftPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeLeftPivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakeLeftPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // set break mode and inversion
        intakeLeftPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//BRAKE NORMALLY, BUT LANE IS GONNA FART
        intakeLeftPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // create PID gains
        intakeLeftPivotConfig.Slot0.kP = 8.0;
        intakeLeftPivotConfig.Slot0.kI = 0.1;
        intakeLeftPivotConfig.Slot0.kD = 0.0;
        intakeLeftPivotConfig.Slot0.kA = 0.0;
        intakeLeftPivotConfig.Slot0.kV = 0.0;
        intakeLeftPivotConfig.Slot0.kS = 0.0;
        intakeLeftPivotConfig.Slot0.kG = 0.0;

        intakeLeftPivotConfig.MotionMagic.MotionMagicAcceleration = 64*2;
        intakeLeftPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 128*2;
    }
}