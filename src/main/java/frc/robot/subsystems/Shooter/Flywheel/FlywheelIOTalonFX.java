package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.util.PhoenixUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelVoltage;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Double> flywheelDutyCycle;

    public static TalonFX FlywheelLeftMotor;
    public static TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    public double targetRPS = 0.0;
    private VelocityVoltage FlywheelRightMotorRequest = new VelocityVoltage(0);

    private Follower follow = new Follower(Constants.flyWheelRightID, MotorAlignmentValue.Aligned);

    
    public FlywheelIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        FlywheelLeftMotor = new TalonFX(Constants.flyWheelLeftID, Constants.krakenBus);
        FlywheelRightMotor = new TalonFX(Constants.flyWheelRightID, Constants.krakenBus);

        FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);

        FlywheelLeftMotor.setControl(follow);

        flywheelVelocity = FlywheelRightMotor.getVelocity();
        flywheelCurrent = FlywheelRightMotor.getStatorCurrent();
        flywheelVoltage = FlywheelRightMotor.getMotorVoltage();
        flywheelDutyCycle = FlywheelRightMotor.getDutyCycle();

        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,flywheelVelocity));
        PhoenixUtil.tryUntilOk(5, () -> FlywheelRightMotor.optimizeBusUtilization(0,1.0));

        PhoenixUtil.registerSignals(
            Constants.krakenBus.isNetworkFD(),
            flywheelVelocity,
            flywheelCurrent,
            flywheelVoltage);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.flywheelCurrentState = Flywheel.getInstance().getCurrentState();
        inputs.FlywheelRightRPS = getRightMotorVelocity();
        inputs.flywheelRightVelocity = flywheelVelocity.getValueAsDouble();
        inputs.flywheelRightVoltage = flywheelVoltage.getValueAsDouble();
        inputs.flywheelRightCurrent = flywheelCurrent.getValueAsDouble();
        inputs.flywheelRightDutyCycle = flywheelDutyCycle.getValueAsDouble();
        inputs.flywheelWantedSpeed = targetRPS;
    }

    @Override
    public void setFlywheelVelocity(FlywheelStates state) {
        Logger.recordOutput("flywheelWantedSpeed", state.FlywheelRightRPS);
        targetRPS  = state.FlywheelRightRPS;
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(state.FlywheelRightRPS));
    }
    @Override
    public void setFlywheelVelocity(double rps) {
        Logger.recordOutput("flywheelWantedSpeed", rps);
        targetRPS = rps;
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest.withVelocity(rps));
    }

    public double getRightMotorVelocity() {
        return flywheelVelocity.getValueAsDouble();
    }

    public double getRightMotorStatorCurrent() {
        return flywheelCurrent.getValueAsDouble();
    }

    public double getRightMotorDutyCycle() {
        return flywheelDutyCycle.getValueAsDouble();
    }

    @Override
    public boolean FlywheelInTolerance(double tolerance){
        return MathUtil.isNear(targetRPS, flywheelVelocity.getValueAsDouble(),tolerance)&& targetRPS > 0;
    }

}
