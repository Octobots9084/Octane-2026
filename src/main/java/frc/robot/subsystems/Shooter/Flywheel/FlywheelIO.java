package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Shooter.Feeder.FeederStates;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public FlywheelStates flywheelCurrentState;
    // public double FlywheelLeftRPS = 0.0;
    // public double FlywheelLeftMotorTemp = 0.0;
    public double FlywheelRightRPS = 0.0;
    public double flywheelRightVelocity = 0.0;
    public double flywheelRightCurrent = 0.0;
    public double flywheelRightVoltage = 0.0;
    public double flywheelRightDutyCycle = 0.0;
    // public double FlywheelRightMotorTemp = 0.0;
    // public double FlywheelLeftCurrent = 0.0;
    // public double FlywheelRightCurrent = 0.0;
    public double flywheelWantedSpeed = 0.0;

  }

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

  public default void setFlywheelVelocity(FlywheelStates state) {
  }
  public default void setFlywheelVelocity(double state) {
  }

  public default double getRightMotorVelocity() {
    return 0;
  }

  public default double getRightMotorStatorCurrent() {
    return 0;
  }

  public default double getRightMotorDutyCycle() {
    return 0;
  }

  public default boolean FlywheelInTolerance(double tolerance) {
    return false;
  }
}
