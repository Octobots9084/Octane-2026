package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

//import com.revrobotics.

/**As an interface, none of these variables work. Javadocs only apply to implementations.*/
public interface IntakeIO {

    /**Used to contain the list of measured and logged values. */
    @AutoLog
    public static class IntakeIOInputs {
        public IntakeStates wantedState = IntakeStates.SAFE;
        public IntakeStates currentState = IntakeStates.SAFE;
        public double rollerRPS = 0.0;
        public double intakePosition = 0.0;
        public double rollerTemp = 0.0;
        public double pivotTemp = 0.0;
        public double rollerCurrent = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotRequest = 0.0;
        public boolean pivotLimitSwitch = false;
        public double rollerVoltage;
        public boolean intakePivotStalled;
    }

    /** The mechanism by which values are updated with.*/
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /**Sets the state the motors read to spin at.*/
    public default void setIntakeState(IntakeStates states) {
    }

    /**Sets the roller motor speed in rotations per second. Not of much use outside of IntakeIOTalonFX.*/
    public default void setRollerSpeed(double rps){}

    /**Retrieves the rotations per second of the roller.*/
    public default double getRollerRPS() {
        return 0;
    }

    /**Retrieves the angle the intake is at.*/
    public default double getIntakePosition() {
        return 0;
    }
    
    /**Low-level way of adjusting speed of the motors. Bypasses everything, and not safe. Also run in a loop, so little use as well.*/ 
    public default void setRotateVoltage(double volts){}

    /**Detects the zeroing switch, which may change.*/
    public default boolean isZeroingSwitchPressed() {// GAS_D
        return false;
    }

    public default boolean intakePivotStalled() {// GAS_D
        return false;
    }

    /**@return true if it has finished.*/
    public default boolean zeroIntake(){
        return false;
    }

    public default TalonFX[] getTalonFXPivotMotors(){
        return new TalonFX[] {};
    }

    public default TalonFX[] getTalonFXRollerMotors(){
          return new TalonFX[] {};
    }
}
