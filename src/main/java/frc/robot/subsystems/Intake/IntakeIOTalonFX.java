package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import Utils.FPGA.currentTime;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
     private final StatusSignal<AngularVelocity> rollerVelocity;
     private final StatusSignal<Angle> pivotAngle;
     private final StatusSignal<Current> rollerCurrent;
     private final StatusSignal<Voltage> rollerVoltage;
     private final StatusSignal<Current> pivotCurrent;

     public DigitalInput zeroingSwitch = new DigitalInput(0);
     public IntakeConfigurator config;
     public TalonFX pivot;
     public TalonFX roller;
     public TalonFX pivotfollower;
     public TalonFX rollerfollower;
     private VelocityVoltage rollerRequest = new VelocityVoltage(0);
     private MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

     // private Follower followPivot = new Follower(Constants.intakePivotID, MotorAlignmentValue.Opposed);
     private Follower followRoller = new Follower(Constants.intakeRollerID, MotorAlignmentValue.Opposed);

     public IntakeIOTalonFX() {
          config = new IntakeConfigurator();
          
          roller = new TalonFX(Constants.intakeRollerID, Constants.krakenBus);
          pivot = new TalonFX(Constants.intakePivotID, Constants.krakenBus);

          rollerfollower = new TalonFX(Constants.intakeRollerFollowerID, Constants.krakenBus);
          pivotfollower = new TalonFX(Constants.intakePivotFollowerID, Constants.krakenBus);
          // pivotfollower = new TalonFX(Constants.intakePivotFollowerID, Constants.krakenBus);
          pivotfollower.setNeutralMode(NeutralModeValue.Coast);//BRAKE NORMALLY, BUT LANE IS GONNA FART

          roller.setNeutralMode(NeutralModeValue.Coast);

          roller.getConfigurator().apply(config.intakeRollerConfig);
          pivot.getConfigurator().apply(config.intakeRightPivotConfig);
          pivot.getConfigurator().apply(Constants.standardIntakeCurrentLimits);
          pivotfollower.getConfigurator().apply(config.intakeLeftPivotConfig);
          pivotfollower.getConfigurator().apply(Constants.standardIntakeCurrentLimits);

          rollerfollower.setControl(followRoller);
          // pivotfollower.setControl(followPivot);

          rollerVelocity = roller.getVelocity();
          pivotAngle = pivot.getPosition();
          rollerCurrent = roller.getStatorCurrent();
          rollerVoltage = roller.getMotorVoltage();
          pivotCurrent = pivot.getStatorCurrent();

          PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,rollerVelocity,pivotAngle));
          PhoenixUtil.tryUntilOk(5, () -> roller.optimizeBusUtilization(0,1.0));
          PhoenixUtil.tryUntilOk(5, () -> pivot.optimizeBusUtilization(0,1.0));
          PhoenixUtil.tryUntilOk(5, () -> pivotfollower.optimizeBusUtilization(0,1.0));

          PhoenixUtil.registerSignals(
               Constants.krakenBus.isNetworkFD(),
               rollerVelocity,
               pivotAngle,
               rollerCurrent,
               rollerVoltage,
               pivotCurrent);
     }

     public void updateInputs(IntakeIOInputs inputs) {
          inputs.intakePosition = getIntakePosition();
          inputs.rollerRPS = getRollerRPS();
          inputs.pivotRequest = pivotRequest.getPositionMeasure().in(Units.Rotations);
          inputs.rollerCurrent = rollerCurrent.getValueAsDouble();
          inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
          inputs.pivotCurrent = pivotCurrent.getValueAsDouble();
          inputs.pivotLimitSwitch = this.isZeroingSwitchPressed();
          inputs.intakePivotStalled = this.intakePivotStalled();
     }

     @Override
     public void setIntakeState(IntakeStates states) {
          // pivotRequest.Position = states.intakePosition;
          pivot.setControl(pivotRequest.withPosition(states.intakePosition).withFeedForward(1));
          pivotfollower.setControl(pivotRequest.withPosition(states.intakePosition).withFeedForward(1));
          roller.setControl(rollerRequest.withVelocity(states.rollerRPS));

     }

     @Override
     public double getRollerRPS() {
          return rollerVelocity.getValueAsDouble();
     }

     @Override
     public double getIntakePosition() {
          return pivotAngle.getValueAsDouble();
     }
     @Override
     public void setRotateVoltage(double voltage) {
          this.pivot.setVoltage(voltage);
     }

     @Override
     public void setRollerSpeed(double rps){
          roller.setControl(rollerRequest.withVelocity(rps));
     }

     public boolean isZeroingSwitchPressed() {
          // return zeroingSwitch.get();
          return false;
     }

     public boolean intakePivotStalled() {
          // return pivot.getFault_StatorCurrLimit().getValue();turretMotor
          return false;
     }

     public boolean zeroIntake() {
        boolean pressed = isZeroingSwitchPressed();
        if (!pressed) {
            setRotateVoltage(0);
            pivot.setPosition(0);
            pivotfollower.setPosition(0);
            Intake.getInstance().alreadyZeroed = true;
        } else {
          setRotateVoltage(-2);
          Intake.getInstance().alreadyZeroed = false;
        }
        return !pressed;
     }

     @Override
     public TalonFX[] getTalonFXPivotMotors(){
          return new TalonFX[] {pivot, pivotfollower};
     }

     @Override
     public TalonFX[] getTalonFXRollerMotors(){
          return new TalonFX[] {roller, rollerfollower};
     }
}
