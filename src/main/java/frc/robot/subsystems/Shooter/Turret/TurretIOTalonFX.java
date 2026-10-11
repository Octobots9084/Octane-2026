package frc.robot.subsystems.Shooter.Turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal; 
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConfigurator;
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<Angle> hoodPosition;

    public TalonFX hoodMotor;
    public TalonFX turretMotor;
    public double zeroTurret;
    private MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
    public DigitalInput turretMagnetBreak = new DigitalInput(1);
    public double deadZoneTolerance = 0.1;
    public double wrapPoint = 0;
    public boolean aimed;
    public boolean aimedToShoot;

    public ShooterConfigurator shooterConfigs = new ShooterConfigurator();

    public TurretIOTalonFX() {
        shooterConfigs = new ShooterConfigurator();
        hoodMotor = new TalonFX(Constants.hoodID, Constants.krakenBus);
        turretMotor = new TalonFX(Constants.turretID, Constants.krakenBus);
        hoodMotor.getConfigurator().apply(shooterConfigs.hoodConfig);
        turretMotor.getConfigurator().apply(shooterConfigs.turretConfig);
        
        hoodPosition = hoodMotor.getPosition();
        turretPosition = turretMotor.getPosition();

        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,turretPosition,hoodPosition));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.optimizeBusUtilization(0,1.0));
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.optimizeBusUtilization(0,1.0));

        PhoenixUtil.registerSignals(
            Constants.krakenBus.isNetworkFD(),
            hoodPosition,
            turretPosition);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretLimitSwitch = turretMagnetBreak.get();
        inputs.hoodPosition = this.getHoodPosition()*360;
        inputs.turretPosition = this.getTurretPosition()*360;
        inputs.hoodRequest = hoodRequest.Position*360;
        inputs.turretRequest = turretRequest.Position*360;
        // inputs.turretVoltage = this.turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.turretPositionErr = Math.abs(inputs.turretPosition-inputs.turretRequest);
        // inputs.turretCurrent = this.turretMotor.getStatorCurrent().getValueAsDouble();
        // inputs.hoodCurrent = this.hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setTurretPosition(double turretAngle) {
        // aimedToShoot = false;
        // if (turretAngle < Constants.minTurretAngle/(Math.PI*2)) {
        //     turretMotor.setControl(turretRequest.withPosition(Constants.minTurretAngle/(Math.PI*2)));
        // }else if(turretAngle > Constants.maxTurretAngle/(Math.PI*2)){
        //     turretMotor.setControl(turretRequest.withPosition(Constants.maxTurretAngle/(Math.PI*2)));
        // }else{   
        //     aimedToShoot = true;
            turretMotor.setControl(turretRequest.withPosition(turretAngle));
        // }



    }

    @Override
    public boolean getAimedToShoot(){
        return aimedToShoot;
    }

    @Override
    public void setHoodPosition(double hoodAngle) {
        hoodAngle = Math.max(hoodAngle, Constants.minimumHoodPosition);
        hoodAngle = Math.min(hoodAngle, Constants.maximumHoodPosition);
        hoodMotor.setControl(hoodRequest.withPosition(hoodAngle));
    }

    @Override
    public double getHoodPosition() {
        return hoodPosition.getValueAsDouble();
        
    }
    public boolean getMagnetBreakValue(){
            return turretMagnetBreak.get();
    }

    @Override
    public double getTurretPosition() {
        return turretPosition.getValueAsDouble();
    }

    @Override
    public void moveTurretAndHoodToZero(){
        this.setHoodPosition(74/360.0);
        this.setTurretPosition(0);
    }

    @Override
    public boolean hoodInTolerance(double tolerance) {
        return MathUtil.isNear(hoodRequest.getPositionMeasure().in(Units.Revolution), this.getHoodPosition(),
                tolerance);
    }

    @Override
    public boolean turretInTolerance(double tolerance) {
        boolean turretInTolerance = MathUtil.isNear(turretRequest.getPositionMeasure().in(Units.Revolution), this.getTurretPosition(),
                tolerance);
        Logger.recordOutput("turretRequest for tolerance",turretRequest.getPositionMeasure().in(Units.Revolution));
        Logger.recordOutput("getTurretPosition for tolerance",this.getTurretPosition());
        Logger.recordOutput("turretInTolerance", turretInTolerance);
        return turretInTolerance;
    }

    @Override
    public void zeroHoodMotor(){
        hoodMotor.setVoltage(0.5);
        hoodMotor.setPosition(Constants.maximumHoodPosition);
    }
    @Override
    public void zeroTurretPosition() {
        turretMotor.setPosition(Constants.turretZeroPosition);
    }

    /** loop this if it is being used */

    double startTime = -1000;
    double duration = 1000;

    public void startFiring() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean zeroTurret() {
        Shooter shooterIntstance = Shooter.getInstance(); 
        if(!Shooter.getInstance().turretAlreadyZeroed){
            if (!turretMagnetBreak.get()) {
                turretMotor.setVoltage(0);
                turretMotor.setPosition(Constants.turretZeroPosition);
                this.setTurretPosition(0);
                shooterIntstance.turretAlreadyZeroed = true;
            } else {
                turretMotor.setVoltage(1.25);// was 3v
                shooterIntstance.turretAlreadyZeroed = false;//Int everything (sets the instanance to false tp show that the turrent has not been zerod)
            }
        }
        Logger.recordOutput("turretAlreadyZeroed", Shooter.getInstance().turretAlreadyZeroed);
        return shooterIntstance.turretAlreadyZeroed;
    }

    @Override
    public void setTurretZeroVoltage() {
        turretMotor.setVoltage(0.0);
    }

    

    //  public boolean intakePivotStalled() {
    //       // return pivot.getFault_StatorCurrLimit().getValue();turretMotor
    //       return false;
    //  }

    // TODO add later gravity zeroing is fine for now
    // public boolean hoodZeroed() {
    //     if(!Shooter.getInstance().hoodAlreadyZeroed){
    //         if (hoodMotor.getFault_StatorCurrLimit().getValue()){
    //             hoodMotor.setVoltage(0);
    //             hoodMotor.setPosition(85/360.0);
    //             setHoodPosition(45/360.0);
    //             Shooter.getInstance().hoodAlreadyZeroed = true;
    //         } else {
    //             hoodMotor.setVoltage(1);// was 3v
    //             Shooter.getInstance().hoodAlreadyZeroed = false;
    //         }
    //     }
    //     return Shooter.getInstance().hoodAlreadyZeroed;
    // }
}
