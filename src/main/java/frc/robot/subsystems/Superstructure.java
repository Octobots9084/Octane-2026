package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveStates;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.*;//same
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.*;//same here
import frc.robot.subsystems.Shooter.Turret.Turret;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    public States currentState = States.SAFE;
    public States wantedState = States.SAFE;
    public States prevState = States.SAFE;
    public IntakeStates userRequestedIntakeState = IntakeStates.SAFE;

    boolean climbDescending = true;
    boolean climbAligned = false;
    public static Superstructure currentInstance;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    // public Climb climb = Climb.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Intake intake = Intake.getInstance();

    @Override
    public void periodic() {
        prevState = currentState;
        Logger.recordOutput("prevState", this.prevState);
        Logger.recordOutput("currentState", this.currentState);
        Logger.recordOutput("wantedState", this.wantedState);
        Logger.recordOutput("climbDescending", this.climbDescending);
        handleStateTransitions();
        applyStates();
    }

    public Superstructure() {
        currentInstance = this;
    }

    public static void setInstance(Superstructure instance) {
        currentInstance = instance;
    }

    public static Superstructure getInstance() {
        // if(currentInstance == null){
        // throw new IllegalStateException("Superstructure Instance not set");
        // }
        return currentInstance;
    }

    public States getCurrentState() {
        return currentState;
    }

    public States getWantedState() {
        return wantedState;
    }

    public void setCurrentState(States state) {
        currentState = state;
    }

    public void setWantedState(States state) {
        wantedState = state;
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SAFE:
                // if (climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3) {
                    currentState = States.SAFE;
                // }
                break;
            case SHOOTER:
                if(currentState != States.SHOOTER){
                    shooter.wantedShooterState = ShooterStates.HUB;
                }
                currentState = States.SHOOTER;
                break;
            case ZERO:
                if (!shooter.turretAlreadyZeroed || !intake.alreadyZeroed){
                    currentState = States.ZERO;
                }
                else if(DriverStation.isAutonomousEnabled()){
                    wantedState = States.AUTONONFIRE;
                    currentState = States.AUTONONFIRE;
                }
                else {
                    wantedState = States.SHOOTER;
                    currentState = States.SAFE;
                }
                break;
            case AUTO:
                if(DriverStation.isAutonomous() && currentState != States.ZERO){
                    this.currentState = States.AUTO;
                }
                break;
            case AUTOFERRY:
                if(DriverStation.isAutonomous() && currentState != States.ZERO){
                    this.currentState = States.AUTOFERRY;
                }
                break;
            case AUTODEPOTSHOOT:
                if(DriverStation.isAutonomous() && currentState != States.ZERO){
                    this.currentState = States.AUTODEPOTSHOOT;
                }
                break;
            case AUTONONFIRE:
                if(DriverStation.isAutonomous() && currentState != States.ZERO){
                    this.currentState = States.AUTONONFIRE;
                    shooter.wantedShooterState = ShooterStates.AUTONONFIRE;
                }
                break;
            case UNJAM:
                this.currentState = States.UNJAM;
                break;
            case FIXEDFIRE:
                this.currentState=States.FIXEDFIRE;
            break;
            case SPITTOCONTAINER:
                this.currentState = States.SPITTOCONTAINER;
                break;
            default:
                break; // do nothing
        }
    }

    public void applyStates() {
        switch (currentState) {
            case SAFE:
                stateSAFE();
                break;
            case MANUAL:
                stateMANUAL();
                break;
            case SHOOTER:
                stateSHOOTER();
                break;
            case ZERO:
                //Lights.getLightInstance().lightsWantedState = LightAnimations.ZEROED;
                if(stateZERO()){
                    if(DriverStation.isAutonomousEnabled()){
                        wantedState = States.AUTONONFIRE;
                        currentState = States.SAFE;
                    }else{
                        wantedState = States.SHOOTER;
                        currentState = States.SAFE;
                    }
                }
                break;
            case AUTONONFIRE:
                // swerve.wantedState = SwerveStates.IDLE;
                // shooter.wantedShooterState = ShooterStates.AUTONONFIRE;
                break;
            case AUTO:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.AUTOHUB;
                break;
            case AUTOFERRY:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.AUTOFERRY;
                break;
            case AUTODEPOTSHOOT:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.AUTODEPOTSHOOT;
                break;
            case FIXEDFIRE:
                shooter.wantedShooterState = ShooterStates.FIXEDFIRE;
                break;
            case UNJAM:
                stateUnJam();
                break;
            case SPITTOCONTAINER:
                swerve.wantedState = SwerveStates.IDLE;
                shooter.wantedShooterState = ShooterStates.SPITTOCONTAINER;
                break;
            default:
                throw new RuntimeException("Superstructure state is invalid! : " + currentState.toString());
        }


    }

    private void stateSAFE() {
        // climb.setClimbState(ClimbStates.IDLE);
        // // set shooter into safe state
        // intake.setWantedState(IntakeStates.SAFE);
        shooter.wantedShooterState = ShooterStates.SAFE;

    }
    private void stateMANUAL() {
        // TODO map buttons to direct inputs
        // turn off intake when starting
        // turn off shooter when starting        
    }

    private void stateSHOOTER() {
        // if (userRequestedIntakeState != Intake.getInstance().currentState) {
        //     Intake.getInstance().wantedState = userRequestedIntakeState;
        // }
        // if(prevState != States.SHOOTER){
        //     shooter.wantedShooterState = ShooterStates.HUB;
        //     prevState = States.SHOOTER;

        // }
        // shooter.wantedShooterState = ShooterStates.HUB;  
    }

    private void stateUnJam(){
        shooter.wantedShooterState = ShooterStates.UNJAM;
    }

    private boolean stateZERO(){
        shooter.wantedShooterState = ShooterStates.ZERO;
        if (!intake.alreadyZeroed) {
            intake.wantedState = IntakeStates.ZERO; 
        }
        return (shooter.turretAlreadyZeroed && intake.alreadyZeroed);//&& climb.alreadyZeroed);
    }

    public void stowForClimb(){
        //   Intake.getInstance().setWantedState(IntakeStates.SAFE);
          Shooter.getInstance().wantedShooterState = ShooterStates.SAFE;
          
    }
}