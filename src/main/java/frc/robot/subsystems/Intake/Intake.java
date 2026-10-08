package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveStates;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;

// import frc.robot.subsystems.Lights;
public class Intake extends SubsystemBase {
    /**
   * Tells the robot to intake during autonomous.
   *
   * @param True Sets the wantedState to INTAKING
   * @param False Does nothing
   */
    // public boolean autonomousIntake = false;
    // public IntakeStates autoToTeleopState = IntakeStates.SAFE;
 /**
   * The current state of the intake, which determines what the intake does
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Intake.IntakeStates#SAFE SAFE}
   * @param IntakeStates The intake states contain roller speed and intake position - {@link frc.robot.subsystems.Intake.IntakeStates IntakeStates}
   */
  
    public IntakeStates currentState = IntakeStates.SAFE;


    /**
   * The wanted state of the intake, which the subsystem attempts to set the {@link #currentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Intake.IntakeStates#ZERO ZERO}
   * @param IntakeStates The intake states contain roller speed and intake position - {@link frc.robot.subsystems.Intake.IntakeStates IntakeStates}
   */
    public IntakeStates wantedState = IntakeStates.ZERO;

    /**The way the intake accesses and manages motors. Changing this changes the nature of the entire system, although the only system it can be at this time of writing is IntakeIOTalonFX.*/
    public IntakeIO io;

    /**
     * The instance of Intake bring used by the bot at the moment. Creating another Intake object overrides this. <br></br>
     * Accessing it through the variable is not recommended; use #getInstance() instead.
     */
    public static Intake instance;

    public static boolean driverOverride = false;
    public static boolean driverElephantiasisPart2Override = false;


    /*True if the zeroing process has been completed. */
    public boolean alreadyZeroed = false;

    /**The logger for Intake.*/
    public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    /** Used for delaying the oscillation of elephantiasis. Misspelled. :/*/
    public int elephantiaissTimer = 0;

    /** Enabled at intake startup. */
    public boolean autoIntaked = false;

    public boolean enteringNewState;


    /**
     * Create a new instance of Intake. <br></br>
     * Highly recommended to avoid using if an existing Intake already exists; it will likely screw over quite a few systems and will generally not end well with the bot.
     * 
     * @param io A way for Intake to retrieve methods regarding motors. Pass in <pre> new IntakeIOTalonFX() </pre> for standard usage.
     */

    public Intake(IntakeIO io) {
        this.io = io;
        instance = this;

    }

    /** Retrieves the bot's current instance of Intake. Recommended. */
    public static Intake getInstance() {
        return instance;
    }

    /** Intake's way of processing information. Called via the loop it is overriding; not recommended to use this method. */
    @Override
    public void periodic() {
        // if (autonomousIntake) {
        //     wantedState = IntakeStates.INTAKING;
        // }only if it intake auto doesnt work
        // This will handle changing between states at the user's request.
        handleStateTransitions();

        // this is where states actually take effect.
        applyStates();
        
        inputs.currentState = currentState;
        inputs.wantedState = wantedState;
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    /** Internal helper method for #periodic(). */
    public void handleStateTransitions() {
        if (currentState != wantedState){
            enteringNewState =true;
        }
        switch (wantedState) {
            case SAFE:
                currentState = IntakeStates.SAFE;
                break;

            case INTAKING:
            if (currentState != IntakeStates.ZERO || alreadyZeroed == true) {
                if(driverElephantiasisPart2Override){
                    currentState = IntakeStates.ELEPHANTIASISPART2;
                } else if (driverOverride){
                    currentState = IntakeStates.INTAKING;
                } else if (SwerveSubsystem.getInstance().currentState == SwerveStates.AUTODRIVE) {
                    currentState = IntakeStates.INTAKING;
                } else {
                    wantedState = IntakeStates.EXTENDED;
                }
            }
                // Lights.getLightInstance().lightsWantedState = LightAnimations.INTAKING;
                break;
            case AUTOINTAKING:
            if (currentState != IntakeStates.ZERO || alreadyZeroed == true) {
                // only works if not climbing
                currentState = IntakeStates.AUTOINTAKING;
            }
                // Lights.getLightInstance().lightsWantedState = LightAnimations.INTAKING;
                break;

            case EXTENDED:
                // only works if not climbing
                if (driverOverride){
                    wantedState = IntakeStates.INTAKING;
                }
                currentState = IntakeStates.EXTENDED;
                break;

            case REVERSEINTAKING:
                // only works if not climbing
                currentState = IntakeStates.REVERSEINTAKING;
                // Lights.getLightInstance().lightsWantedState =
                // LightAnimations.REVERSEINTAKING;

                break;
            case ZERO:
                    currentState = IntakeStates.ZERO;
                break;
            case ELEPHANTIASISPART2:
                    if (currentState!=IntakeStates.ELEPHANTIASISPART2){
                        elephantiaissTimer = 0;
                        currentState = IntakeStates.ELEPHANTIASISPART2;
                    }
                break;
            default:
                currentState = IntakeStates.SAFE;
                break;

        }
        if(currentState == wantedState){
            enteringNewState = false;
            if(Math.abs(currentState.intakePosition - io.getIntakePosition()) <= Constants.currentLimitTolerance){
                io.getTalonFXPivotMotors()[0].getConfigurator().apply(Constants.limitedIntakeCurrentLimits);
                io.getTalonFXPivotMotors()[1].getConfigurator().apply(Constants.limitedIntakeCurrentLimits);
            }else{
                io.getTalonFXPivotMotors()[0].getConfigurator().apply(Constants.standardIntakeCurrentLimits);
                io.getTalonFXPivotMotors()[1].getConfigurator().apply(Constants.standardIntakeCurrentLimits);
            }
        }
    }
    /** Internal helper method for #periodic(). */
    public void applyStates() {

        switch (currentState) {

            case INTAKING:
            //motors on intake out
            io.setIntakeState(currentState);
            break;
            case AUTOINTAKING:
            //motors on intake out
            io.setIntakeState(currentState);
            break;

            case EXTENDED:
            //motors off intake out
            io.setIntakeState(currentState);
                
            break;

            case SAFE:
            //motors off intake in
            io.setIntakeState(currentState);
            break;

            case REVERSEINTAKING:
            io.setIntakeState(currentState);
            break;
            case ZERO:
                if (io.zeroIntake()) {
                    alreadyZeroed = true;
                    if (!DriverStation.isAutonomousEnabled()) {
                        wantedState = IntakeStates.EXTENDED;
                    }
                }else{
                    alreadyZeroed = false;
                }
                break;
            case ELEPHANTIASISPART2:
                if (elephantiaissTimer<0){
                    io.setIntakeState(IntakeStates.LESSPARTIALEXTENTION);
                } else {
                    io.setIntakeState(IntakeStates.PARTIALEXTENTION);
                }


                elephantiaissTimer++;
                if (elephantiaissTimer > 15){
                    elephantiaissTimer = -15;
                }
                break;
        default:
            io.setIntakeState(currentState);    
            break;
        }
    }
    /** Forces the bot to switch its state to the one passed in. Not recommended; has a tendency to break systems. */
    public void setCurrentState(IntakeStates state) {
        this.currentState = state;
    }


    /**Getter for currentState. Recommended.*/
    public IntakeStates getCurrentState() {
        return this.currentState;
    }

    /**Tells the bot to try to transition to the provided state. It goes through all the necessary systems to do so; recommended.*/
    public void setWantedState(IntakeStates state) {
        this.wantedState = state;
    }

    /*Getter for wantedState. Not recommended outside of niche applications.*/
    public IntakeStates getWantedState() {
        return this.wantedState;
    }
}
