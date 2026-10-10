package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.RotTrlTransform3d;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.ShooterAngle;
import frc.robot.subsystems.Vision.ShooterAngleCalculator;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Feeder.FeederStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Turret.Turret;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private ShooterAngle pastShooterAngle = new ShooterAngle(0, 0, 0);
    public ShooterStates currentShooterState = ShooterStates.SAFE;
    public ShooterStates wantedShooterState = ShooterStates.SAFE;
    private static Shooter instance = null;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    public final FeederIO fIO;
    public final FlywheelIO fwIO;
    public final TurretIO tIO;
    public final ShooterIO sIO;
    public SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    // public final CommandXboxController coDriverController;
    // public final CommandXboxController coDriverController;
    public Feeder feeder = new Feeder();
    public Turret turret;
    public boolean turretAlreadyZeroed = false;
    public boolean hoodAlreadyZeroed = false;
    public Flywheel flywheel = new Flywheel();
    public final static double prefire = 0;
    public static boolean driverOverride = false;
    public static boolean coDriverOverride = false;
    public static boolean flywheelOverride = false;
    public static boolean ferryOverride = false;
    private String gameData;
    public double turretAim = -0.1;
    private ShooterAngle shooterAngle;
    private Translation2d hubPoseBlue = new Translation2d(4.6228, 4.02082);
    private Translation2d hubPoseRed = new Translation2d(11.88974, 4.02082);
    private Translation2d blueFerryOutpost = new Translation2d(4.6239 - 2,2.011-0.5);
    private Translation2d redFerryOutpost = new Translation2d(11.917 + 2,6.031+0.5);
    private Translation2d blueFerryDepot = new Translation2d(4.6239 - 2,6.03+0.5);
    private Translation2d redFerryDepot = new Translation2d(11.917 + 2,2.011-0.5);
    public boolean isAimedAtHub;
    public boolean isAimedAtFerry;
    public static boolean flywheelInToleranceOnce = false;
    public static int flywheelDebouncer = 10;
    public double hubBallSpeed = 6.7;
    public double hubFlywheelSpeed = 10;
    public double ferryBallSpeed = 6.7;
    public double ferryFlywheelSpeed = 10;

    public Pose2d aimpose;

    public static int flywheelToleranceThreshold = 10;
    public double flywheelTolerance = 3;

    public double manuelHood = 77; 
    public double manuelFlywheel = 40;

    public double hoodTargetPosition = Constants.maximumHoodPosition;

    //A couple variables designed to handle historisis.
    public int historisis = 0;
    public boolean lastSideDepot = false;
    public boolean currentSideDepot = false;
    public boolean allowSideSwap = false;

    double YToHubFerry = 0;
    double XToHubFerry = 0;


    public Shooter(FeederIO fIO, FlywheelIO fwIO, TurretIO tIO, ShooterIO sIO) {
        this.fIO = fIO;
        this.fwIO = fwIO;
        this.tIO = tIO;
        this.sIO = sIO;
        instance = this;
        turret = new Turret(tIO);
    }

    public static Shooter setInstance(FeederIO fIO, FlywheelIO fwIO, TurretIO tIO, ShooterIO sIO) {
        instance = new Shooter(fIO, fwIO, tIO, sIO);
        return instance;
    }

    public static Shooter getInstance() {

        if (instance == null) {
            throw new IllegalStateException("Shooter Instance Not Set");
        }
        return instance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ferryOverride", ferryOverride);
        Logger.recordOutput("Shooter/driverOverride", driverOverride);
        Logger.recordOutput("Shooter/coDriverOverride", coDriverOverride);
        Logger.recordOutput("Shooter/flywheelOverride", flywheelOverride);
        double startTime = Timer.getFPGATimestamp();
        // ApplyStates();
        handleStateTransitions();
        fIO.updateInputs(feederInputs);
        Logger.processInputs("Shooter/Feeder", feederInputs);
        fwIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        tIO.updateInputs(turretInputs);
        Logger.processInputs("Shooter/Turret and Hood", turretInputs);
        sIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter/Shooter", shooterInputs);
        // SmartDashboard.putBoolean("HubAcivity", isHubActive());
        Logger.recordOutput("isInTrenchZone", inEnterTrenchZone());
        Logger.recordOutput("trench danger zone", inTrenchDangerZone());
        Logger.recordOutput("Shooter/ShooterTimeMS", (Timer.getFPGATimestamp() - startTime)*1000.0);
    }

    public void ApplyStates() {
        Logger.recordOutput("turretPos",new Pose2d(getX(hubPoseRed.getX()),getY(hubPoseRed.getX()),new Rotation2d((turret.getTurretPosition()-0.25) * Math.PI * 2 + swerve.io.getPose2d().getRotation().getRadians())));
        switch (currentShooterState) {
            case SAFE:
                // stop the flywheel
                feeder.setFeederVelocity(FeederStates.OFF);
                flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                flywheelInToleranceOnce = false;
                turret.setHoodPosition(Constants.maximumHoodPosition);
                turret.io.setTurretZeroVoltage();
                break;
            case MANUAL:
                // joystick controlls turret and hood
                // tIO.setTurretPosition(getTurretPosFromJoystick());
                // tIO.setHoodPosition(getHoodPosFromJoystick());
                break;
            case UNJAM:
                //stops the flywheel
                // flywheel.setFlywheelVelocity(0);
                feeder.setFeederVelocity(FeederStates.UNJAM);
                break;
            case SPITTOCONTAINER:
                feeder.setFeederVelocity(FeederStates.SPITTING);
                flywheel.setFlywheelVelocity(FlywheelStates.SPITTOCONTAINER);
                turret.setHoodPosition(Constants.maximumHoodPosition);
                turret.setTurretPosition(-90/360.0);
                break;
            case MANUEL:
                Logger.recordOutput("manuel hood position", manuelHood); 
                Logger.recordOutput("manuel flywheel position", manuelFlywheel); 
                // isAimedAtHub = isAimedAtHub(); 
                turret.setTurretPosition(90/360.0);
                turret.setHoodPosition(manuelHood/360.0);
                if(driverOverride||coDriverOverride){
                    if(isAimedAtHub){
                        flywheel.setFlywheelVelocity(manuelFlywheel);
                        activateFeeder();
                    }
                }else{
                    feeder.setFeederVelocity(FeederStates.OFF);
                    flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                }
                break;
            case FERRY:
                isAimedAtFerry = aimFerry();
                turret.setHoodPosition(Constants.maximumHoodPosition);
                if(!swerve.isInAllianceZone()){
                    if(inEnterTrenchZone()){ 
                        if(inTrenchDangerZone()){
                            wantedShooterState = ShooterStates.TRENCH;
                        }
                    }
                    if(driverOverride || ferryOverride || coDriverOverride){
                        turret.setHoodPosition(hoodTargetPosition);
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        if(isAimedAtFerry){
                            if(flywheel.FlywheelInTolerance(flywheelTolerance)){
                                //Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTHUB;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<flywheelToleranceThreshold){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                            
                        } else {
                            flywheelDebouncer ++;
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                    }else{
                        if (flywheelOverride){
                            flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        }
                        else {
                            flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                        }
                        feeder.setFeederVelocity(FeederStates.OFF);
                        flywheelInToleranceOnce = false;
                    }
                }else{
                    if(!inEnterTrenchZone()){
                    wantedShooterState = ShooterStates.BUMP;
                    }
                }
                break;
            case AUTOFERRY:
                isAimedAtFerry = aimFerry();
                SmartDashboard.putBoolean("IsAimedAtFerry", isAimedAtFerry);
                turret.setHoodPosition(Constants.maximumHoodPosition);

                if(!swerve.isInAllianceZone()){
                    if(inEnterTrenchZone()){ 
                        if(inTrenchDangerZone()){
                            wantedShooterState = ShooterStates.TRENCH;
                        }
                    }

                        turret.setHoodPosition(hoodTargetPosition);
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);

                        if (isAimedAtFerry) {

                            if(flywheel.FlywheelInTolerance(flywheelTolerance)){
                                //Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTHUB;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;

                            } else if (flywheelDebouncer<flywheelToleranceThreshold){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else {
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                            
                        } else {
                            flywheelDebouncer ++;
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                }else{
                    if(!inEnterTrenchZone()){
                    wantedShooterState = ShooterStates.BUMP;
                    }
                }
                break;
            case HUB:
                Logger.recordOutput("rui is bouncing wrong", flywheelDebouncer);
                
                isAimedAtHub = isAimedAtHub();
                turret.setHoodPosition(Constants.maximumHoodPosition);
                    if(driverOverride || coDriverOverride){
                        turret.setHoodPosition(hoodTargetPosition);
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        feeder.setFeederVelocity(FeederStates.SPINUP);
                        if(isAimedAtHub){
                            if(flywheel.FlywheelInTolerance(flywheelTolerance)){
                                //Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTHUB;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<flywheelToleranceThreshold){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        } else {
                            flywheelDebouncer ++;
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                        if (flywheelOverride){
                            flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        }
                        else {
                            flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                        }
                    }
                break;
            case TRENCH:
             //7.5
                currentShooterState = ShooterStates.HUB;
                if((swerve.getRobotPose().getX() < 7.5 && Constants.isBlueAlliance) || (swerve.getRobotPose().getX() > 7.5 && !Constants.isBlueAlliance)){
                    isAimedAtHub();
                }else{
                    aimFerry();
                }
                turret.setHoodPosition(Constants.maximumHoodPosition);
                
                feeder.setFeederVelocity(FeederStates.OFF);
                if (flywheelOverride||driverOverride||coDriverOverride) {
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                }
                else {
                    flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                }
                if(!inTrenchDangerZone()){
                    if(swerve.isInAllianceZone()){
                        wantedShooterState = ShooterStates.HUB;
                    }else{
                        // wantedShooterState = ShooterStates.FERRY;
                        wantedShooterState = ShooterStates.HUB;
                    }
                }

                break;
            case AUTOHUB:
            isAimedAtHub = isAimedAtHub();

                if(swerve.isInAllianceZone()){
                    if(inEnterTrenchZone() && inTrenchDangerZone()){
                        turret.setHoodPosition(Constants.maximumHoodPosition);
                    }else{
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        turret.setHoodPosition(hoodTargetPosition);
                        if(isAimedAtHub){
                            //Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTHUB;
                            Intake.getInstance().wantedState = IntakeStates.ELEPHANTIASISPART2;
                            if(flywheel.FlywheelInTolerance(flywheelTolerance)){
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<flywheelToleranceThreshold){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        } else {
                            flywheelDebouncer ++;
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                    }

                }else{
                    wantedShooterState = ShooterStates.BUMP;
                    //Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;

                }
                break;
            case AUTONONFIRE:
                isAimedAtHub = isAimedAtHub();
                turret.setHoodPosition(Constants.maximumHoodPosition);
                flywheel.setFlywheelVelocity(FlywheelStates.OFF);
                feeder.setFeederVelocity(FeederStates.OFF);
                break;
            case AUTODEPOTSHOOT:
            isAimedAtHub = isAimedAtHub();

                if(swerve.isInAllianceZone()){
                    if(inEnterTrenchZone() && inTrenchDangerZone()){
                        turret.setHoodPosition(Constants.maximumHoodPosition);
                    }else{
                        flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                        turret.setHoodPosition(hoodTargetPosition);
                        if(isAimedAtHub){
                            //Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTHUB;
                            Intake.getInstance().wantedState = IntakeStates.AUTOINTAKING;
                            if(flywheel.FlywheelInTolerance(flywheelTolerance)){
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<flywheelToleranceThreshold){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        }
                    }

                }else{
                    wantedShooterState = ShooterStates.BUMP;
                    //Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;

                }
                break;
            case AUTOPRESHOOT:
                isAimedAtHub = isAimedAtHub();

                flywheel.setFlywheelVelocity(pastShooterAngle.turretFlywheelSpeed);
                turret.setHoodPosition(Constants.maximumHoodPosition);
                break;
            case BUMP:
                //figures out if were on our side our in the neutral zone and if were in auto
                // if (!swerve.isTilted(0, 3)) { 
                turret.setHoodPosition(Constants.maximumHoodPosition);
                    if (swerve.isInAllianceZone()) {
                        if (DriverStation.isAutonomousEnabled()){
                        wantedShooterState = ShooterStates.AUTOHUB;
                        } else{
                        wantedShooterState = ShooterStates.HUB;
                        }
                    } else {
                        if (DriverStation.isAutonomousEnabled()){
                        wantedShooterState = ShooterStates.AUTONONFIRE;
                        } else {
                        // wantedShooterState = ShooterStates.FERRY;
                        wantedShooterState = ShooterStates.HUB;
                        }
                    }
                // }
                break;
            case SPIT:
                //shoot but slower
                feeder.setFeederVelocity(FeederStates.SPITTING);
                flywheel.setFlywheelVelocity(FlywheelStates.SPIT);
                turret.setHoodPosition(turret.spitTurrentHood);
                turret.setTurretPosition(turretAim);
                break;
            case FIXEDFIRE:
                //second button that shoots the same shot everytime
                flywheel.setFlywheelVelocity(FlywheelStates.FIXEDFIRE);
                if(flywheel.io.FlywheelInTolerance(flywheelTolerance))
                    feeder.setFeederVelocity(FeederStates.FIXEDFIRE);
                else
                    feeder.setFeederVelocity(FeederStates.OFF);
                turret.setHoodPosition(75 / 360.0);
                turret.setTurretPosition(90 / 360.0);
                
                break;
            case ZERO:
                //makes the turret figure out where it is
                Robot.zeroingLights = true;
                turret.io.zeroHoodMotor();
                if(turret.io.zeroTurret()){
                    wantedShooterState = ShooterStates.HUB;
                    Robot.zeroingLights = false;
                }
                break;
            default:
                break;
        }

    }

    // state transitions for spit and manual needed
    public void handleStateTransitions() {
        if ((currentShooterState == ShooterStates.AUTOHUB || currentShooterState == ShooterStates.AUTODEPOTSHOOT) && wantedShooterState != currentShooterState)
            flywheelDebouncer = flywheelToleranceThreshold;
        switch (wantedShooterState) {
            case HUB:
                ferryOverride = false;
                // if we're on our side of the field
                // if (!swerve.isTilted(0, 3) && swerve.isInAllianceZone()) {// !tilted and in alliance
                // if(swerve.isInAllianceZone()){
                //     currentShooterState = ShooterStates.HUB;
                // }else{
                //     currentShooterState = ShooterStates.FERRY;
                // }
                currentShooterState = ShooterStates.HUB;
                break;
            case ZERO:
                currentShooterState = ShooterStates.ZERO;
                break;
            default:
                currentShooterState = ShooterStates.HUB;
                break;
        }
    }

    /**
     * Turns on the feeder if the flywheel is up to speed
     */
    public void activateFeeder(){
        if(flywheel.FlywheelInTolerance(flywheelTolerance) || flywheelInToleranceOnce){
            feeder.setFeederVelocity(FeederStates.SCORING);
            flywheelInToleranceOnce = true;
        }
        else{
            feeder.setFeederVelocity(FeederStates.OFF);
        }
    }

    /**
     * Determines if we are in danger of slaming the hood into the trench.
     * <p>
     * 
     * Uses our current speed and distance realtive to the trench to determine if we are in danger of hitting the trench, meaning we need to bring the hood back down.
     * 
     * @return true or false dependig on if we are in danger of hitting the hood on the trench.
     */
    public boolean inTrenchDangerZone(){
        double zeroSpeedDistance = 0.7;
        double coefficientForDistance = 1.5;
        double hoodFullSwingTime = 0.5; //TODO
        double trenchRelativeXVelocity = getTrenchRelativeVelocity().vxMetersPerSecond;
        Logger.recordOutput("dist to trench", getDistanceToClosestTrench());
        Logger.recordOutput("velocity dist to trench", zeroSpeedDistance + coefficientForDistance*trenchRelativeXVelocity*hoodFullSwingTime);
        if(trenchRelativeXVelocity > 0){
            return getDistanceToClosestTrench()<(zeroSpeedDistance + coefficientForDistance*trenchRelativeXVelocity*hoodFullSwingTime);
        }
        return getDistanceToClosestTrench() < zeroSpeedDistance;
    }
    


    /**Increments the timer for Historisis.
    * <br></br>
    *  This is used to stop the turret from violently and rapidly shifting between different ferry points when repeatedly crossing the center.
    **/
    
    public void processHistorisisTimer() {
        historisis++;

        // allow change if on different side for 1 second (50 cycles in 1 second)
        if (historisis > 1 * 50)
            allowSideSwap = true;

        if (lastSideDepot == currentSideDepot)
            return;

        allowSideSwap = false;
        historisis = 0;
    }




    /**
     * Gets the velocity of the robot relative to the trench(Trench Relative Velocity)
     * <p>
     * 
     * Trench Relative Velocity means that if you are moving towards the closest trench then your velocity is positive in the X direction and the Y stays the same, 
     * and if you are moving away the X velocity is negative and Y remains the same. 
     * Essentially inverting the X velocity to get it always pointing toward or away the trench.
     * 
     * This checks if our velocity is positive or negative and if we are in certain zones of the field to determine if we need to invert the velocity
     *
     * @return the Trench Relative Velocity based on the robots Field Relative Velocity
     */
    public ChassisSpeeds getTrenchRelativeVelocity(){
        // Trench relative velocity means that positive is moving towards the closest trench, and negative is moving away from the closest trench.
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());
        ChassisSpeeds fieldRelativeInverted = new ChassisSpeeds(fieldRelative.vxMetersPerSecond*-1, fieldRelative.vxMetersPerSecond, fieldRelative.omegaRadiansPerSecond);
        double robotX = swerve.getRobotPose().getX();
        boolean inversion;

        if(robotX < 4){ //if in blue alliance zone going to neutral
            inversion = false;
        }else if(robotX < 8.25){ // in the neutral zone on the side of blue alliance moving towards the red alliance
                inversion = true;
        }else if(robotX < 12.5){ // if moving towards red alliance zone and in the red alliance side of the neutral zone
            inversion = false;
        }else{
            inversion = true;
        }
        if(fieldRelative.vxMetersPerSecond < 0){
            return inversion ? fieldRelativeInverted : fieldRelative;
        }else{
            return inversion ? fieldRelative : fieldRelativeInverted;
        }
    }

    /**
     * Is the robot in a position to which it could drive through the trench(or enter the trench)
     * <p>
     * 
     * Uses the method {@link #getXToTarget()} to calclate the distance between the hood and both sets of trenches(red and blue) and compares them to get the least distance.
     *
     * @return whether or not the turret is in a place in which we could enter the trench from
     */
    public double getDistanceToClosestTrench(){
        Pose2d robotPose = swerve.getRobotPose();
        double distToRedTrench = Math.abs(getXToTarget(Constants.redTrenchX));
        double distToBlueTrench = Math.abs(getXToTarget(Constants.blueTrenchX));
        if(distToBlueTrench < distToRedTrench){
            return distToBlueTrench;
        }
        return distToRedTrench;
    }
     /**
     * Is the robot in a position to which it could drive through the trench(or enter the trench)
     * <p>
     * 
     * Checks if the robot is in the X areas that correspond with the width of the trench
     *
     * @return whether or not the turret is in a place in which we could enter the trench from
     */
    public boolean inEnterTrenchZone(){
        if(swerve.getRobotPose().getMeasureY().in(Units.Meters) < 2 || swerve.getRobotPose().getMeasureY().in(Units.Meters) > 6){
            return true;
        }
        return false;
    }

    public boolean cantShoot(){
            if ((isAimedAtHub || isAimedAtFerry) && swerve.isInAllianceZone() && isHubActive() && !swerve.isTilted(0, 3))
            return false;
        else
            return true;
    }

    /**
     * Get the distance from the turret to the hub
     * <p>
     * 
     * Uses the helper methods {@link #getXToTarget()} and {@link #getYToTarget()} to calculate the individual distances
     * and then uses the pythagorean theorem to calculate the distance between the two points
     *
     * @return the distance from the turret to the hub
     */
    public double getDistanceToHub(){
        double XToHub;
        double YToHub;
        if(Constants.isBlueAlliance){
            YToHub = getYToTarget(hubPoseBlue.getY());
            XToHub =  getXToTarget(hubPoseBlue.getX());
        } else {
            YToHub = getYToTarget(hubPoseRed.getY());
            XToHub =  getXToTarget(hubPoseRed.getX());
        }
        return Math.sqrt(YToHub*YToHub+XToHub*XToHub);
    }
    /**
     * Get the difference in the Y component of the field relative positions of a target and the turret.
     * <p>
     * 
     * Calculates the current position of the turret from robot position, rotation, and distance from the center 
     * then gets the difference in the Y components of the positions
     *
     * @param poseY the y component of the target field relative position
     * @return the difference in the y components of the turret and target
     */
    public double getYToTarget(double poseY){
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());
        return poseY - (swerve.io.getPose2d().getY()
            + Constants.TurretDistFromCenter
                * Math.sin(((swerve.io.getPose2d().getRotation().getRadians() + fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.lagTime) + Constants.TurretAngleFromCenter)) + fieldRelative.vyMetersPerSecond * ShooterAngleCalculator.lagTime);
    }

    /**
     * Get the difference in the X component of the field relative positions of a target and the turret.
     * <p>
     * 
     * Calculates the current position of the turret from robot position, rotation, and distance from the center 
     * then gets the difference in the X components of the positions
     *
     * @param poseX the X component of the target field relative position
     * @return the difference in the X components of the turret and target
     */
    public double getXToTarget(double poseX){
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());
        return poseX - (swerve.io.getPose2d().getX()
            + Constants.TurretDistFromCenter
                * Math.cos(((swerve.io.getPose2d().getRotation().getRadians() + fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.lagTime) + Constants.TurretAngleFromCenter)) + fieldRelative.vxMetersPerSecond * ShooterAngleCalculator.lagTime);
    }


    public double getVXOfRobot(ChassisSpeeds fieldRelative){
        return fieldRelative.vxMetersPerSecond -
            Math.sin(
                Constants.TurretAngleFromCenter 
                + swerve.getRobotPose().getRotation().getRadians() 
                + fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.lagTime
            )
            * fieldRelative.omegaRadiansPerSecond * Constants.TurretDistFromCenter;
    }

    public double getVYOfRobot(ChassisSpeeds fieldRelative){
        return fieldRelative.vyMetersPerSecond +
            Math.cos(
                Constants.TurretAngleFromCenter 
                + swerve.getRobotPose().getRotation().getRadians() 
                + fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.lagTime
            )
            * fieldRelative.omegaRadiansPerSecond * Constants.TurretDistFromCenter;
    }

    public double getY(double hubPoseY){
        
        return swerve.io.getPose2d().getY()
             + Constants.TurretDistFromCenter
                * Math.sin(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter);
    }

    public double getX(double hubPoseX){
        
        return swerve.io.getPose2d().getX()
            + Constants.TurretDistFromCenter
                * Math.cos(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter);
    }


    /**
     * Aims the turret and hood in order to make it into the hub.
     * <p>
     * 
     * Uses the wrapped turret angle given by {@link #GetProposedAngle()} and the hood angle interpolated from the LUT to command the turret and hood to the positions required to make it in the hub.
     *
     * @return if the hood and turret are within tolerance of their setpoint given by the LUT
     */
    public boolean isAimedAtHub() {
        double YToHub;
        double XToHub;
        if (Constants.isBlueAlliance) {
            YToHub = getYToTarget(hubPoseBlue.getY());
            XToHub = getXToTarget(hubPoseBlue.getX());
        } else {
            YToHub = getYToTarget(hubPoseRed.getY());
            XToHub = getXToTarget(hubPoseRed.getX());
        }

        Logger.recordOutput("ToHub",new Translation2d(getXToTarget(hubPoseRed.getX()),getYToTarget(hubPoseRed.getY())));
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());

        shooterAngle = ShooterAngleCalculator.getShooterAngle(
                            getVXOfRobot(fieldRelative),
                            getVYOfRobot(fieldRelative),
                            XToHub,
                            YToHub,
                    ShooterAngleCalculator.flywheelSpeedMapHub,
                    ShooterAngleCalculator.timeOfFlightMapHub,
                    ShooterAngleCalculator.hoodAngleMapHub
                        );

        if (shooterAngle != null) { // implement passing null when the input is oustisde the bounds of the lookuptable
            pastShooterAngle = shooterAngle;
        }

        double proposedAngle = GetProposedAngle();

        turret.setTurretPosition((proposedAngle - fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.turretLagTime)/(2*Math.PI));
        // turret.setTurretPosition(-0.25);
        
        Logger.recordOutput("CalculatedCorrectedTurretAngle", 180*proposedAngle/(Math.PI));

        // turret.setHoodPosition(pastShooterAngle.hoodRotation/(2.0*Math.PI));
        hoodTargetPosition = pastShooterAngle.hoodRotation/(2.0*Math.PI);
        Logger.recordOutput("CalculatedHoodAngle", pastShooterAngle.hoodRotation/(2*Math.PI));

        // return true;
        return (turret.hoodInTolerance(.005) && turret.turretInTolerance(0.06));
    }

    /**
     * Uses the LUT to determine the turret angle to aim towards the hub and then wraps it to be in the ROM of the turret on our robot
     * <p>
     * 
     * Uses the calculated angle from Newtons method with a LUT to get the field relative rotation, then converts that to be robot relative so it always aims no matter the rotation.
     *
     * @return the turret angle required to make it in the hub
     */
    public double GetProposedAngle(){
        double rotation = SwerveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

        rotation = rotation-Math.PI/2;

        double proposedAngle = (((pastShooterAngle.turretRotation - rotation) + Math.PI) % (2*Math.PI) - Math.PI);
        Logger.recordOutput("ProposedAngle", 180*proposedAngle/(Math.PI));

        if (
            (proposedAngle - 2*Math.PI) > Constants.minTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle - 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle - 2*Math.PI;
        }
        else if (
            (proposedAngle + 2*Math.PI) < Constants.maxTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle + 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle + 2*Math.PI;
        }
        return proposedAngle;
    }


    /**
     * Aims the turret and hood in order to make it into the hub.
     * <p>
     * 
     * Uses the wrapped turret angle given by {@link #GetProposedAngle()} and the hood angle interpolated from the LUT to command the turret and hood to the positions required to make it in the hub.
     *
     * @return if the hood and turret are within tolerance of their setpoint given by the LUT
     */
    

    public boolean aimFerry() {
        processHistorisisTimer();
        lastSideDepot = currentSideDepot;

        //Handles target based upon what alliance the bot is on
        if(!Constants.isBlueAlliance){
            double redFerryDepotDistance = Math.sqrt(Math.pow(redFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double redFerryOutpostDistance = Math.sqrt(Math.pow(redFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            
            if (redFerryDepotDistance <= redFerryOutpostDistance && allowSideSwap) {
                XToHubFerry = getXToTarget(redFerryDepot.getX());
                YToHubFerry = getYToTarget(redFerryDepot.getY());

                currentSideDepot = true;

            } else if (allowSideSwap) {
                XToHubFerry = getXToTarget(redFerryOutpost.getX());
                YToHubFerry = getYToTarget(redFerryOutpost.getY());

                currentSideDepot = false;
            }

        }else{
            double blueFerryDepotDistance = Math.sqrt(Math.pow(blueFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double blueFerryOutpostDistance = Math.sqrt(Math.pow(blueFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            
            if(blueFerryDepotDistance <= blueFerryOutpostDistance && allowSideSwap){
                XToHubFerry = getXToTarget(blueFerryDepot.getX());
                YToHubFerry = getYToTarget(blueFerryDepot.getY());

                currentSideDepot = true;
            
            }else if (allowSideSwap) {
                XToHubFerry = getXToTarget(blueFerryOutpost.getX());
                YToHubFerry = getYToTarget(blueFerryOutpost.getY());

                currentSideDepot = false;
            }
        }

        Logger.recordOutput("ToHub",new Translation2d(getXToTarget(hubPoseRed.getX()),getYToTarget(hubPoseRed.getY())));

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());

        shooterAngle = ShooterAngleCalculator.getShooterAngle(
                    getVXOfRobot(fieldRelative),
                    getVYOfRobot(fieldRelative),
                    XToHubFerry,
                    YToHubFerry,
                    ShooterAngleCalculator.flywheelSpeedMapFerry,
                    ShooterAngleCalculator.timeOfFlightMapFerry,
                    ShooterAngleCalculator.hoodAngleMapFerry
        );

        if (shooterAngle != null) {
            pastShooterAngle = shooterAngle;
        }

        double proposedAngle = GetProposedAngle();

        turret.setTurretPosition(proposedAngle/(2*Math.PI));
        hoodTargetPosition = pastShooterAngle.hoodRotation/(2.0*Math.PI);

        return (turret.hoodInTolerance(.005) && turret.turretInTolerance(0.1));
    }

    public boolean isHubActive() {
        double timer = Constants.timer.get();
        if(Robot.TeleopStarted){
            if (Robot.WonAuto()) {
                return (Constants.timer.get() <= 10 || (timer >= (35 - prefire) && timer <= 60)
                                || (timer >= (85 - prefire)));
                }else{
                    return (timer <= 35) || (timer >= (60 - prefire) && timer <= 85)
                        || (timer >= (110 - prefire));               
                }
        }else{
            return true;
        }
    }
}
