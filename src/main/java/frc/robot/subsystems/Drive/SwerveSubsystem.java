package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import choreo.trajectory.EventMarker;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ButtonConfig;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Vision.ShooterAngleCalculator;
import frc.robot.subsystems.Vision.Vision;

public class SwerveSubsystem extends SubsystemBase {


    private static SwerveSubsystem instance;
    /**
   * The wanted state of the swerve, which the subsystem attempts to set the {@link #currentState} to
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Drive.SwerveStates#MANUAL MANUAL}
   * @param SwerveStates The swerve states contain no information - {@link frc.robot.subsystems.Drive.SwerveStates SwerveStates}
   */
    public SwerveStates wantedState = SwerveStates.IDLE;
    /**
   * The current state of the swerve, which determines how the robot drives
   *
   * <br></br><b>Default State</b> - {@link frc.robot.subsystems.Drive.SwerveStates#IDLE IDLE}
   * @param SwerveStates The swerve states contain no information - {@link frc.robot.subsystems.Drive.SwerveStates SwerveStates}
   */

    public double xLimit = 1;
    public double yLimit = 1;
    public SwerveStates currentState = SwerveStates.IDLE;
    public SwerveIO io;
    public CommandXboxController driverController;
    public double maxVelocity;
    public double maxAngularVelocity;
    public double xLockWaitTime = 0.5;
    public double rotLockAngle = 0;
    public SwerveDriveBrake xLockbrake = new SwerveRequest.SwerveDriveBrake();
    public static Timer xLockTimer = new Timer();

    public boolean hasAutoDriveTarget;
    public Translation3d bestPlaceToGo;
    private final ProfiledPIDController rotationPID = new ProfiledPIDController(3.0, 0.2, 0.8, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 4));

    public static boolean pieceVision = false; //TODO piecevis
    //we need to figure out what to call it on
    // The robot pose estimator for tracking swerve odometry and applying vision corrections.
    
    //maxaccel
    private double lastVx = 0, lastVy = 0;
    private static final double kMaxAccel = 12; // m/s^2
    private static final double kDt = 0.02;

    public static PIDController angularPidcontroller = new PIDController(3, 0.5, 0);

    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    /**Handles the bot moving too quickly while shooting. */
    public SlewRateLimiter poslimiter;
    /**Handles the bot turning too quickly while shooting. */
    public SlewRateLimiter rotlimiter;

    public Command currentPieceVisionDriveCommand = null;

    public Command currentTrenchAlignCommand = null;

    public PathPlannerPath pieceVisionPath;
    
    public static StructArrayPublisher<Pose2d> travelAngleLog = NetworkTableInstance.getDefault().getStructArrayTopic("travelAngleLog", Pose2d.struct).publish();
    


    public SwerveSubsystem(
            SwerveIO io, CommandXboxController driverController, double maxAngularVelocity, double maxVelocity) {
        this.io = io;
        this.driverController = driverController;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVelocity = maxVelocity;

        this.poslimiter = new SlewRateLimiter(1.5);
        this.rotlimiter = new SlewRateLimiter(Math.PI*10);

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);///i uncommented all this and maybe it broke it idk
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        instance = this;
        registerNamedCommands();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null)
            throw new IllegalStateException("Swerve instance not set");
        else
            return instance;
    }

    public static SwerveSubsystem setInstance(SwerveIO io, CommandXboxController driverController,
            double maxAngularVelocity, double maxVelocity) {
        SwerveSubsystem instance = new SwerveSubsystem(io, driverController, maxAngularVelocity, maxVelocity);
        instance.registerNamedCommands();
        return instance;
    }

    public Pose2d getRobotPose() {
        return io.getPose2d();
    }

    public Pose3d getRobotPose3d() {
        return io.getPose3d();
    }

    public boolean isInAllianceZone() {
        Pose2d currentPose = this.getRobotPose();
        if (Constants.isBlueAlliance) {
            if (currentPose.getX() <= 4.6) {// x boundary for blue alliance zone
                return true;
            }
            return false;
        } else {
            if (currentPose.getX() >= 11.9) {// x boundary for red alliance zone
                return true;
            }
            return false;
        }
    }

    @Override
    public void periodic() {
        xLimit = SmartDashboard.getNumber("xLimit", 1.0);
        yLimit = SmartDashboard.getNumber("yLimit", 1.0);
        this.io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        currentState = handleStateTransition();
        Logger.recordOutput("Xrot", this.io.getRotation3d().getX());
        Logger.recordOutput("Yrot", this.io.getRotation3d().getY());
        Logger.recordOutput("Zrot", this.io.getRotation3d().getZ());
        Logger.recordOutput("Tilt",
        Math.acos(this.io.getRotation3d().toMatrix().get(2, 2)));
        SmartDashboard.putBoolean("onRamp", isTilted(0, 3));
        
        if (!pieceVision) { //TODO piecevis
            applyStates();
        }
        Logger.recordOutput("isInAllianceZone",this.isInAllianceZone());
        // Logger.recordOutput("front left absolute", io.getAbsoluteEncoderPositions(0));
        // Logger.recordOutput("front right absolute", io.getAbsoluteEncoderPositions(1));
        // Logger.recordOutput("back left absolute", io.getAbsoluteEncoderPositions(2));
        // Logger.recordOutput("back right absolute", io.getAbsoluteEncoderPositions(3));
    }

    public boolean isTilted(double wanted, double tolerance) { /////////////////////
        tolerance = Units.degreesToRadians(tolerance);
        Matrix<N3,N3> gyroMatrix = this.io.getRotation3d().toMatrix();
        
        // gyro mounted upside down so subtact PI radians out
        double tilt = Math.acos(gyroMatrix.get(2, 2)) - 0.015 - Math.PI; 
        return !MathUtil.isNear(wanted, tilt, tolerance);
    }

    public void registerNamedCommands () {

        //DO THE INTAKE THING (5 sec) for shoot
      NamedCommands.registerCommand("StartShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTO;}));
      NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTONONFIRE;}));
      
      NamedCommands.registerCommand("StartFerry", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTOFERRY;}));
      NamedCommands.registerCommand("StopFerry", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTONONFIRE;}));
      
      NamedCommands.registerCommand("StartPreShoot", new InstantCommand(() -> {Shooter.getInstance().wantedShooterState = ShooterStates.AUTOPRESHOOT;}));
      NamedCommands.registerCommand("StopPreShoot", new InstantCommand(() -> {Shooter.getInstance().wantedShooterState = ShooterStates.AUTONONFIRE;}));
      
      NamedCommands.registerCommand("DepotShoot", new InstantCommand(() -> {Superstructure.getInstance().wantedState = States.AUTODEPOTSHOOT;}).asProxy());
        
      NamedCommands.registerCommand("SafeIntake", new InstantCommand(() -> {Intake.getInstance().wantedState = IntakeStates.SAFE;}));
        
        NamedCommands.registerCommand("StartIntake", 
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> Intake.getInstance().alreadyZeroed),//so it doedsnt override auto init wanted=zero
                new InstantCommand(() -> {
                    Intake.getInstance().wantedState = IntakeStates.AUTOINTAKING;
                    Intake.getInstance().autoIntaked = true;
                })//,
            ).withTimeout(5)
        );
    
      NamedCommands.registerCommand("StartElephant", new InstantCommand(() -> {
        Intake.getInstance().wantedState = IntakeStates.ELEPHANTIASISPART2;
      }));

      NamedCommands.registerCommand("StopElephant", new InstantCommand(() -> {
        Intake.getInstance().wantedState = IntakeStates.EXTENDED;
      }));
      NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().wantedState = IntakeStates.EXTENDED;}));
    //   NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = true;}));
    //   NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {Intake.getInstance().autonomousIntake = false;}));
        //TODO: i want to be ablel to just set the intake state to intaking, but it difnt work the first time, not confident it has to be done like this tho
        //(it does a loop instead of 1 set state)
    }

    private SwerveStates handleStateTransition() {
        switch (wantedState) {
            case AUTODRIVE:
                currentState = SwerveStates.AUTODRIVE;
                return SwerveStates.AUTODRIVE;
            //redid how we handle the states that we switch to without modifications
            case MANUAL, IDLE, ROTATION_LOCK, REVERSE: 
                if (wantedState == SwerveStates.ROTATION_LOCK && currentState != SwerveStates.ROTATION_LOCK){
                    rotLockAngle = (Math.PI/2)*Math.round(getRobotPose().getRotation().getRadians()/(Math.PI/2));
                }
                return wantedState;
            case TRENCHLOCK:
                // rotLockAngle = 90;
                return wantedState;
            case SLOW:
                if (currentState != SwerveStates.IDLE)
                    return wantedState;
            case XLOCK:
                if(MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftXDeadband) != 0 || 
                MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftYDeadband) != 0 ||
                 MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband) != 0 ||
                  MathUtil.applyDeadband(driverController.getRightY(), Constants.rightYDeadband) != 0)
                {
                    if(Shooter.driverOverride || Shooter.coDriverOverride){
                        wantedState = SwerveStates.SLOW;
                    } else {
                        wantedState = SwerveStates.MANUAL;
                    }
                }
                return wantedState;
                
            default:
                return this.currentState;

        }
    }

    
    public void applyStates() {
        switch (currentState) {
            case MANUAL:
                if ((Shooter.driverOverride || Shooter.coDriverOverride) && this.isInAllianceZone()) {
                    wantedState = SwerveStates.SLOW;  
                }
                
                shouldXLock();

                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));

                break;
            case TRENCHLOCK:
                // if (Shooter.getInstance().inEnterTrenchZone()) {

                    //force it to not go away from it
                    // double halfFieldY = 4;//idk what ha;f is
                    // if (getRobotPose().getY() > halfFieldY) {//left?
                    //     // io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                    //     //         .withSpeeds(calculateSpeedsBasedOnJoystickInputsForceTrench(false))
                    //     //         .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                    // } else {//right?
                    //     // io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                    //     //         .withSpeeds(calculateSpeedsBasedOnJoystickInputsForceTrench(true))
                    //     //         .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                    // }
                // }

                // alignToTrenchEnterance();
                
                // if(Shooter.getInstance().inTrenchDangerZone()){
                //     //force it to stay right

                // }

                break;
            case SLOW:        
                //TODO test
                shouldXLock();
                ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs().div(1.5);

                double absolute = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
                double limited = poslimiter.calculate(absolute);
                if (absolute != 0){
                    double x = limited * (speeds.vxMetersPerSecond / absolute);
                    double y = limited * (speeds.vyMetersPerSecond / absolute);
                }


                // speeds.vxMetersPerSecond = x;
                // speeds.vyMetersPerSecond = y;

                speeds.omegaRadiansPerSecond = rotlimiter.calculate(speeds.omegaRadiansPerSecond);

                //end segment

                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(speeds)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));

                if ((!Shooter.driverOverride && !Shooter.coDriverOverride) || !this.isInAllianceZone()) 
                    wantedState = SwerveStates.MANUAL;
                break;
            case IDLE:

                break;
            case ROTATION_LOCK:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(calculateRotLockSpeedsBasedOnJoystickInputs())
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case REVERSE:
                io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-0.3, 0, 0))
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
                break;
            case XLOCK:
                //array of each angle we want the module to be in
                // new swervemodule states with desired angles

                //swervemodules. set control with swerve module states
                io.setSwerveState(xLockbrake);
                break;
            case AUTODRIVE:
                // if (hasAutoDriveTarget)
                //     driveToPosition(bestPlaceToGo); // Pass dt here
            default:
                break;

        }
    }

    public void logd() {
        ChassisSpeeds currentSpeeds = inputs.Speeds;

        double travelAngleRad = Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond);

        Pose2d[] travelAngleLogArray = new Pose2d[2];
        travelAngleLogArray[0] = getRobotPose();
        travelAngleLogArray[1] = new Pose2d(getRobotPose().getX() + Math.cos(travelAngleRad), getRobotPose().getY() + Math.sin(travelAngleRad), new Rotation2d());

        travelAngleLog.set(travelAngleLogArray);
    }

    public void zeroGyro() {
        io.zeroGyro();
    }

    public void alignToTrenchEnterance () {

        double halfFieldY = 4;
        double halfFieldX = 8.3;
        double redHalfTrenchX = 12;
        double blueHalfTrenchX = 4.5;
        
        ChassisSpeeds currentSpeeds = inputs.Speeds;


        PathConstraints constraints = new PathConstraints(maxVelocity, maxVelocity, maxAngularVelocity, maxAngularVelocity);

        double startX = 0;
        double startY = 0;

        Pose2d target = null;

        startX = getRobotPose().getX();
        startY = getRobotPose().getY();

        
        double y = getRobotPose().getY();
        double x = getRobotPose().getX();

        if (x < blueHalfTrenchX) {//alliance
            if (y > halfFieldY) {//right
                //top right align
                target = new Pose2d(3.5,7.45, new Rotation2d());//13.1=bottomred//3.5 is blue
            } else {
                //top left align
                target = new Pose2d(3.5,0.55, new Rotation2d());
            }
        } else if (x > redHalfTrenchX) {//enemy
            if (y > halfFieldY) {//right
                //bottom right align
                target = new Pose2d(13.1,7.45, new Rotation2d());
            } else {
                //bottom left align
                target = new Pose2d(13.1,0.55, new Rotation2d());
            }
        } else {//middle, use trajectory
            if (y > halfFieldY) {//right
                if (Math.abs(Units.radiansToDegrees(Math.atan2(currentSpeeds.vyMetersPerSecond, -currentSpeeds.vxMetersPerSecond))) < 90) {//red side
                    target = new Pose2d(5.7,7.45, new Rotation2d());
                } else {//blue side
                    target = new Pose2d(10.7,7.45, new Rotation2d());
                }
            } else {
                if (Math.abs(Units.radiansToDegrees(Math.atan2(currentSpeeds.vyMetersPerSecond, -currentSpeeds.vxMetersPerSecond))) < 90) {//red side
                    target = new Pose2d(5.7,.55, new Rotation2d());
                } else {//blue side
                    target = new Pose2d(10.7,.55, new Rotation2d());
                }
            }
        }

        if (target == null) return;


        Rotation2d travelDirection = new Rotation2d(
            target.getX() - startX,
            target.getY() - startY
        );

        List<Pose2d> poses = new ArrayList<Pose2d>();
        poses.add(new Pose2d(startX, startY, travelDirection));

        poses.add(new Pose2d(target.getX(), target.getY(), travelDirection));

        
        double endTargetRotation = 0;
        double rot = getRobotPose().getRotation().getDegrees();

        if (Math.abs(rot) < 90) {
            endTargetRotation = 0;
        } else {
            endTargetRotation = 180;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
    
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new ArrayList<RotationTarget>(),
            new ArrayList<PointTowardsZone>(),
            new ArrayList<ConstraintsZone>(),
            new ArrayList<>(),
            constraints,
            null,
            new GoalEndState(1, Rotation2d.fromDegrees(endTargetRotation)),
            false
        );
        path.preventFlipping = true;
        currentTrenchAlignCommand = AutoBuilder.followPath(path);
    }




    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0),
        // Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1),
        // Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0),
        // Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);
        double xVelocity = xMagnitude * maxVelocity;
        double yVelocity = yMagnitude * maxVelocity;

        double angularVelocity = angularMagnitude * maxAngularVelocity;

        double[] v = limitAccel(xVelocity*xLimit, yVelocity*yLimit);

        if (Constants.isBlueAlliance) {   
            return new ChassisSpeeds(-v[0], -v[1], angularVelocity);
        }
        return new ChassisSpeeds(v[0], v[1], angularVelocity);
    }
    
    public double getXToTarget(double poseX){
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(io.getChassisSpeeds(), getRobotPose().getRotation());
        return poseX - (getRobotPose().getX()
            + Constants.TurretDistFromCenter
                * Math.cos(((getRobotPose().getRotation().getRadians() + fieldRelative.omegaRadiansPerSecond * ShooterAngleCalculator.lagTime) + Constants.TurretAngleFromCenter)) + fieldRelative.vxMetersPerSecond * ShooterAngleCalculator.lagTime);
    }

    public double getDistanceToClosestTrench(){
        double distToRedTrench = Math.abs(getXToTarget(Constants.redTrenchX));
        double distToBlueTrench = Math.abs(getXToTarget(Constants.blueTrenchX));
        if(distToBlueTrench < distToRedTrench){
            return distToBlueTrench;
        }
        return distToRedTrench;
    }

    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputsForceTrench(boolean onRight) {
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        
        double rawRightX = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);

        // Directly set the target angle based on stick position
        if (rawRightX > 0.9) {
            rotLockAngle = 0.0; // Replace with your desired angle when pushing right
        } else if (rawRightX < -0.9) {
            rotLockAngle = Math.PI; // Replace with your desired angle when pushing left
        }

        double xVelocity = xMagnitude * maxVelocity;
        double yVelocity = yMagnitude * maxVelocity;


        double rot = getRobotPose().getRotation().getDegrees();

        //0-360

        
        if (Math.abs(rot) < 90) {
            rotLockAngle = 0;
        } else {
            rotLockAngle = 180;
        }
        
        double dR = 0;
        if (rotLockAngle == 0) {
            dR = rotLockAngle - rot;
        } else {
            if (Math.signum(rot) < 0) {
                rot += 360;
            }
                dR = rotLockAngle - rot;
        }



        double RotVelocity = Math.signum(dR) * ((-0.000375 * Math.pow(Math.abs(dR), 2)) + (0.100058 * Math.abs(dR)));

        boolean goingCorrectWay = false;
        if (onRight) {
            if (yVelocity < 0) { // forces right
            goingCorrectWay = true;
            }
        } else {
            if (yVelocity > 0) { // forces left
            goingCorrectWay = true;
            }
        }
        //right = 0.45 - 0.65
        //left  = 7.35 - 7.55

        //maxSpeed = 3.75
        double distFromEdge = getRobotPose().getY();
        if (distFromEdge > 4) {
            distFromEdge = 8 - distFromEdge;
        }

        if (goingCorrectWay) {
            if (distFromEdge < .45) {
                yVelocity *= 0.1;
            } else if (distFromEdge < .65) {
                yVelocity *= 0.2;
            } else if (distFromEdge < .8) {
                yVelocity *= 0.3;
            } else if (distFromEdge < 2) {
                yVelocity *= 1.2;
            }
        } else {
            yVelocity *= 0.1;
        }

        if (getDistanceToClosestTrench() < 1.5 && distFromEdge > .8) {
            xVelocity *= 0.6;
        }
        else if (getDistanceToClosestTrench() < 2 && distFromEdge > 1.5) {
            xVelocity *= 0.8;
        } 
        // else
        // if (getDistanceToClosestTrench() < 3 && distFromEdge < 2.5) {
        //     xVelocity *= 0.8;
        // } 
        

        if (Constants.isBlueAlliance) {
            return new ChassisSpeeds(-xVelocity, -yVelocity, RotVelocity);
        }
        return new ChassisSpeeds(xVelocity, yVelocity, RotVelocity);
    }

    public ChassisSpeeds calculateRotLockSpeedsBasedOnJoystickInputs() {
        // double yMagnitude = MathUtil.applyDeadband(driverLeft.getRawAxis(0),
        // Constants.leftYDeadband);
        // double xMagnitude = -MathUtil.applyDeadband(driverLeft.getRawAxis(1),
        // Constants.leftXDeadband);
        // double angularMagnitude = -MathUtil.applyDeadband(driverRight.getRawAxis(0),
        // Constants.rightXDeadband);
        double yMagnitude = MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftYDeadband);
        double xMagnitude = MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftXDeadband);
        if (getRobotPose().getRotation().getRadians()-rotLockAngle<0.3){
            if (-MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband)>0.9){
                rotLockAngle = (rotLockAngle + Math.PI/2) % (Math.PI*2);
            }
            else if (-MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband)<-0.9){
                rotLockAngle = (rotLockAngle - Math.PI/2) % (Math.PI*2);
                
            }
        }
        double angularMagnitude = -MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband);
        double xVelocity = xMagnitude * maxVelocity*xLimit;
        double yVelocity = yMagnitude * maxVelocity*yLimit;

        double RotVelocity = angularPidcontroller.calculate(getRobotPose().getRotation().getRadians(),rotLockAngle);

        if (Constants.isBlueAlliance) {   
            return new ChassisSpeeds(-xVelocity, -yVelocity, RotVelocity);
        }
        return new ChassisSpeeds(xVelocity, yVelocity, RotVelocity);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveFieldRelative(fieldRelativeSpeeds);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        // SmartDashboard.putNumberArray("Vision_STDEVS", stdDevs.getData());
        // poseEstimator.resetPose(visionMeasurement);

        io.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public void shouldXLock(){
        if(MathUtil.applyDeadband(driverController.getLeftX(), Constants.leftXDeadband) == 0
        && MathUtil.applyDeadband(driverController.getLeftY(), Constants.leftYDeadband) == 0
        && MathUtil.applyDeadband(driverController.getRightX(), Constants.rightXDeadband) == 0
        && MathUtil.applyDeadband(driverController.getRightY(), Constants.rightYDeadband) == 0) {


            xLockTimer.start();
            if (xLockTimer.get() >= xLockWaitTime){
                xLockTimer.stop();
                wantedState = SwerveStates.XLOCK;
            }
        
        } else {
            xLockTimer.stop();
            xLockTimer.reset();
        }
    }

    public SwerveStates getCurrentState() {
        return this.currentState;
    }

    // public void driveToPosition(Translation3d pos) {

    
    //     try{
    //         PathPlannerPath path = createPathToPos(pos);
    //         CommandScheduler.getInstance().schedule(AutoBuilder.followPath(path));
    //     } catch (Exception e) {
            
    //         return;
    //     }
    // }

    public void collectFuels () {

    if (pieceVisionPath == null) return;
        try{
            // for (Translation3d pose : poses) { // bestPlaceToGo.set(new Pose3d(best.getX(), best.getY(), 0.08, new Rotation3d()));
        // fakeRobot3d.accept(testRobotPos);

                // Commands.sequence(sequence);
                
            currentPieceVisionDriveCommand = AutoBuilder.followPath(pieceVisionPath);
            CommandScheduler.getInstance().schedule(currentPieceVisionDriveCommand);
            
            // driveUntilAtEndPos(pieceVisionPath, 0.05);

            // double poseToleranceMeters = 0.05;

            // Command path2 = currentPieceVisionDriveCommand.until(() -> pieceVisionPath.getPathPoses().get(pieceVisionPath.getPathPoses().size() - 1).getTranslation().getDistance(getRobotPose().getTranslation()) < poseToleranceMeters);
            // CommandScheduler.getInstance().schedule(AutoBuilder.followPath(path));
            // CommandScheduler.getInstance().schedule(path2);
        } catch (Exception e) {
            
            return;
        }
    }

    /*
     MUST BE A 2 POINTP 
     */
    // public boolean driveUntilAtEndPos (PathPlannerPath path, double toleranceInMeters) {
    //     if (path == null) return true;
    //     if (path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation().getDistance(getRobotPose().getTranslation()) < toleranceInMeters) {
    //         return true;
    //     }

    //     List<RotationTarget> rotationTargets = path.getRotationTargets();
    //     List<Waypoint> waypoints = path.getWaypoints();

    //     // RotationTarget newRotationTarget = new RotationTarget(, null)

    //     waypoints.set(0, PathPlannerPath.waypointsFromPoses(new Pose2d(getRobotPose().getX(), getRobotPose().getY(), new Rotation2d(path.getPathPoses().get(1).getX() - getRobotPose().getX(),path.getPathPoses().get(1).getY() - getRobotPose().getY()))).get(0));

    //     PathConstraints constraints = new PathConstraints(0.5, 0.5, 0.5 * Math.PI, 0.5 * Math.PI);
    //     // posesList.add(new Pose2d(target.getX(), target.getY(), travelDirection));
    //     //         Rotation2d targetChassisRotation = Rotation2d.fromRadians(
    //     //             Math.atan2(target.getY() - startY, target.getX() - startX)
    //     //         );

    //     //         if (k != poses.length-1) {
    //     //             rotationTargets.add(new RotationTarget(k+1, targetChassisRotation)); // Index 1
    //     //         } else {
    //     //             endTargetRotation = targetChassisRotation;
    //     //         }
        
    //         PathPlannerPath path2 = new PathPlannerPath(
    //             waypoints,
    //             path.getRotationTargets(),
    //             new ArrayList<PointTowardsZone>(),
    //             new ArrayList<ConstraintsZone>(),
    //             new ArrayList<>(),
    //             constraints,
    //             null,
    //             new GoalEndState(0, Rotation2d.fromRadians(Math.atan2(path.getPathPoses().get(1).getY() - getRobotPose().getY(), path.getPathPoses().get(1).getX() - getRobotPose().getX()))),
    //             false
    //         );
    //         path.preventFlipping = true;

    //     CommandScheduler.getInstance().schedule(AutoBuilder.followPath(path2));

    //     return false;
    // }

    private double[] limitAccel(double vx, double vy) {
        double dx = vx - lastVx;
        double dy = vy - lastVy;
        double dist = Math.hypot(dx, dy);
        double maxDelta = kMaxAccel * kDt;
        if (dist > maxDelta) {
            double scale = maxDelta / dist;
            dx *= scale;
            dy *= scale;
        }
        lastVx += dx;
        lastVy += dy;
        return new double[] {lastVx, lastVy};
    }



    public PathPlannerPath createPathsToPos(Translation3d[] poses) {
        PathConstraints constraints = new PathConstraints(0.5, 0.5, 0.5 * Math.PI, 0.5 * Math.PI);

        Rotation2d endTargetRotation = null;
        double startX;
        double startY;
        Translation3d target;

        List<Pose2d> posesList = new ArrayList<Pose2d>();
        List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();

        for (int k = 0; k < poses.length; k++) {
            if (k == 0) {
                startX = getRobotPose().getX();
                startY = getRobotPose().getY();
                target = poses[0];
                Rotation2d travelDirection = new Rotation2d(
                    target.getX() - startX,
                    target.getY() - startY
                );
                posesList.add(new Pose2d(startX, startY, travelDirection));
            } else {
                startX = poses[k-1].getX();
                startY = poses[k-1].getY();
            }

            target = poses[k];

            Rotation2d travelDirection = new Rotation2d(
                target.getX() - startX,
                target.getY() - startY
            );

                posesList.add(new Pose2d(target.getX(), target.getY(), travelDirection));
                Rotation2d targetChassisRotation = Rotation2d.fromRadians(
                    Math.atan2(target.getY() - startY, target.getX() - startX)
                );

                if (k != poses.length-1) {
                    rotationTargets.add(new RotationTarget(k+1, targetChassisRotation)); // Index 1
                } else {
                    endTargetRotation = targetChassisRotation;
                }
        }

        if (posesList != null && !posesList.isEmpty()) {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(posesList);
        
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                new ArrayList<PointTowardsZone>(),
                new ArrayList<ConstraintsZone>(),
                new ArrayList<>(),
                constraints,
                null,
                new GoalEndState(0, endTargetRotation),
                false
            );
            path.preventFlipping = true;

            return path;
        }
        return null;
    }

    // public PathPlannerPath[] createPathsToPos(Translation3d[] poses) {
    //     PathPlannerPath[] paths = new PathPlannerPath[poses.length];
    //     PathConstraints constraints = new PathConstraints(2, 2, 1 * Math.PI, 1 * Math.PI);

    //     Rotation2d previousEndRotation = getRobotPose().getRotation();

    //     for (int k = 0; k < poses.length; k++) {
    //         List<Pose2d> posesList = new ArrayList<Pose2d>();
    //         List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();

    //         double startX;
    //         double startY;

    //         if (k == 0) {
    //             startX = getRobotPose().getX();
    //             startY = getRobotPose().getY();
    //         } else {
    //             startX = poses[k - 1].getX();
    //             startY = poses[k - 1].getY();
    //         }
    //         Translation3d target = poses[k];

    //         Rotation2d travelDirection = new Rotation2d(
    //             target.getX() - startX,
    //             target.getY() - startY
    //         );

    //         posesList.add(new Pose2d(startX, startY, travelDirection));
    //         rotationTargets.add(new RotationTarget(0, previousEndRotation)); // Index 0

    //         posesList.add(new Pose2d(target.getX(), target.getY(), travelDirection));

    //         Rotation2d targetChassisRotation = Rotation2d.fromRadians(
    //             Math.atan2(target.getY() - startY, target.getX() - startX)
    //         );
            
    //         rotationTargets.add(new RotationTarget(1, targetChassisRotation)); // Index 1

    //         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(posesList);

    //         int endV = 0;
    //         if (poses.length - 1 != k) {
    //             endV = 2; // Keep moving if there are more path segments coming up (matches max vel)
    //         }

    //         PathPlannerPath path = new PathPlannerPath(
    //             waypoints,
    //             rotationTargets,
    //             new ArrayList<PointTowardsZone>(),
    //             new ArrayList<ConstraintsZone>(),
    //             new ArrayList<>(),
    //             constraints,
    //             null,
    //             new GoalEndState(endV, targetChassisRotation),
    //             false
    //         );

    //         path.preventFlipping = true;
    //         paths[k] = path;

    //         // Carry this segment's end rotation forward as the next segment's start rotation
    //         previousEndRotation = targetChassisRotation;
    //     }

    //     return paths;
    // }






    /*
     

    public PathPlannerPath[] createPathToPos (Translation3d[] poses) {
        List<Pose2d> posesList = new ArrayList<Pose2d>();

        List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();
        // List
        //     new RotationTarget(0.0, ,   // Look straight at start
        //     new RotationTarget(1.0, Rotation2d.fromDegrees(90))  // Rotate to face 90 degrees at waypoint 1
        // );
        
        // 1. Calculate the direct angle from your current position to the VERY FIRST target pose
    Translation3d firstTarget = poses[0];
    Rotation2d travelDirection = new Rotation2d(
        firstTarget.getX() - getRobotPose().getX(),
        firstTarget.getY() - getRobotPose().getY()
    );

    for (int i = 0; i < poses.length + 1; i++) {
        if (i == 0) {
            // OVERRIDE START POSE ROTATION: Use travel direction instead of actual robot gyro heading
            posesList.add(new Pose2d(getRobotPose().getTranslation(), travelDirection));
            rotationTargets.add(new RotationTarget(0, Rotation2d.fromRadians(0)));
        } else {
            Translation3d pose = poses[i - 1];
            
            // If you have multiple segments, calculate the heading to the NEXT point.
            // For a single point, this will just be the same travelDirection.
            Rotation2d segmentDirection = travelDirection;
            if (i < poses.length) {
                Translation3d nextPose = poses[i];
                segmentDirection = new Rotation2d(nextPose.getX() - pose.getX(), nextPose.getY() - pose.getY());
            }

            // FORCE TRAVEL DIRECTION: This guarantees the spline handles face each other perfectly
            posesList.add(new Pose2d(pose.getX(), pose.getY(), segmentDirection));
            
            // Keep your custom holonomic rotation target (where the bumpers face)
            rotationTargets.add(new RotationTarget(i, Rotation2d.fromRadians(Math.atan2(pose.getY() - getRobotPose().getY(), pose.getX() - getRobotPose().getX()))));
        }
    }

    // Pass to waypoints from poses as usual
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(posesList);

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
                                            //velocity,acceleration,angularvelocity,angularacceleration

        // PathPlannerPath path = new PathPlannerPath(
        //         waypoints,
        //         constraints,
        //         null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        //         new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        // );          //double velocityMPS, Rotation2d rotation

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            rotationTargets,                        // Custom directions
            new ArrayList<PointTowardsZone>(),      // Empty zones
            new ArrayList<ConstraintsZone>(),
            new ArrayList<>(),           // Empty markers
            constraints,                            // Path constraints
            null,                                   // No initial state
            new GoalEndState(0.0, rotationTargets.get(rotationTargets.size()-1).rotation()), // Match target
            false                                   // Not reversed
        );
    // List<Waypoint> waypoints, 
    // List<RotationTarget> holonomicRotations, 
    // List<PointTowardsZone> pointTowardsZones, 
    // List<ConstraintsZone> constraintZones, 
    // List<EventMarker> eventMarkers, 
    // PathConstraints globalConstraints, 
    // IdealStartingState idealStartingState, 
    // GoalEndState goalEndState, 
    // boolean reversed) {
      
        path.preventFlipping = true;

        return path;
    }
     */

    /*
     public PathPlannerPath createPathToPos (Translation3d pose) {
        List<Pose2d> posesList = new ArrayList<Pose2d>();

        List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();
        // List
        //     new RotationTarget(0.0, ,   // Look straight at start
        //     new RotationTarget(1.0, Rotation2d.fromDegrees(90))  // Rotate to face 90 degrees at waypoint 1
        // );
        
            posesList.add(getRobotPose());
            posesList.add(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)));
            rotationTargets.add(new RotationTarget(0, Rotation2d.fromRadians(Math.atan2(pose.getY() - getRobotPose().getY(), pose.getX() - getRobotPose().getX()))));
            rotationTargets.add(new RotationTarget(1, Rotation2d.fromRadians(Math.atan2(pose.getY() - getRobotPose().getY(), pose.getX() - getRobotPose().getX()))));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            posesList
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 4 * Math.PI, 8 * Math.PI); // The constraints for this path.
                                            //velocity,acceleration,angularvelocity,angularacceleration

        // PathPlannerPath path = new PathPlannerPath(
        //         waypoints,
        //         constraints,
        //         null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        //         new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        // );          //double velocityMPS, Rotation2d rotation

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            rotationTargets,                        // Custom directions
            new ArrayList<PointTowardsZone>(),      // Empty zones
            new ArrayList<ConstraintsZone>(),
            new ArrayList<>(),           // Empty markers
            constraints,                            // Path constraints
            null,                                   // No initial state
            new GoalEndState(0.0, Rotation2d.fromDegrees(90)), // Match target
            false                                   // Not reversed
        );
    // List<Waypoint> waypoints, 
    // List<RotationTarget> holonomicRotations, 
    // List<PointTowardsZone> pointTowardsZones, 
    // List<ConstraintsZone> constraintZones, 
    // List<EventMarker> eventMarkers, 
    // PathConstraints globalConstraints, 
    // IdealStartingState idealStartingState, 
    // GoalEndState goalEndState, 
    // boolean reversed) {
      
        path.preventFlipping = true;

        return path;
    }
     */



    /*
     
    public PathPlannerPath createPathToPos (Translation3d pose) {
        // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        //         new Pose2d(pos.getX(), pos.getY(), Rotation2d.fromRadians(90))//Math.atan2(pos.getY() - getRobotPose().getY(), pos.getX() - getRobotPose().getX())
        // );
        List<Pose2d> posesList = new ArrayList<Pose2d>();

        Translation3d endPose = new Translation3d();
// Translation3d(1,2,0.1);
        // posesList.add(new Pose2d(1, 2, new Rotation2d(0)));//Rotation2d.fromRadians(Math.atan2(2 - getRobotPose().getY(), 1 - getRobotPose().getX())))
        for (Translation3d pose : poses) {
            posesList.add(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromRadians(Math.atan2(pose.getY() - getRobotPose().getY(), pose.getX() - getRobotPose().getX()))));
            endPose = pose;
        }
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            // posesList
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 4 * Math.PI, 8 * Math.PI); // The constraints for this path.
                                            //velocity,acceleration,angularvelocity,angularacceleration

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(30)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );          //double velocityMPS, Rotation2d rotation

        path.preventFlipping = true;
        return path;
    }
     */
}