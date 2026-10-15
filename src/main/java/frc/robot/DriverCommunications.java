package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.PieceVision;
import frc.robot.subsystems.Vision.Vision;

public class DriverCommunications {
    static boolean CurrentHubState = Shooter.getInstance().isHubActive();
    static String NextPhaseIndication = "Transition Period";
    static String PhaseIndication = "Autonomous";
    public static Field2d fieldPose2d = new Field2d();
    public static StructPublisher<Pose3d> robot3d;
    public static StructPublisher<Pose3d> autoDriveLocation;
    public static double PhaseClock = 0;
    public static double TeleopTimer = Timer.getMatchTime();
    public static boolean hasAutoDriveTarget;
    public static StructArrayPublisher<Translation3d> bestPieceVisionDrivePaths = NetworkTableInstance.getDefault()
    .getStructArrayTopic("bestPieceVisionDrivePaths", Translation3d.struct).publish();
    public static StructArrayPublisher<Translation3d> idk = NetworkTableInstance.getDefault()
    .getStructArrayTopic("idk", Translation3d.struct).publish();



    public static StructArrayPublisher<Pose2d> best = NetworkTableInstance.getDefault().getStructArrayTopic("Pathplanner/best", Pose2d.struct).publish();
    public static StructArrayPublisher<Pose2d> curr = NetworkTableInstance.getDefault().getStructArrayTopic("Pathplanner/curr", Pose2d.struct).publish();
    




    static void allianceShift(int ShiftEndTime){
        PhaseClock = Math.floor(TeleopTimer - ShiftEndTime);
        if(!Shooter.getInstance().isHubActive()){
            NextPhaseIndication = "Our Shift";
            PhaseIndication = "Opposing Shift";
        }else{
            NextPhaseIndication = "Opposing Shift";
            PhaseIndication = "Our Shift";
        }

    }
    
    public static void pushToElastic() {
        SwerveSubsystem.getInstance().logd();
        if(Robot.TeleopStarted){ //if in teleop
            if (TeleopTimer > 130) { //in transition period
                PhaseIndication = "Transition Period";
                PhaseClock = Math.floor(TeleopTimer - 130);
                //Changing the "Next phase" indicator based on who won auto
                 if (Robot.WonAuto()){
                    NextPhaseIndication = "Opposing Shift";
                 }else{
                    NextPhaseIndication = "Our Shift";
                }

            }else if (TeleopTimer > 105) { //in alliance shift 1
                allianceShift(105);
            }else if (TeleopTimer > 80) { //in alliance shift 2
                allianceShift(80);
            }else if (TeleopTimer > 55) { //in alliance shift 3
                allianceShift(55);
            }else if (TeleopTimer > 30) { //in alliance shift 4
                PhaseClock = Math.floor(TeleopTimer - 30);
                NextPhaseIndication = "Endgame";
            }else{
                PhaseClock = Math.floor(TeleopTimer);
                NextPhaseIndication = "Match End";
                PhaseIndication = "Endgame";
            }

        }else{ //if in auto
            PhaseClock = Math.floor(TeleopTimer - 140);
            NextPhaseIndication = "Transition Phase";
            PhaseIndication = "Autonomous";

        }

        // set PhaseClock as the time before phase shift by subtracting timer from max
        // shift time
        SmartDashboard.putString("Next Phase", NextPhaseIndication);
        SmartDashboard.putString("Current Phase", PhaseIndication);
        SmartDashboard.putNumber("Phase Shift Countdown", PhaseClock);
        SmartDashboard.putData("Field", fieldPose2d);
        SmartDashboard.putNumber("Timer", Constants.timer.get());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Hub Activity", Shooter.getInstance().isHubActive());
        SmartDashboard.putBoolean("Cam/Front Right", Vision.getInstance().io.CameraConnect(0));
        SmartDashboard.putBoolean("Cam/Front Left", Vision.getInstance().io.CameraConnect(1));
        SmartDashboard.putBoolean("Cam/Left", Vision.getInstance().io.CameraConnect(2));
        SmartDashboard.putBoolean("Cam/Right", Vision.getInstance().io.CameraConnect(3));
        SmartDashboard.putBoolean("Cam/Back", Vision.getInstance().io.CameraConnect(4));
        SmartDashboard.putNumber("teleopTimer", TeleopTimer);
        SmartDashboard.putBoolean("hasAutoDriveTarget", hasAutoDriveTarget);

        robot3d.accept(SwerveSubsystem.getInstance().getRobotPose3d());
        if (SwerveSubsystem.getInstance().bestPlaceToGo != null)
        autoDriveLocation.accept(new Pose3d(SwerveSubsystem.getInstance().bestPlaceToGo.getX(),SwerveSubsystem.getInstance().bestPlaceToGo.getY(), 0.2, new Rotation3d()));
        else
        autoDriveLocation.accept(new Pose3d(0,0, 2, new Rotation3d()));

        //SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
        //SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
        // SmartDashboard.putBoolean("hasTargetjjj", ButtonConfig.hasTarget);
        // Translation3d[] poses = PieceVision.sortPosesByDistance(PieceVision.getCollectableFuel(Vision.getInstance().getPieceCamera().poses));
        // if (poses == null) {
        //     poses = new Translation3d[0];
        // }
        // Translation3d[] pieceVisionPaths = new Translation3d[poses.length + 1];
        // System.arraycopy(poses, 0, pieceVisionPaths, 1, poses.length);
        // pieceVisionPaths[0] = new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getTranslation());
        // bestPieceVisionDrivePaths.set(pieceVisionPaths);
        // SmartDashboard.putBoolean("hasTargets", ButtonConfig.hasTargets);
        // SmartDashboard.putNumber("idkL", Vision.getInstance().getPieceCamera().poses.length);
        // idk.set(Vision.getInstance().getPieceCamera().poses);
        // try {
        // best.set(SwerveSubsystem.getInstance().pieceVisionPath.getPathPoses().toArray(new Pose2d[0]));
        // } catch (Exception e) {
        //     best.set(new Pose2d[0]);
        // }
    }    


    public static void pushToElasticInit () {
        CameraServer.startAutomaticCapture();
        PathPlannerLogging.setLogActivePathCallback(poses -> curr.set(poses.toArray(new Pose2d[0])));

        robot3d = NetworkTableInstance.getDefault().getStructTopic("Robot3d", Pose3d.struct).publish();
        autoDriveLocation = NetworkTableInstance.getDefault().getStructTopic("AutoDriveLocation", Pose3d.struct).publish();
        // SmartDashboard.putNumber("tuneKp", 24);
        SmartDashboard.putBoolean("hasTargetjjj", false);
        // SmartDashboard.putNumber("tuneKi", 0.2);
        // SmartDashboard.putNumber("tuneKd", 0.8);

        // SmartDashboard.putNumber("tuneMaxErr", 0.5);
        // SmartDashboard.putNumber("tuneAddP", 0.05);
    }
}
