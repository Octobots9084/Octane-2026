package frc.robot.subsystems.Vision;

import java.lang.constant.Constable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriverCommunications;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class PieceVision {
    private double inchesToMetersRatio = 0.0254;
    private Transform3d intakeCameraPosition;
    private final Translation3d testRobotPos = new Translation3d(0.32,0.32,0.35);

    private PhotonCamera camera;// = new PhotonCamera(Constants.frontRightCameraName);
    public List<PhotonTrackedTarget> targets = List.of();
    private double yawRotation;
    private double xTransform;


    private static final double FX = 762.32;
    private static final double FUEL_DIAMETER_METERS = 0.1524; // 6 inches in meters
    private static final double HALF_FUEL_METERS = FUEL_DIAMETER_METERS / 2.0;


    private double halfFuel = 150/2; //TODO make right

    private int fieldMaxX = 17;//TODO: make real nums, round up
    private int fieldMaxY = 8;
    
    public Translation3d[] poses;
    Translation3d best;
    public Map<Translation3d, Double> values = new HashMap<Translation3d, Double>();
    public StructArrayPublisher<Translation3d> fuelLog = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Fuels", Translation3d.struct).publish();
    public StructPublisher<Pose3d> bestPlaceToGo = NetworkTableInstance.getDefault()
    .getStructTopic("bestPlaceToGo", Pose3d.struct).publish();

    public StructPublisher<Translation3d> fakeRobot3d = NetworkTableInstance.getDefault().getStructTopic("fakeRobot3d", Translation3d.struct).publish();


    private int numTargets = 0;

    
    // public PieceVision(String photonCameraName, Transform3d robotToCamera) {
    //     camera = new PhotonCamera(photonCameraName);
    //     yawRotation = robotToCamera.getRotation().getZ();
    //     xTransform = Math.abs(robotToCamera.getTranslation().getX());
    //     this.intakeCameraPosition = robotToCamera;
    //     Constants.centerToCameraDefaultPosition = robotToCamera;
    //     // SmartDashboard.putNumber("roll", 0.66);
    //     // SmartDashboard.putNumber("pitch", -9.5);
    //     poses = new Translation3d[0];
    //     // poses[0] = new Translation3d(1,2,0.1);
    //     // poses[1] = new Translation3d(4,3,0.1);
    // }


    public void updateIntakeCameraPosition () {
        // intakeCameraPosition = Constants.centerToCameraDefaultPosition.plus(new Transform3d(Intake.getInstance().io.getIntakePosition()*inchesToMetersRatio,0,0,new Rotation3d()));
        intakeCameraPosition = Constants.centerToCameraDefaultPosition.plus(new Transform3d(
            Units.inchesToMeters(((9.125)/11.125733)*Intake.getInstance().io.getIntakePosition())
            ,0.0,//CHANGE THE THINGS UP AND DOWN FROM HERE (extended pos) to be the true full out dist, so set 0 properly and then set extended to like 15 and see what it gets to when enabled
            Units.inchesToMeters(((-2.125)/11.125733)*Intake.getInstance().io.getIntakePosition())
            ,new Rotation3d(0,0,0)));
        // intakeCameraPosition = new Transform3d(0.25,0,0.3125, new Rotation3d(Units.degreesToRadians(SmartDashboard.getNumber("roll", 100)), Units.degreesToRadians(SmartDashboard.getNumber("pitch", 100)),0));
    }//IM STUPID AND I LITERALLY HAD IT, IF THIS DOESNT WORK, GO BACK TO m/m*inchestoM(intakepos)
    //extended val #1:x=0.465, z=0.265 in m
    //retracted val #1:x=0.25, z=0.3125 in m
    //might want better measurements

    //9.25 diff
    // 1.375 diff

    public void updateYawAndX () {
        updateIntakeCameraPosition();
        yawRotation = intakeCameraPosition.getRotation().getZ();
        xTransform = Math.abs(intakeCameraPosition.getTranslation().getX());
    }


    public double getFuelDepthCameraRelative(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        if (corners.size() < 2) return -1;

        // Find the horizontal pixel width of the bounding box
        double minX = corners.stream().mapToDouble(c -> c.x).min().orElse(0);
        double maxX = corners.stream().mapToDouble(c -> c.x).max().orElse(0);
        double pixelWidth = maxX - minX;

        if (pixelWidth <= 0) return -1;

        // Use the pinhole camera model: Depth = (RealWidth * FocalLength) / PixelWidth
        // This is mathematically superior to manual IFOV/angle-based math
        return (FUEL_DIAMETER_METERS * FX) / pixelWidth;
    }
     
    // public double getFuelDepthCameraRelative(PhotonTrackedTarget target) {
    //     List<TargetCorner> corners = target.getDetectedCorners();
        
    //     if (corners.size() < 2) {
    //         // SmartDashboard.putNumber("algae depth", -1);
    //         return -1;
    //     }
    
    //     double minX = Double.MAX_VALUE;
    //     double maxX = -Double.MAX_VALUE;
    
    //     for (TargetCorner corner : corners) {
    //         double xAngle = corner.x * 1;//IFOV
    //         minX = Math.min(minX, xAngle);
    //         maxX = Math.max(maxX, xAngle);
    //     }
    
    //     double angularWidth = maxX - minX;//maybe need to make radians
    //     double depth = halfFuel / Math.tan(angularWidth / 2); 
    
    //     // SmartDashboard.putNumber("algae depth", depth);
    //     return depth;
    // }

    public Translation3d get3dPoseFieldRelative(PhotonTrackedTarget target) {
        double depth = getFuelDepthCameraRelative(target);
        if (depth < 0) return null;

        double yaw = Math.toRadians(target.getYaw());
        double pitch = Math.toRadians(target.getPitch());

        double targetX = depth * Math.cos(pitch) * Math.cos(yaw);
        double targetY = -depth * Math.cos(pitch) * Math.sin(yaw) + 0;
        double targetZ = depth * Math.sin(pitch);

        Translation3d targetInCameraSpace = new Translation3d(targetX, targetY, targetZ);

        Pose3d robotPose = SwerveSubsystem.getInstance().getRobotPose3d();
        
        Pose3d cameraPoseFieldRelative = robotPose.transformBy(intakeCameraPosition);
        
        return cameraPoseFieldRelative
                .transformBy(new Transform3d(targetInCameraSpace, new Rotation3d()))
                .getTranslation();
    }

    public double calculateRobotRelativeYaw(PhotonTrackedTarget target){
        double oppositeSide =  getFuelDepthCameraRelative(target)*Math.cos(yawRotation - target.getYaw()) -xTransform; //was yaw
        double adjacentSide = getFuelDepthCameraRelative(target)*Math.sin(yawRotation - target.getYaw()); // was yaw
        return (Math.PI/2)-Math.atan2(oppositeSide,adjacentSide);
    }

    public double[] getCenterOffsets(){
        
        if (hasTargets()){
            // return calculateRobotRelativeYaw(target);
            double[] yaws = new double[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                yaws[i] = targets.get(i).getYaw();
            }
            return yaws;
        }
        return null;
    }

    public double[] getOffsetPitches() {
        
        if (hasTargets()){
            // return calculateRobotRelativeYaw(target);
            double[] pitches = new double[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                pitches[i] = targets.get(i).getPitch() + 12.5;//why 12.5?
            }
            return pitches;
        }
        return null;
    }

    public int numTargets () {
        return targets.size();
    }

    public boolean hasTargets() {
        // boolean test;
        // PhotonPipelineResult tset;
        // List<PhotonPipelineResult> tetd;
        // List<PhotonTrackedTarget> t2;
        // PhotonTrackedTarget t3;
        // tetd = camera.getAllUnreadResults();
        // test = camera.getAllUnreadResults().isEmpty();

        // try {
        // tset = tetd.get(0);
        // t2 = tset.getTargets();
        // t3 = t2.get(0);
        // t3.getPitch();
        // } catch (Exception e) {}
        // tset = camera.getLatestResult();
        if (targets == null) {return false;}
        return !targets.isEmpty();
    }

    public void cycle () {
        updateYawAndX();
        addTargets();
        filterTargets();
        logPoses();
    }

    public void addTargets () {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        try {
            targets = result.get(0).getTargets();
        } catch (Exception e) {}

        // if (result == null) {
        //     targets.clear();;
        // }


        
        if (!hasTargets()) {
            poses = new Translation3d[0];
            return;
        }

        poses = new Translation3d[numTargets()];

        for (int i = 0; i < targets.size(); i++) {
            //replace w/ real pose
            //poses.add(targets.get(i).something);
            poses[i] = get3dPoseFieldRelative(targets.get(i));
        }
        // poses[0] = new Translation3d(1.5,1.5, 0.08);
    }

    public void filterTargets () {
        //make empty list of poses. 
        //for loop thru the poses. if this pose is within .5m, skip. else, add to list.
        //line of best fit?
    }

    public void setTargetDistances () {
        values.clear();
        for (int i = 0; i < targets.size(); i++) {
            //replace w/ real pose
            //poses.add(targets.get(i).something);
            values.put(poses[i], getFuelDepthCameraRelative(targets.get(i)));
        }

    }

    //do 5x5 around robot? 
    public int[] getRegion (Translation3d target) {
        return new int[]{(int) target.getX(), (int) target.getY()};
    }

    public int numFuelsInRegion (int[] region) {
        int numFuel = 0;
        if (poses.length == 0) {return 0;}
        for (int i = 0; i < poses.length; i++) {
            if ((int) poses[i].getX() == region[0] && (int) poses[i].getY() == region[1]) {
                numFuel++;
            }
        }
        return numFuel;
    }

    // public Translation3d bestPlaceToGo () {
    //     int searchRadius = 5;//radius outside of robot, 1 = 3 diameter, 2 = 5;
    //     // int[] robotRegion = getRegion(testRobotPos);
    //     int[] robotRegion = getRegion(new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), 0.08));
    //     int[][] top3Regions = new int[3][2];
    //     int[] bestRegion = new int[]{-1,-1};
    //     double maxRatio = 0;
    //     int first = 0;
    //     int second = 0;
    //     int third = 0;
    //     double firstDist = 0;
    //     double secondDist = 0;
    //     double thirdDist = 0;
    //     int currentFuelNum;
        
    //     for (int x = 0; x < searchRadius*2+1; x++) {
    //         for (int y = 0; y < searchRadius*2+1; y++) {
    //             currentFuelNum = numFuelsInRegion(new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y});
    //             if (currentFuelNum > first) {
    //                 third = second;
    //                 top3Regions[2] = top3Regions[1];
                    
    //                 second = first;
    //                 top3Regions[1] = top3Regions[0];
                    
    //                 first = currentFuelNum;
    //                 top3Regions[0] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
    //             } else if (currentFuelNum > second) {
    //                 third = second;
    //                 top3Regions[2] = top3Regions[1];
                    
    //                 second = currentFuelNum;
    //                 top3Regions[1] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
    //             } else if (currentFuelNum > third) {
    //                 third = currentFuelNum;
    //                 top3Regions[2] = new int[] {robotRegion[0]-searchRadius+x, robotRegion[1]-searchRadius+y};
    //             }
    //         }
    //     }//got top 3 regions

    //     if (first == 0) {
    //         DriverCommunications.hasAutoDriveTarget = false;
    //         SwerveSubsystem.getInstance().hasAutoDriveTarget = false;
    //         SwerveSubsystem.getInstance().bestPlaceToGo = null;
    //         return null;
    //     }

    //     //first neighbors
    //     first += numFuelsInRegion(new int[] {top3Regions[0][0], top3Regions[0][1] + 1});
    //     first += numFuelsInRegion(new int[] {top3Regions[0][0], top3Regions[0][1] - 1});
    //     first += numFuelsInRegion(new int[] {top3Regions[0][0] + 1, top3Regions[0][1]});
    //     first += numFuelsInRegion(new int[] {top3Regions[0][0] -1, top3Regions[0][1]});
    //     //second neighbors
    //     second += numFuelsInRegion(new int[] {top3Regions[1][0], top3Regions[1][1] + 1});
    //     second += numFuelsInRegion(new int[] {top3Regions[1][0], top3Regions[1][1] - 1});
    //     second += numFuelsInRegion(new int[] {top3Regions[1][0] + 1, top3Regions[1][1]});
    //     second += numFuelsInRegion(new int[] {top3Regions[1][0] -1, top3Regions[1][1]});
    //     //third neighbors
    //     third += numFuelsInRegion(new int[] {top3Regions[2][0], top3Regions[2][1] + 1});
    //     third += numFuelsInRegion(new int[] {top3Regions[2][0], top3Regions[2][1] - 1});
    //     third += numFuelsInRegion(new int[] {top3Regions[2][0] + 1, top3Regions[2][1]});
    //     third += numFuelsInRegion(new int[] {top3Regions[2][0] -1, top3Regions[2][1]});

    //     //y/x
    //     firstDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[0][0], robotRegion[1] - top3Regions[0][1]));
    //     secondDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[1][0], robotRegion[1] - top3Regions[1][1]));
    //     thirdDist = Math.max(1, Math.hypot(robotRegion[0] - top3Regions[2][0], robotRegion[1] - top3Regions[2][1]));

        

    //     maxRatio = Math.max((first * (1/firstDist)), Math.max((second * (1/secondDist)), (third * (1/thirdDist))));

    //     if ((first * (1/firstDist)) == maxRatio) {
    //             bestRegion = top3Regions[0];
    //     } else if ((second * (1/secondDist)) == maxRatio) {
    //         bestRegion = top3Regions[1];
    //     } else if ((third * (1/thirdDist)) == maxRatio) {
    //         bestRegion = top3Regions[2];
    //     } 

    //     return new Translation3d(bestRegion[0] + 0.5, bestRegion[1] + 0.5, 0.08);
    // }//returns nearest center of best region)



    public Translation3d bestPlaceToGo () {
        Translation3d closest = null;
        double dist = 0;
        for (Translation3d fuel : poses) {
            if (closest == null) {
                closest = fuel;
                dist = fuel.getDistance(new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), fuel.getZ()));
            } else {
                if(fuel.getDistance(new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), fuel.getZ())) < dist) {
                    closest = fuel;
                    dist = fuel.getDistance(new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), fuel.getZ()));
                }
            
            }
        }

        return closest;
    }//returns nearest center of best region)







    public void driveToPosition (Translation3d pos) {
        Pose2d robotPose = SwerveSubsystem.getInstance().getRobotPose();
        double globalAngle = Math.toDegrees(Math.atan2(pos.getY() - robotPose.getY(), pos.getX() - robotPose.getX()));
        double change = globalAngle - robotPose.getRotation().getDegrees();
        change = Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(change)), 
            Math.cos(Math.toRadians(change))
        ));
        
        double deltaX = pos.getX() - robotPose.getX();
        double deltaY = pos.getY() - robotPose.getY();

        double kPt = 0.5; //TODO: tune
        double speedX = deltaX * kPt;
        double speedY = deltaY * kPt;

        double kPr = 1; //TODO: tune
        double speedR = change * kPr;

        SwerveSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(speedX, speedY, speedR));
    }


    //how to get which one i should go to?
    //max fuel for least distance
    //consider future trips?
    /*
     
        know where each fuel is
        know dists
        num fuel in area + num in neighbors?
        ok
        choose top 3 areas (m) w/ most
        add neighbor vals
        go to #1
        go to closest fuel within those neighbors



     */

    public void logPoses () {


        // SmartDashboard.putP
        SmartDashboard.putNumber("timeTest", Timer.getMatchTime());

        //new arr [#targets]
        //for #targets, set i = target pose

        // arrayPublisher.set(poses);//REAL
        // poses = new Translation3d[] {
        //     new Translation3d(1,1,0.08),
        //     new Translation3d(1.1,1.1,0.08),
        //     new Translation3d(1.2,1,0.08),
        //     new Translation3d(1.3,1.1,0.08),
        //     new Translation3d(2,1,0.08),
        //     new Translation3d(2,1.2,0.08),
        //     new Translation3d(3,1,0.08),
        //     new Translation3d(3,1.1,0.08),
        //     new Translation3d(3,1.3,0.08),
        //     new Translation3d(4,1,0.08),
        //     new Translation3d(4,1,0.08),
        //     new Translation3d(0.9,1,0.08),
        // };
        fuelLog.set(poses);
        // best = bestPlaceToGo();
        // if (best != null)
        // bestPlaceToGo.set(new Pose3d(best.getX(), best.getY(), 0.08, new Rotation3d()));
        // fakeRobot3d.accept(testRobotPos);
        
    }

    public void logHeatmap () {
        //blank for now
    }

    public static Translation3d[] getCollectableFuel (Translation3d[] poses) {
        if (poses == null) {return new Translation3d[0];}
        List<Translation3d> posesList = new ArrayList<Translation3d>();
        for (Translation3d pose : poses) {
            if (pose.getZ() < .4) {
                posesList.add(pose);
            }
        }
        return posesList.toArray(new Translation3d[0]);
    }
    
    public static Translation3d[] sortPosesByDistance(Translation3d[] poses) {
    if (poses == null) {return new Translation3d[0];}
    Translation3d robotPos = new Translation3d(SwerveSubsystem.getInstance().getRobotPose().getX(), SwerveSubsystem.getInstance().getRobotPose().getY(), 0); // Grab current position once

    Arrays.sort(poses, (a, b) -> {
        // Calculate squared distances to skip heavy Math.sqrt() calculations
        double distSqA = a.getSquaredDistance(robotPos);
        double distSqB = b.getSquaredDistance(robotPos);
        
        return Double.compare(distSqA, distSqB);
    });

    return poses;
}
}
/*
 
foreach target
get pose of target, add to list of time + pose

foreach time (every sec, 50 cycle)
if curr - time > max delete,
set list to pose + value, if .contains pose, use max

log pose



get # in sqr m, then if its > min, give pts
pts:
# * (1- (time since seen/max))
if rn, 999



 */