package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionIOSystem implements VisionIO {
    private final PhotonCamera[] cameras;
    // private final PhotonCamera intakeCamera;
    private final PhotonPoseEstimator[] photonEstimators;
    // public final PieceVision pieceAlignCamera = new PieceVision(Constants.pieceVisionCameraName, Constants.centerToCameraDefaultPosition);  //TODO change constatns

    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private double visonCycleTime;
    public static int climbAlignStage = 0;
    public static double timeAtLastMultiTagPose= -1000;

    public static PIDController xPidcontroller = new PIDController(2,0.2,0.01);
    public static PIDController yPidcontroller = new PIDController(2,0.2,0.01);
    public static PIDController angularPidcontroller = new PIDController(3, 0.5, 0);

    // // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;

    public VisionIOSystem(EstimateConsumer estConsumer) {
        // intakeCamera = new PhotonCamera(Constants.intakeCameraName);
        // intakeCamera.setDriverMode(true);
        // CameraServer.startAutomaticCapture(Constants.intakeCameraName, "/dev/video0");
        cameras = new PhotonCamera[] {
            new PhotonCamera(Constants.frontRightCameraName), 
            new PhotonCamera(Constants.frontleftCameraName),
            new PhotonCamera(Constants.leftCameraName), 
            new PhotonCamera(Constants.rightCameraName), 
            new PhotonCamera(Constants.backCameraName)
        };

        photonEstimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFrontRight), 
            new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamFrontLeft),
            new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamLeft),
            new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamRight),  
            new PhotonPoseEstimator(Constants.kTagLayout, Constants.robotToCamBack)
        };
        this.estConsumer = estConsumer; // Lamba that will accept a pose estimate and pass it to your desired {@link
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.intakeCameraConected = intakeCamera.isConnected();
        inputs.frontRightCameraConected = cameras[0].isConnected();
        inputs.frontLeftCameraConected = cameras[1].isConnected();
        inputs.leftCameraConected = cameras[2].isConnected();
        inputs.rightCameraConected = cameras[3].isConnected();
        inputs.backCameraConected = cameras[4].isConnected();
        inputs.visonCycleTime = visonCycleTime;
        // inputs.pieceCameraHasTargets = pieceAlignCamera.hasTargets();
        // inputs.pieceCameraNumTargets = pieceAlignCamera.numTargets();
    }

    public boolean CameraConnect(int camera){
        if(cameras[camera].isConnected()){
            return true;
        }else{
            return false;
        }
    }

    public boolean CamerasConnected(){
        for(int i =0;i<cameras.length;i++)
            if(!cameras[i].isConnected())
                return false;
        return true;
    }
    /*
     * the periodic seaches all cameras for a hub mutitag pose and if found uses only that pose however if it is not found it adds togeter all the other tag poses to get a sutable estimate.
    */
    @Override
    public void periodic() {

        double startTime = Timer.getFPGATimestamp();

        boolean doSingletag = true;

        FilteredCameraResults[] FilteredResults = new FilteredCameraResults[cameras.length];
        
        for(int i = 0;i < FilteredResults.length;i++)
            FilteredResults[i]=filterPhotonResults(cameras[i], photonEstimators[i], doSingletag);

        int qualityOfBestCamera;
        boolean foundMultiTagHubResult = false;
        boolean foundMultiTagResult = false;
        boolean foundsingleTagResult = false;
        for(int i =0; i<FilteredResults.length;i++)
            if (FilteredResults[i].multiTagHubResults.size()>0)
                foundMultiTagHubResult=true;
        if(!foundMultiTagHubResult)
            for(int i =0; i<FilteredResults.length;i++)
                if (FilteredResults[i].multiTagResults.size()>0)
                    foundMultiTagResult=true;
        if(!foundMultiTagResult)
            for(int i =0; i<FilteredResults.length;i++)
                if (FilteredResults[i].singleTagResults.size()>0)
                    foundsingleTagResult=true;



        if (foundMultiTagHubResult){
            for(int i = 0;i < FilteredResults.length;i++)
                addVisionEstemation(FilteredResults[i].multiTagHubResults, FilteredResults[i].multiTagHubTargets, true, photonEstimators[i]);
            qualityOfBestCamera = 3;
        }
        else if (foundMultiTagResult)
        {
            for(int i = 0;i < FilteredResults.length;i++)
                addVisionEstemation(FilteredResults[i].multiTagResults, FilteredResults[i].multiTagTargets, false, photonEstimators[i]);
            qualityOfBestCamera = 2;
        }
        else if (foundsingleTagResult)
        {
            for(int i = 0;i < FilteredResults.length;i++)
                addVisionEstemation(FilteredResults[i].singleTagResults, FilteredResults[i].singleTagTargets, false, photonEstimators[i]);
            qualityOfBestCamera = 1;
        }
        else
            qualityOfBestCamera = 0;
        
        Logger.recordOutput("camera Quality", qualityOfBestCamera);
        this.visonCycleTime = Timer.getFPGATimestamp() - startTime;
    }

    //loops through all camera results for each caemra and checkes sorts them between hub multitag, multitag, and singletag
    private FilteredCameraResults filterPhotonResults(PhotonCamera camera, PhotonPoseEstimator photonEstimator, boolean doSingletag){
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        ArrayList<Optional<List<PhotonTrackedTarget>>> hubMultiTagTargets = new ArrayList<Optional<List<PhotonTrackedTarget>>>();
        ArrayList<Optional<EstimatedRobotPose>> hubMultiTagResults = new ArrayList<Optional<EstimatedRobotPose>>();

        ArrayList<Optional<List<PhotonTrackedTarget>>> multiTagTargets = new ArrayList<Optional<List<PhotonTrackedTarget>>>();
        ArrayList<Optional<EstimatedRobotPose>> multiTagResults = new ArrayList<Optional<EstimatedRobotPose>>();

        ArrayList<Optional<List<PhotonTrackedTarget>>> singleTagTargets = new ArrayList<Optional<List<PhotonTrackedTarget>>>();
        ArrayList<Optional<EstimatedRobotPose>> singleTagResults = new ArrayList<Optional<EstimatedRobotPose>>();

        for (var result : camera.getAllUnreadResults()) {
            if(result.hasTargets()) {
                result.targets = removeAmbigousTargets(result.targets);

                visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty() && timeAtLastMultiTagPose < Timer.getFPGATimestamp() - 0.1) {
                    if (doSingletag){
                        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                        if(!visionEst.isEmpty()) {
                            singleTagResults.add(visionEst);
                            singleTagTargets.add(Optional.of(result.getTargets()));
                        }
                    }
                } else {
                        timeAtLastMultiTagPose = result.getTimestampSeconds();
                    int numberOfHubTags = 0;
                    for(int i = 0; i < result.getTargets().size(); i++)
                        if(addToHubTagNumber(result, i)){
                            numberOfHubTags+=1;
                        }
                    if (numberOfHubTags>=2){
                        doSingletag=false;
                        hubMultiTagResults.add(visionEst);
                        hubMultiTagTargets.add(Optional.of(result.getTargets()));
                        break;
                    }
                    else{
                        multiTagResults.add(visionEst);
                        multiTagTargets.add(Optional.of(result.getTargets()));
                    }
                    
                }
            }
        }

        return new FilteredCameraResults(
            hubMultiTagResults,
            multiTagResults,
            singleTagResults,
            hubMultiTagTargets,
            multiTagTargets,
            singleTagTargets
        );
    }

    private boolean addToHubTagNumber(PhotonPipelineResult result, int i){
        if(
                result.getTargets().get(i).getFiducialId() == 18
                || result.getTargets().get(i).getFiducialId() == 19
                || result.getTargets().get(i).getFiducialId() == 20
                || result.getTargets().get(i).getFiducialId() == 21
                || result.getTargets().get(i).getFiducialId() == 24
                || result.getTargets().get(i).getFiducialId() == 27
                || result.getTargets().get(i).getFiducialId() == 26
                || result.getTargets().get(i).getFiducialId() == 25
                || result.getTargets().get(i).getFiducialId() == 8
                || result.getTargets().get(i).getFiducialId() == 9
                || result.getTargets().get(i).getFiducialId() == 10
                || result.getTargets().get(i).getFiducialId() == 11
                || result.getTargets().get(i).getFiducialId() == 2
                || result.getTargets().get(i).getFiducialId() == 5
                || result.getTargets().get(i).getFiducialId() == 4
                || result.getTargets().get(i).getFiducialId() == 3
            ){
            return true;
        }
        return false;
    }

    private void addVisionEstemation(ArrayList<Optional<EstimatedRobotPose>> filteredResults, ArrayList<Optional<List<PhotonTrackedTarget>>> filteredTargets, boolean ishub, PhotonPoseEstimator photonEstimator){
        for (int i =0;i < filteredResults.size(); i++){
            updateEstimationStdDevs(filteredResults.get(i), filteredTargets.get(i).get(), ishub, photonEstimator);

            filteredResults.get(i).ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs();
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
        }
    }
    
    private List<PhotonTrackedTarget> removeAmbigousTargets(List<PhotonTrackedTarget> allTargets){
        List<PhotonTrackedTarget> optimizedTargets = new ArrayList<PhotonTrackedTarget>();
        for(var target : allTargets){
            if(target.poseAmbiguity < 0.2){
                optimizedTargets.add(target);
            }
        }
        return optimizedTargets;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, boolean ishub ,PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = ishub ? Constants.kMultiTagHubStdDevs : Constants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    estStdDevs = VecBuilder.fill(0.01,0.01,0.01);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public static double getDistBetweenPoints(Translation2d pose1,Translation2d pose2){
        return Math.sqrt((
                pose1.getY() - pose2.getY())
                * (pose1.getY() - pose2.getY()
            ) + (
                pose1.getX() - pose2.getX())
                * (pose1.getX() - pose2.getX()
            ));
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    // public PieceVision getPieceCamera () {
        // return pieceAlignCamera;
    // }
}