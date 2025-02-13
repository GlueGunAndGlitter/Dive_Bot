package frc.robot.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class AprilTagVision {

    // Cameras for AprilTag detection
    private final PhotonCamera aprilTagsCamera;
    private final PhotonCamera secondCamera;

    // Field layout with AprilTag locations
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Camera-to-robot transforms
    private final Transform3d robotToCam1;
    private final Transform3d robotToCam2;

    // PhotonVision Pose Estimators for each camera
    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;

    public AprilTagVision() {
        // Initialize cameras
        aprilTagsCamera = new PhotonCamera("AprilTags_Camera_1");
        secondCamera = new PhotonCamera("Second-Camera");

        // Load the field layout for AprilTag positions (e.g., for the 2024 game field)
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // Define the camera positions on the robot
        robotToCam1 = new Transform3d(
            Constants.AprilTagConstants.cam1Positoin,
            new Rotation3d(Math.toRadians(180), Math.toRadians(24), 0)
        );
        robotToCam2 = new Transform3d(
            Constants.AprilTagConstants.cam2Positoin,
            new Rotation3d(0, 0, 0)
        );



        // Initialize pose estimators with strategy and camera-to-robot transforms
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam1);
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam2);
    }


    public double leftGetY(){
        int index = 0;
        var latestResult = secondCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 1; i<latestResult.getTargets().size(); i++){
                if((Math.pow(latestResult.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(latestResult.getTargets().get(index).getBestCameraToTarget().getX(), 2)) >
                    (Math.pow(latestResult.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(latestResult.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = latestResult.getTargets().get(index);
        return best.getBestCameraToTarget().getY();
        }
        return 0;

    }

    public double rightGetY(){
        int index = 0;
        var letest = aprilTagsCamera.getLatestResult();
        if (letest.hasTargets()) {
            for (int i = 1; i<letest.getTargets().size(); i++){
                if((Math.pow(letest.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(letest.getTargets().get(index).getBestCameraToTarget().getX(), 2)) > 
                    (Math.pow(letest.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(letest.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = letest.getTargets().get(index);
        return best.getBestCameraToTarget().getY();
        }
        return 0;
    }

    public double leftGetX(){
        int index = 0;
        var secondCameraResult = secondCamera.getLatestResult();
        if (secondCameraResult.hasTargets()) {
            for (int i = 1; i< secondCameraResult.getTargets().size(); i++){
                if((Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getX(), 2)) > 
                    (Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = secondCameraResult.getTargets().get(index);
        return best.getBestCameraToTarget().getX();
        }
        return 0;
    }
    public double rightGetX(){
        int index = 0;
        var secondCameraResult = aprilTagsCamera.getLatestResult();
        if (secondCameraResult.hasTargets()) {
            for (int i = 1; i<secondCameraResult.getTargets().size(); i++){
                if((Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getX(), 2)) >
                    (Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = secondCameraResult.getTargets().get(index);
        return best.getBestCameraToTarget().getX();
        }
        return 0;
    }

    public int rightGetId(){
        int index = 0;
        var secondCameraResult = aprilTagsCamera.getLatestResult();
        if (secondCameraResult.hasTargets()) {
            for (int i = 1; i< secondCameraResult.getTargets().size(); i++){
                if((Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getX(), 2)) >
                    (Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = secondCameraResult.getTargets().get(index);
        return best.getFiducialId();
        }
        return 0;
    }
    public int leftGetId(){
        int index = 0;
        var secondCameraResult = secondCamera.getLatestResult();
        if (secondCameraResult.hasTargets()) {
            for (int i = 1; i<secondCameraResult.getTargets().size(); i++){
                if((Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(index).getBestCameraToTarget().getX(), 2)) > 
                    (Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getY(),2) +
                    Math.pow(secondCameraResult.getTargets().get(i).getBestCameraToTarget().getX(), 2))){
                    index = i;
                   }
            }
        var best = secondCameraResult.getTargets().get(index);
        return best.getFiducialId();
        }
        return 0;
    }

    public boolean hasTarget(){
        var firstCameraResult = aprilTagsCamera.getLatestResult();
        var secondCameraResult = secondCamera.getLatestResult();
        return firstCameraResult.hasTargets() || secondCameraResult.hasTargets();

    }



    // Method to get the estimated pose, combining both camera inputs if available
    public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        PhotonPipelineResult camera1Resolt = aprilTagsCamera.getLatestResult();
        PhotonPipelineResult camera2Resolt = secondCamera.getLatestResult();


        // Set reference pose for both estimators
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);

        // Get the pose estimates from both cameras
        Optional<EstimatedRobotPose> pose1 = poseEstimator1.update(camera1Resolt);
        Optional<EstimatedRobotPose> pose2 = poseEstimator2.update(camera2Resolt);

        // Combine or choose the best pose estimate based on availability
        if (pose1.isPresent() && pose2.isPresent()) {
            // Average the two poses (or you could use a weighted average based on confidence)
            Pose2d averagePose = new Pose2d(
                pose1.get().estimatedPose.toPose2d().getTranslation().plus(pose2.get().estimatedPose.toPose2d().getTranslation()).div(2),
                pose1.get().estimatedPose.toPose2d().getRotation()
            );
            return Optional.of(averagePose);
        } else if (pose1.isPresent()) {
            return Optional.of(pose1.get().estimatedPose.toPose2d());
        } else if (pose2.isPresent()) {
            return Optional.of(pose2.get().estimatedPose.toPose2d());
        }

        return Optional.empty();
    }
}
