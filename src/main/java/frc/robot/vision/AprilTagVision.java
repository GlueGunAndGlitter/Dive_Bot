
package frc.robot.vision;

import java.util.Optional;
import java.util.PrimitiveIterator;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Robot;

public class AprilTagVision {

    private final PhotonCamera DownLeftCamera;
    private final PhotonCamera DownRightCamera;

    // Field layout with AprilTag locations
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Camera-to-robot transforms
    private final Transform3d robotToCam1;
    private final Transform3d robotToCam2;

    // PhotonVision Pose Estimators for each camera
    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;

    public AprilTagVision() {
        DownLeftCamera = new PhotonCamera("DownLeftCamera");
        DownRightCamera = new PhotonCamera("DownRightCamera");

        // Load the field layout for AprilTag positions
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // Define the camera positions on the robot
        robotToCam1 = new Transform3d(
            new Translation3d(-0.155, -0.215, 0.30),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-5), Math.toRadians(0))
        );
        robotToCam2 = new Transform3d(
            new Translation3d(0.045, 0.27, 0.285),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-10))
        );

        // Initialize pose estimators with strategy and camera-to-robot transforms
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1);
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2);

        poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public boolean isRightCameraHasTarget(){
        return DownRightCamera.getLatestResult().hasTargets();
    }

    public boolean isLeftCameraHasTarget(){
        return DownLeftCamera.getLatestResult().hasTargets();
    }
    public double leftGetY(int Id){
        var latestResult = DownRightCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 0; i<latestResult.getTargets().size(); i++){
                    if (latestResult.getTargets().get(i).getFiducialId() == Id) {
                        return latestResult.getTargets().get(i).getBestCameraToTarget().getY();
                    }
                   }
                   return 0;
            }
        return 0;

    }

    public double rightGetY(int Id){
        int index = 0;
        var latestResult = DownLeftCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 0; i<latestResult.getTargets().size(); i++){
                    if (latestResult.getTargets().get(i).getFiducialId() == Id) {
                        return latestResult.getTargets().get(i).getBestCameraToTarget().getY();
                    }
                   }
                   return 0;
            }
        return 0;
    }


    public double leftGetX(){
        var latestResult = DownRightCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            return latestResult.getBestTarget().getBestCameraToTarget().getX();
            }
        return 10;
    }
    public double rightGetX(){
        var latestResult = DownLeftCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            return latestResult.getBestTarget().getBestCameraToTarget().getX();
            }
        return 10;
    }

    public double leftGetXForID(int Id){
        int index = 0;
        var latestResult = DownRightCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 0; i<latestResult.getTargets().size(); i++){
                    if (latestResult.getTargets().get(i).getFiducialId() == Id) {
                        return latestResult.getTargets().get(i).getBestCameraToTarget().getX();
                    }
                   }
                   return 0;
            }
        return 0;
    }
    public double rightGetXForID(int Id){
        int index = 0;
        var latestResult = DownLeftCamera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 0; i<latestResult.getTargets().size(); i++){
                    if (latestResult.getTargets().get(i).getFiducialId() == Id) {
                        return latestResult.getTargets().get(i).getBestCameraToTarget().getX();
                    }
                   }
                   return 0;
            }
        return 0;
    }


    public boolean hasIDAt(int Id, boolean isleft){
        PhotonPipelineResult result;
        if (!isleft) {
            result = DownLeftCamera.getLatestResult();
            for (int i = 0; i < result.getTargets().size(); i++) {
                if (result.getTargets().get(i).getFiducialId() == Id) {
                    return true;
                }
            }
            return false;

        }else{
            
            result = DownRightCamera.getLatestResult();
            for (int i = 0; i < result.getTargets().size(); i++) {
                if (result.getTargets().get(i).getFiducialId() == Id) {
                    return true;
                }
            }
            return false;

        }
    }


    public boolean hasID(int Id){
        PhotonPipelineResult result;
        if (Robot.isRight) {
            result = DownLeftCamera.getLatestResult();
            for (int i = 0; i < result.getTargets().size(); i++) {
                if (result.getTargets().get(i).getFiducialId() == Id) {
                    return true;
                }
            }
            return false;

        }else{
            
            result = DownRightCamera.getLatestResult();
            for (int i = 0; i < result.getTargets().size(); i++) {
                if (result.getTargets().get(i).getFiducialId() == Id) {
                    return true;
                }
            }
            return false;

        }
    }


    public boolean hasTarget(){
        var firstCameraResult = DownLeftCamera.getLatestResult();
        var DownRightCameraResult = DownRightCamera.getLatestResult();
        return firstCameraResult.hasTargets() || DownRightCameraResult.hasTargets();

    }



    // Method to get the estimated pose, combining both camera inputs if available
    public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        PhotonPipelineResult camera1Result = DownLeftCamera.getLatestResult();
        PhotonPipelineResult camera2Result = DownRightCamera.getLatestResult();
    
        // Set reference pose for all estimators
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);
    
        // Get the pose estimates from all cameras
        Optional<EstimatedRobotPose> pose1 = poseEstimator1.update(camera1Result);
        Optional<EstimatedRobotPose> pose2 = poseEstimator2.update(camera2Result);
    
        // Combine or choose the best pose estimate based on availability


        if (pose1.isPresent() && pose2.isPresent()) {
            // Average pose1 and pose2
            Pose2d averagePose = new Pose2d(
                pose1.get().estimatedPose.toPose2d().getTranslation()
                    .plus(pose2.get().estimatedPose.toPose2d().getTranslation()).div(2),
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