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

public class AprilTagVision {

    // Cameras for AprilTag detection
    private final PhotonCamera DownLeftCamera;
    private final PhotonCamera DownRightCamera;
    private final PhotonCamera upCamera;   // Third Camera

    // Field layout with AprilTag locations
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Camera-to-robot transforms
    private final Transform3d robotToCam1;
    private final Transform3d robotToCam2;
    private final Transform3d robotToCam3;  // Third Camera Transform

    // PhotonVision Pose Estimators for each camera
    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;
    private final PhotonPoseEstimator poseEstimator3; // Third Camera Estimator

    public AprilTagVision() {
        // Initialize cameras
        DownLeftCamera = new PhotonCamera("DownLeftCamera");
        DownRightCamera = new PhotonCamera("DownRightCamera");
        upCamera = new PhotonCamera("upCamera"); // Initialize Third Camera

        // Load the field layout for AprilTag positions
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // Define the camera positions on the robot
        robotToCam1 = new Transform3d(
            new Translation3d(-0.155, -0.215, 0.30),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))
        );
        robotToCam2 = new Transform3d(
            new Translation3d(0.045, 0.27, 0.285),
            new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(190))
        );
        robotToCam3 = new Transform3d(   // Third Camera Transform
            new Translation3d(-0.04, 0.035, 0.93),
            new Rotation3d(Math.toRadians(0), Math.toRadians(40), Math.toRadians(0))
        );

        // Initialize pose estimators with strategy and camera-to-robot transforms
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam1);
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam2);
        poseEstimator3 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam3);
    }

    // Method to get the estimated pose, combining all camera inputs if available
   // Method to get the estimated pose, combining all camera inputs if available
public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

    PhotonPipelineResult camera1Result = DownLeftCamera.getLatestResult();
    PhotonPipelineResult camera2Result = DownRightCamera.getLatestResult();
    PhotonPipelineResult camera3Result = upCamera.getLatestResult(); // Third Camera Result

    // Set reference pose for all estimators
    poseEstimator1.setReferencePose(prevEstimatedRobotPose);
    poseEstimator2.setReferencePose(prevEstimatedRobotPose);
    poseEstimator3.setReferencePose(prevEstimatedRobotPose);

    // Get the pose estimates from all cameras
    Optional<EstimatedRobotPose> pose1 = poseEstimator1.update(camera1Result);
    Optional<EstimatedRobotPose> pose2 = poseEstimator2.update(camera2Result);
    Optional<EstimatedRobotPose> pose3 = poseEstimator3.update(camera3Result);

    // Combine or choose the best pose estimate based on availability
    if (pose1.isPresent() && pose2.isPresent() && pose3.isPresent()) {
        // Average all three poses
        Pose2d averagePose = new Pose2d(
            pose1.get().estimatedPose.toPose2d().getTranslation()
                .plus(pose2.get().estimatedPose.toPose2d().getTranslation())
                .plus(pose3.get().estimatedPose.toPose2d().getTranslation()).div(3),
            pose1.get().estimatedPose.toPose2d().getRotation()
        );
        return Optional.of(averagePose);
    } else if (pose1.isPresent() && pose2.isPresent()) {
        // Average pose1 and pose2
        Pose2d averagePose = new Pose2d(
            pose1.get().estimatedPose.toPose2d().getTranslation()
                .plus(pose2.get().estimatedPose.toPose2d().getTranslation()).div(2),
            pose1.get().estimatedPose.toPose2d().getRotation()
        );
        return Optional.of(averagePose);
    } else if (pose1.isPresent() && pose3.isPresent()) {
        // Average pose1 and pose3
        Pose2d averagePose = new Pose2d(
            pose1.get().estimatedPose.toPose2d().getTranslation()
                .plus(pose3.get().estimatedPose.toPose2d().getTranslation()).div(2),
            pose1.get().estimatedPose.toPose2d().getRotation()
        );
        return Optional.of(averagePose);
    } else if (pose2.isPresent() && pose3.isPresent()) {
        // Average pose2 and pose3
        Pose2d averagePose = new Pose2d(
            pose2.get().estimatedPose.toPose2d().getTranslation()
                .plus(pose3.get().estimatedPose.toPose2d().getTranslation()).div(2),
            pose2.get().estimatedPose.toPose2d().getRotation()
        );
        return Optional.of(averagePose);
    } else if (pose1.isPresent()) {
        return Optional.of(pose1.get().estimatedPose.toPose2d());
    } else if (pose2.isPresent()) {
        return Optional.of(pose2.get().estimatedPose.toPose2d());
    } else if (pose3.isPresent()) {
        return Optional.of(pose3.get().estimatedPose.toPose2d());
    }

    return Optional.empty();
}
}
