package frc.robot.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTag {

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

    public AprilTag() {
        // Initialize cameras
        aprilTagsCamera = new PhotonCamera("AprilTags_side_Camera");
        secondCamera = new PhotonCamera("Second-Camera");

        // Load the field layout for AprilTag positions (e.g., for the 2024 game field)
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // Define the camera positions on the robot
        robotToCam1 = new Transform3d(
            new Translation3d(0.4, -0.265, 0.17), 
            new Rotation3d(Math.toRadians(180), Math.toRadians(24), 0)
        );
        robotToCam2 = new Transform3d(
            new Translation3d(-0.5, 0.0, 0.5), 
            new Rotation3d(0, 0, 0)
        );

        // Initialize pose estimators with strategy and camera-to-robot transforms
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, aprilTagsCamera, robotToCam1);
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, secondCamera, robotToCam2);
    }

    // Method to get the estimated pose, combining both camera inputs if available
    public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        // Set reference pose for both estimators
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);

        // Get the pose estimates from both cameras
        Optional<EstimatedRobotPose> pose1 = poseEstimator1.update();
        Optional<EstimatedRobotPose> pose2 = poseEstimator2.update();

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
