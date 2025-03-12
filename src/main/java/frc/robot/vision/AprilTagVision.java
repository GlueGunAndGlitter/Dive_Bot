
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
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AprilTagVision {

    // PhotonVision Pose Estimators for each came
//
    private AprilTagVision visionEstimator;

    public AprilTagVision() {

    }
    public LimelightHelpers.PoseEstimate getEstimatedPose(){ 
      
        LimelightHelpers.SetRobotOrientation("limelight", RobotContainer.s_Swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        return mt2;
    }

    



}