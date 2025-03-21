
package frc.robot.vision;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LimelightHelpers;
import frc.robot.RobotContainer;

public class AprilTagVision {

    // PhotonVision Pose Estimators for each came
//
    public AprilTagVision() {
    }
    
    public Optional<LimelightHelpers.PoseEstimate> getEstimatedPose(String limelightName){ 
        LimelightHelpers.SetRobotOrientation(limelightName, RobotContainer.s_Swerve.getHeadingAngle().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null) {
            return Optional.empty();
        }
        return Optional.of(mt2);
    }
    public Optional<LimelightHelpers.PoseEstimate> getEstimatedPoseM1(String limelightName){ 
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null) {
            return Optional.empty();
        }
        return Optional.of(mt1);
    }




    public Pose2d averagePose(Pose2d[] poses) {

    
        double sumX = 0, sumY = 0, sumHeading = 0;
        
        for (Pose2d pose : poses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumHeading += pose.getRotation().getDegrees();
        }
        
        int count = poses.length;
        return new Pose2d(sumX / count, sumY / count, Rotation2d.fromDegrees(sumHeading / count));
    }



}