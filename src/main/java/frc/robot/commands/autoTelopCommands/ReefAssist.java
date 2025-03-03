package frc.robot.commands.autoTelopCommands;

import frc.robot.ApriltagConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class ReefAssist extends Command {    
    private final Swerve s_Swerve;    
    private boolean isRedAlliance;
    private Pose2d target;

    private final PIDController rotationPID = new PIDController(
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kP,
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kI,
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kD
    );

    private final PIDController yPID = new PIDController(
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kP,
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kI,
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kD
    );

    private final PIDController xPID = new PIDController(
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kP,
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kI,
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kD
    );

    public ReefAssist(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() == Alliance.Red){
          isRedAlliance = true;
        }else{
          isRedAlliance = false;
        }
        target = new Pose2d(new Translation2d(13.87,5.17),Rotation2d.fromDegrees(60));
  
      }

    @Override
    public void execute() {
        Pose2d robotPose = s_Swerve.getPose();

        double error = target.getRotation().getDegrees() - Math.IEEEremainder(s_Swerve.getHeading().getDegrees(), 360);
        error = Math.IEEEremainder(error, 360); // Normalize error to -180 to 180
        
        double rotationOutput = rotationPID.calculate(error, 0);
        double yOutput = yPID.calculate(robotPose.getY(), target.getY());
        double xOutput = xPID.calculate(robotPose.getX(), target.getX());

        s_Swerve.drive(
            new Translation2d(xOutput, yOutput).times(Constants.Swerve.maxSpeed), 
            rotationOutput * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return atTolerance();
    }

    private boolean atTolerance() {
        Pose2d robotPose = s_Swerve.getPose();
        Transform2d error = robotPose.minus(target);

        return Math.abs(error.getX()) < Constants.ReefAssistConstants.X_TOLERANCE &&
               Math.abs(error.getY()) < Constants.ReefAssistConstants.Y_TOLERANCE &&
               Math.abs(error.getRotation().getDegrees()) < Constants.ReefAssistConstants.ROTATION_TOLERANCE;
    }

    private int getClosestAprilTag() {
        double[] distances;
        int closestIndex = 0;

        if (isRedAlliance) {
            distances = new double[]{
                distanceFromRobot(ApriltagConstants.redReef.TAG_6.x, ApriltagConstants.redReef.TAG_6.y),
                distanceFromRobot(ApriltagConstants.redReef.TAG_7.x, ApriltagConstants.redReef.TAG_7.y),
                distanceFromRobot(ApriltagConstants.redReef.TAG_8.x, ApriltagConstants.redReef.TAG_8.y),
                distanceFromRobot(ApriltagConstants.redReef.TAG_9.x, ApriltagConstants.redReef.TAG_9.y),
                distanceFromRobot(ApriltagConstants.redReef.TAG_10.x, ApriltagConstants.redReef.TAG_10.y),
                distanceFromRobot(ApriltagConstants.redReef.TAG_11.x, ApriltagConstants.redReef.TAG_11.y)
            };
        } else {
            distances = new double[]{
                distanceFromRobot(ApriltagConstants.blueReef.TAG_17.x, ApriltagConstants.blueReef.TAG_17.y),
                distanceFromRobot(ApriltagConstants.blueReef.TAG_18.x, ApriltagConstants.blueReef.TAG_18.y),
                distanceFromRobot(ApriltagConstants.blueReef.TAG_19.x, ApriltagConstants.blueReef.TAG_19.y),
                distanceFromRobot(ApriltagConstants.blueReef.TAG_20.x, ApriltagConstants.blueReef.TAG_20.y),
                distanceFromRobot(ApriltagConstants.blueReef.TAG_21.x, ApriltagConstants.blueReef.TAG_21.y),
                distanceFromRobot(ApriltagConstants.blueReef.TAG_22.x, ApriltagConstants.blueReef.TAG_22.y)
            };
        }

        for (int i = 1; i < distances.length; i++) {
            if (distances[i] < distances[closestIndex]) {
                closestIndex = i;
            }
        }

        return isRedAlliance ? closestIndex + 6 : closestIndex + 17; 
    }

    private double distanceFromRobot(double x, double y) {
        Pose2d robotPose = s_Swerve.getPose();
        return pythagoras(robotPose.getX() - x, robotPose.getY() - y);
    }

    private double pythagoras(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    private Pose2d getTarget() {
        int id = getClosestAprilTag();
        return Robot.isLeft ? 
            Constants.ReefAssistConstants.LEFT_TARGETS.get(id) : 
            Constants.ReefAssistConstants.RIGHT_TARGETS.get(id);
    }
}
