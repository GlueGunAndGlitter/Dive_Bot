package frc.robot.commands.autoTelopCommands;

import frc.robot.ApriltagConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class ReefAsisst extends Command {    
    private Swerve s_Swerve;    
    private boolean isRedAlliance;
    private PIDController rotionPID = new PIDController(0,0, 0, 0);
    private PIDController y_PID = new PIDController(0, 0, 0);
    private PIDController x_PID = new PIDController(0,0,0);

    public ReefAsisst(Swerve s_Swerve) {
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
    }

    @Override
    public void execute() {
        /* Drive */
        Pose2d robotPose = s_Swerve.getPose();
        Pose2d target = getTarget();

        double error = target.getRotation().getDegrees() - Math.IEEEremainder(s_Swerve.getHeading().getDegrees(), 360);
        error = Math.IEEEremainder(error, 360); // Normalize error to -180 to 180
        
        double rotationOutput = rotionPID.calculate(error,0);
        double y_output = y_PID.calculate(robotPose.getY(),target.getY());
        double x_output = x_PID.calculate(robotPose.getX(),target.getX());


        s_Swerve.drive(
          new Translation2d(x_output, y_output).times(Constants.Swerve.maxSpeed), 
          rotationOutput * Constants.Swerve.maxAngularVelocity, 
          true, 
          true
      );

    }

    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }


  private int getClosestAprilTag() {
      double[] distances;
      int currentClosestIndex = 0;
  
      if (isRedAlliance) {
          distances = new double[]{
              distanceFromRobot(ApriltagConstants.redReef.TAG_6.x, ApriltagConstants.redReef.TAG_6.y),
              distanceFromRobot(ApriltagConstants.redReef.TAG_7.x, ApriltagConstants.redReef.TAG_7.y),
              distanceFromRobot(ApriltagConstants.redReef.TAG_8.x, ApriltagConstants.redReef.TAG_8.y),
              distanceFromRobot(ApriltagConstants.redReef.TAG_9.x, ApriltagConstants.redReef.TAG_9.y),
              distanceFromRobot(ApriltagConstants.redReef.TAG_10.x, ApriltagConstants.redReef.TAG_10.y),
              distanceFromRobot(ApriltagConstants.redReef.TAG_11.x, ApriltagConstants.redReef.TAG_11.y)
          };
  
          for (int i = 1; i < distances.length; i++) {
              if (distances[i] < distances[currentClosestIndex]) {
                  currentClosestIndex = i;
              }
          }
          return currentClosestIndex + 6; // Tag IDs are from 6 to 11
      } else {
          distances = new double[]{
              distanceFromRobot(ApriltagConstants.blueReef.TAG_17.x, ApriltagConstants.blueReef.TAG_17.y),
              distanceFromRobot(ApriltagConstants.blueReef.TAG_18.x, ApriltagConstants.blueReef.TAG_18.y),
              distanceFromRobot(ApriltagConstants.blueReef.TAG_19.x, ApriltagConstants.blueReef.TAG_19.y),
              distanceFromRobot(ApriltagConstants.blueReef.TAG_20.x, ApriltagConstants.blueReef.TAG_20.y),
              distanceFromRobot(ApriltagConstants.blueReef.TAG_21.x, ApriltagConstants.blueReef.TAG_21.y),
              distanceFromRobot(ApriltagConstants.blueReef.TAG_22.x, ApriltagConstants.blueReef.TAG_22.y)
          };
  
          for (int i = 1; i < distances.length; i++) {
              if (distances[i] < distances[currentClosestIndex]) {
                  currentClosestIndex = i;
              }
          }
          return currentClosestIndex + 17; // Tag IDs are from 17 to 22
      }
  }
  
  private double distanceFromRobot(double x, double y) {
      Pose2d robotPose = s_Swerve.getPose();
      return pythagoras(robotPose.getX() - x, robotPose.getY() - y);
  }
  
  private double pythagoras(double x, double y) {
      return Math.sqrt(x * x + y * y);
  }
  
  private Pose2d getTarget(){
    int id = getClosestAprilTag();
    if (Robot.isLeft) {
      return Constants.ReefAsisstConstnt.LEFT_TARGETS.get(id);
    }else{
      return Constants.ReefAsisstConstnt.RIGHT_TARGETS.get(id);
    }
    }
  
  
}