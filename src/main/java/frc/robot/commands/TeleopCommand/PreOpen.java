// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommand;

import java.util.logging.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ApriltagConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PreOpen extends Command {
  /** Creates a new ChoosableLevelCommand. */
  
  ArmAngleChange armAngleChange;
  Elevator elevator;
  boolean L4Runing = false;
  boolean isRedAlliance;
  Pose2d target;
  

  public  PreOpen(ArmAngleChange armAngleChange, Elevator elevator) {
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(this.elevator);
    addRequirements(this.armAngleChange);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    L4Runing = false;
       if (DriverStation.getAlliance().get() == Alliance.Red){
          isRedAlliance = true;
        }else{
          isRedAlliance = false;
        }
        target = getTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  



    switch (Robot.level) {
      case 1:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        elevator.setL1Position();
        armAngleChange.setL1Position();
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }

      break;
      case 2:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        elevator.setL2Position();
        armAngleChange.setL2Position();
        }else{
          elevator.defaultCommand();
          armAngleChange.defaultCommand();
        }
          break;
      case 3:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS){
        elevator.setL3Position();
        armAngleChange.setL3Position();
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }
      break;
      case 4:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        if (!L4Runing) {
          armAngleChange.getDown();
        }

        if (RobotContainer.armAngleChange.getPosition() > 0 || L4Runing) {
          L4Runing = true;
          elevator.setL4Position();
        }

        if (RobotContainer.elevator.getPosition() > Constants.ArmAngleChangeConstants.CAN_OPEN_ARM) {
      
            armAngleChange.setL4Position();

          
        }
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }

        break;


      default:
        elevator.setL3Position();
        armAngleChange.setL3Position();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();
        return pythagoras(robotPose.getX() - x, robotPose.getY() - y);
    }

    private double pythagoras(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    private Pose2d getTarget() {
        int id = getClosestAprilTag();
        return !Robot.isRight ? 
            Constants.ReefAssistConstants.LEFT_TARGETS.get(id) : 
            Constants.ReefAssistConstants.RIGHT_TARGETS.get(id);
    }
    private double getDistanceFromTarget(){
      return pythagoras(RobotContainer.s_Swerve.getPose().getX() - target.getX(),RobotContainer.s_Swerve.getPose().getY() - target.getY());
    }

}
