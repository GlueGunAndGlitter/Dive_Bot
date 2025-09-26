// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoTelopCommands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ApriltagConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAsisst extends Command {



  PIDController orizontalPID;
  PIDController forwordBackwordsPID;

  PIDController anglePID;
  PIDController yPidController; 
  PIDController xPidController;

  Pose2d notSeeTarget;

  double angleError;

  double angle = 0;
  Swerve swerve;
  boolean targertIDChange = false;
  boolean isRedAlliance;
  int Id;

  /** Creates a new ReefAsisst. */
  public ReefAsisst(Swerve swerve) {
    orizontalPID = new PIDController(1, 0, 0);
    forwordBackwordsPID = new PIDController(0.4, 0, 0);

    xPidController = new PIDController(0.4, 0, 0);
    yPidController = new PIDController(0.4, 0, 0);
    anglePID = new PIDController(0.0065, 0, 0);

    this.swerve = swerve;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     Constants.ArmAngleChangeConstants.L4_POSITION = -33;
     Optional<Alliance> alliance = DriverStation.getAlliance();
     isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;     targertIDChange = false;
     angle = getAngle();
     Id = getClosestAprilTag();
     notSeeTarget = Constants.ReefAssistConstants.poseMap.get(Id);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();

    //System.out.println(Robot.isRight);
    //System.out.println(getAngle());
    double outputOrizontal;
    double outputforwordBackwords;
    double output;

    angleError = angle - Math.IEEEremainder(swerve.getHeading().getDegrees(), 360);
    angleError = Math.IEEEremainder(angleError, 360); // Normalize error to -180 to 180
    

    
    double notSeeTargetXOutput = xPidController.calculate(robotPose.getX(), notSeeTarget.getX());
    double notSeeTargetYOutput = yPidController.calculate(robotPose.getY(), notSeeTarget.getY());
    output = anglePID.calculate(angleError, 0);

  
    if (Robot.level == 1) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.02);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetXForID(Id), 0.31);
    } 
    else if (Robot.isRight) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(Id),-0.12);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetXForID(Id), 0.55);
    }else{
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.07);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetXForID(Id), 0.35);
    }

    // if (Math.abs(outputOrizontal) > 0.7) {
    //   outputOrizontal = 0.7 * Math.signum(outputOrizontal);
    // }
    //  if (outputforwordBackwords > 0.2) {
    //   outputforwordBackwords = 0.2;
    //  }

     if (angleError > 7) {
      outputforwordBackwords = 0;
      outputOrizontal = 0;
     }
    //  System.out.println(notSeeTarget);
    if (RobotContainer.aprilTag.hasID(Id)) {
      
      swerve.drive(
        new Translation2d(outputforwordBackwords, outputOrizontal).times(Constants.Swerve.maxSpeed), 
        output * Constants.Swerve.maxAngularVelocity, 
        false, 
        true);
      
    } else{
      swerve.drive(
        new Translation2d(0.1, 0).times(Constants.Swerve.maxSpeed), 
        output * Constants.Swerve.maxAngularVelocity, 
        false, 
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     targertIDChange = false;
     Constants.ArmAngleChangeConstants.L4_POSITION = -39;
  }

  private double getAngle(){
    int id = getClosestAprilTag();
    //System.out.println(id);


    if (id ==7 ||id == 18) {
      return 0;
    }else if (id == 8 || id == 17) {
      return 60;
    }else if (id == 6 || id == 19) {
      return -60;
    }else if (id == 11 || id == 20) {
      return -120;
    }else if (id == 10 || id == 21) {
      return 180;
    }else if (id == 9 || id == 22) {
      return 120;  
    }else {
      return swerve.getHeading().getDegrees();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.level == 1) {
      return Math.abs(RobotContainer.aprilTag.leftGetY(Id) -0.02) < 0.06 && Math.abs(RobotContainer.aprilTag.leftGetXForID(Id) - 0.31) < 0.2;
    }
     else if (Robot.isRight){
       return Math.abs(RobotContainer.aprilTag.rightGetY(Id) + 0.12) < 0.04 && Math.abs(RobotContainer.aprilTag.rightGetXForID(Id) - 0.55) < 0.1;
      }
      
      else{
      return Math.abs(RobotContainer.aprilTag.leftGetY(Id) - 0.07) < 0.04 && Math.abs(RobotContainer.aprilTag.leftGetXForID(Id) - 0.35) < 0.1;

    }



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
        Pose2d robotPose = swerve.getPose();
        return pythagoras(robotPose.getX() - x, robotPose.getY() - y);
    }

    private double pythagoras(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }    
}
