// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

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

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousReefAssist extends Command {

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
    boolean isLeft;
    int wig;

    /** Creates a new ReefAsisst. */
    public AutonomousReefAssist(Swerve swerve, int wig, boolean isLeft) {
        orizontalPID = new PIDController(1, 0, 0);
        forwordBackwordsPID = new PIDController(0.4, 0, 0);
        xPidController = new PIDController(0., 0, 0);
        yPidController = new PIDController(0.4, 0, 0);
        anglePID = new PIDController(0.0065, 0, 0);

        this.swerve = swerve;
        this.isLeft = isLeft;
        this.wig = wig;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Constants.ArmAngleChangeConstants.L4_POSITION = -33;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
        targertIDChange = false;

        Id = getId();
        angle = getAngle(Id);

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
  
    
      if (!isLeft) {
        outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(Id),-0.12);
        outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetXForID(Id), 0.55);
      }else{
        outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.07);
        outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetXForID(Id), 0.35);
      }
  
       if (angleError > 7) {
        outputforwordBackwords = 0;
        outputOrizontal = 0;
       }
       System.out.println(notSeeTarget);
      if (RobotContainer.aprilTag.hasIDAt(Id, isLeft)) {
        
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

    private double getAngle(int id) {

      if (id ==7 ||id == 18) {
        return 0;
      }else if (id == 8 || id == 17) {
        return 60;
      }else if (id == 6 || id == 19) {
        return 
         -60
        ;
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

       if (!isLeft){
         return Math.abs(RobotContainer.aprilTag.rightGetY(Id) + 0.12) < 0.04 && Math.abs(RobotContainer.aprilTag.rightGetXForID(Id) - 0.55) < 0.05;
        }
        
        else{
        return Math.abs(RobotContainer.aprilTag.leftGetY(Id) - 0.07) < 0.04 && Math.abs(RobotContainer.aprilTag.leftGetXForID(Id) - 0.37) < 0.05;
  
      }
    }
    private int getId() { if (isRedAlliance) { switch (wig) { 
      case 1: return 7;
       case 2: return 8;
       case 3: return 9;
       case 4: return 10;
       case 5: return 11;
       case 6: return 6; 
      } 
      }else{ 
        switch (wig) { 
        case 1: return 18;
         case 2: return 17;
         case 3: return 22;
         case 4: return 21;
         case 5: return 20;
         case 6: return 19; 
        }
         } 
         return isRedAlliance ? 17 : 8; }



    private double pythagoras(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
}
