// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousReefAssist extends Command {



  PIDController orizontalPID;
  PIDController forwordBackwordsPID;
  PIDController rotionPID;
  double angle = 0;
  Swerve swerve;
  boolean targertIDChange = false;
  ArmAngleChange armAngleChange;
  boolean isRedAlliance;
  int Id;
  boolean isLeft;
  int wig;

  /** Creates a new ReefAsisst. */
  public AutonomousReefAssist(Swerve swerve, ArmAngleChange armAngleChange,boolean isLeft,int wig) {
    orizontalPID = new PIDController(1.05, 0, 0);
    forwordBackwordsPID = new PIDController(0.3, 0, 0);
    rotionPID = new PIDController(0.02, 0, 0);

    this.swerve = swerve;
    this.armAngleChange = armAngleChange;
    this.isLeft = isLeft;
    this.wig = wig;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
    addRequirements(this.armAngleChange);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
     targertIDChange = false;
     angle = getAngle();
     if (isRedAlliance) {
        if (wig == 6) {
          Id = 6;
        }else{
          Id = wig + 6;
        }
     }else{
        if (wig == 2) {
          Id = 17;
        }else{
          Id = wig + 17;
        }
     }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println(Robot.isRight);
    //System.out.println(getAngle());
    armAngleChange.zeroPosition();
    double outputOrizontal;
    double outputforwordBackwords;
    double output;


    
    
    output = Math.IEEEremainder(angle - Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),360) * 0.01;
    

    if (Robot.level == 1) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.02);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(Id), 0.31);
    } 
    else if (isLeft 
    && RobotContainer.aprilTag.isRightCameraHasTarget()
    ) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.11);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(Id), 0.31);
    }else 
    if(!isLeft && RobotContainer.aprilTag.isLeftCameraHasTarget())
    {  
     outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(Id),-0.09);
     outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetX(Id), 0.53);
    }
    else{
      outputOrizontal = 0;
      outputforwordBackwords = -0.3;
    }

    if (RobotContainer.aprilTag.hasTarget()) {
      
      swerve.drive(
        new Translation2d(outputforwordBackwords, outputOrizontal).times(Constants.Swerve.maxSpeed), 
        -output * Constants.Swerve.maxAngularVelocity, 
        false, 
        true);
      
    } else{
      swerve.drive(
        new Translation2d(outputforwordBackwords, outputOrizontal).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        true, 
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     targertIDChange = false;
  }

  private double getAngle(){
    int id = Id;
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
      return Math.abs(RobotContainer.aprilTag.leftGetY(Id) -0.02) < 0.06 && Math.abs(RobotContainer.aprilTag.leftGetX(Id) - 0.31) < 0.2;
    }
     else if (isLeft){
     return Math.abs(RobotContainer.aprilTag.leftGetY(Id) - 0.11) < 0.06 && Math.abs(RobotContainer.aprilTag.leftGetX(Id) - 0.31) < 0.2;
    }

    else{
     return Math.abs(RobotContainer.aprilTag.rightGetY(Id) + 0.09) < 0.06 && Math.abs(RobotContainer.aprilTag.rightGetX(Id) - 0.53) < 0.2;

    }



  }    
}
