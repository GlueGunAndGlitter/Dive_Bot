// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoTelopCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAsisstAuto extends Command {


  DoubleSupplier translationX;
  DoubleSupplier translationY;
  PIDController orizontalPID;
  PIDController forwordBackwordsPID;
  PIDController rotionPID;
  DoubleSupplier rotationSup;
  boolean isLeft;

  Swerve swerve;

  /** Creates a new ReefAsisst. */
  public ReefAsisstAuto(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,DoubleSupplier rotationSup,boolean isLeft) {
    orizontalPID = new PIDController(1, 0, 0);
    forwordBackwordsPID = new PIDController(0.3, 0, 0);
    rotionPID = new PIDController(0.02, 0, 0);

    this.isLeft = isLeft;
    this.swerve = swerve;
    //apply deadband 
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotationSup = rotationSup;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
    double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    double outputOrizontal;
    double outputforwordBackwords;
    double output;

    if (isLeft) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(),0.16);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(), 0.32);
      output = rotionPID.calculate(Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),60);
  
    }else{
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(),0.03);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetX(), 0.53);
      output = rotionPID.calculate(Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),-60);
    }
    
    if (RobotContainer.aprilTag.hasTarget()) {
      swerve.drive(
        new Translation2d(outputforwordBackwords, outputOrizontal).times(Constants.Swerve.maxSpeed), 
        -output * Constants.Swerve.maxAngularVelocity, 
        false, 
        true);
      
    } else{
      swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        true, 
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.aprilTag.rightGetY() - 0.04) < 0.05 && Math.abs(RobotContainer.aprilTag.rightGetX() - 0.51) < 0.15;
    
  }
}
