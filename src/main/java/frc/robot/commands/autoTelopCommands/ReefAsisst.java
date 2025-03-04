// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoTelopCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAsisst extends Command {


  DoubleSupplier translationX;
  DoubleSupplier translationY;
  PIDController orizontalPID;
  PIDController forwordBackwordsPID;
  PIDController rotionPID;
  DoubleSupplier rotationSup;
  double angle = 0;
  Swerve swerve;
  boolean targertIDChange = false;
  ArmAngleChange armAngleChange;

  /** Creates a new ReefAsisst. */
  public ReefAsisst(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,DoubleSupplier rotationSup, ArmAngleChange armAngleChange) {
    orizontalPID = new PIDController(1, 0, 0);
    forwordBackwordsPID = new PIDController(0.4, 0, 0);
    rotionPID = new PIDController(0.02, 0, 0);

    this.swerve = swerve;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotationSup = rotationSup;
    this.armAngleChange = armAngleChange;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
    addRequirements(this.armAngleChange);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     targertIDChange = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(Robot.isRight);
    //System.out.println(getAngle());
    armAngleChange.zeroPosition();
    double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
    double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    double outputOrizontal;
    double outputforwordBackwords;
    double output;


    
    
    
    if (RobotContainer.aprilTag.hasTarget() && !targertIDChange) {
      angle = getAngle();
      targertIDChange = true;
    }
    output = Math.IEEEremainder(angle - Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),360) * 0.02;
    

     if (Robot.level == 1) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(),0.095);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(), 0.34);
     }else if (Robot.isRight) {
      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(),0.16);
      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(), 0.29);
    }else{
     outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(),0.03);
     outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetX(), 0.53);
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
  public void end(boolean interrupted) {
     targertIDChange = false;
  }

  private double getAngle(){
    int id = -1;
    int leftID = RobotContainer.aprilTag.leftGetId();
    int rightID = RobotContainer.aprilTag.rightGetId();
    if (leftID == rightID){
      id = leftID;
    }else if(leftID == -1 && rightID != -1){
      id = rightID;
    }else if(leftID != -1 && rightID == -1){
      id = leftID;
    }else{
      id = rightID;
    }
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
     if (Robot.isRight){
     return Math.abs(RobotContainer.aprilTag.leftGetY() - 0.16) < 0.03 && Math.abs(RobotContainer.aprilTag.leftGetX() - 0.29) < 0.15;
    }

    else{
     return Math.abs(RobotContainer.aprilTag.rightGetY() - 0.03) < 0.03 && Math.abs(RobotContainer.aprilTag.rightGetX() - 0.51) < 0.15;

    }
  }
}
