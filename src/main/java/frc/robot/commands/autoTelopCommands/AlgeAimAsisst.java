// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoTelopCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.PIDOutput_PIDOutputModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.ObjectDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgeAimAsisst extends Command {
  PIDController controller;
  DoubleSupplier translationX;
  DoubleSupplier translationY;
  DoubleSupplier rotationSup;
  /** Creates a new AlgeAimAsisst. */
  public AlgeAimAsisst(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup) {

      this.rotationSup = rotationSup;
      this.translationX = translationX;
      this.translationY = translationY;

      controller = new PIDController(0, 0, 0);
         // apply deadband
        
      }
      
      // Use addRequirements() here to declare subsystem dependencies.
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
    double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    double output = controller.calculate(RobotContainer.objectDetection.getYaw(),0);
    if (RobotContainer.objectDetection.isTarget()) {
      RobotContainer.s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
      output * Constants.Swerve.maxAngularVelocity, 
      true, 
      true);
    } else {
      RobotContainer.s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
      rotationVal * Constants.Swerve.maxAngularVelocity, 
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
    return false;
  }
}
