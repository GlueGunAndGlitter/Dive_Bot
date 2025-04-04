// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {

  Arm arm;
  ArmAngleChange armAngleChange;
  boolean isCoralChange = false;
  /** Creates a new CoralIntake. */
  public CoralIntake(Arm arm, ArmAngleChange armAngleChange) {
    this.arm = arm;
    this.armAngleChange = armAngleChange;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
    addRequirements(this.armAngleChange);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCoralChange = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    armAngleChange.setIntakePosition();
    if (!arm.isCoralIn() && !isCoralChange) {
        arm.intake();
    }else if(arm.isCoralIn()){
        isCoralChange = true;
        arm.output();
    }else if(isCoralChange && !arm.isCoralIn()){
      arm.stopShoot();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isCoralChange = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !arm.isCoralIn() && isCoralChange;
  }
}
