// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  Arm arm;
  ArmAngleChange armAngleChange;
  /** Creates a new CoralIntake. */
  public Intake(Arm arm, ArmAngleChange armAngleChange) {
    this.arm = arm;
    this.armAngleChange = armAngleChange;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
    addRequirements(this.armAngleChange);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    armAngleChange.setIntakePosition();
    if (!arm.isCoralIn()) {
        arm.intake();
    }else{
      arm.stopShoot();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isCoralIn();
  }
}
