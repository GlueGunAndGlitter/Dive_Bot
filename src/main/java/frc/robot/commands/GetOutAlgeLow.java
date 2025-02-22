// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.WatchEvent;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetOutAlgeLow extends Command {
  /** Creates a new S. */
  Elevator elevator;
  Arm arm;
  ArmAngleChange angleChange;
  public GetOutAlgeLow(Elevator elevator, Arm arm, ArmAngleChange armAngleChange) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleChange = armAngleChange;
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(this.angleChange);
    addRequirements(this.arm);
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.lowAlgi();
    angleChange.algi();
    arm.outputAlgi();
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
