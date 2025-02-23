// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroRobotCommand extends Command {
  /** Creates a new CloseArmElevator. */
  ArmAngleChange angleChange;
  Elevator elevator;
  
  public ZeroRobotCommand(ArmAngleChange angleChange, Elevator elevator) {
    this.angleChange = angleChange;
    this.elevator = elevator;

    addRequirements(this.angleChange);
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.zeroPosition();
    angleChange.zeroPosition();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleChange.getPosition()) < 0.3  && Math.abs(elevator.getPosition()) < 0.3;
  }
}
