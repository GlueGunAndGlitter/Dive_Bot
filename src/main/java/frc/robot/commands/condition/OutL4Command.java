// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.condition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutL4Command extends Command {
  Arm arm;
  boolean isCoralChange;
  /** Creates a new OutL2Command. */
  public OutL4Command(Arm arm) {
    isCoralChange = false;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCoralChange = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isCoralIn()){
      isCoralChange = true;
    }
    if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L4_POSITION - 0.5
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L4_POSITION - 1) {
        RobotContainer.arm.outPutL4();
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
