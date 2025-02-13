// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LevelsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3Command extends Command {
  /** Creates a new L1Command. */
  Arm arm;
  ArmAngleChange armAngleChange;
  Elevator elevator;
  boolean isCoralChange;

  public L3Command(ArmAngleChange armAngleChange, Arm arm,Elevator elevator) {
    this.arm = arm;
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;

    addRequirements(this.arm);
    addRequirements(this.armAngleChange);
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (arm.isCoralIn()) {
        isCoralChange = true;
      }
      
      elevator.setL3Position();
      armAngleChange.setL3Position();


      if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION -0.5
      && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L3_POSITION - 1) {
          arm.outPutL2L3();
     }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !arm.isCoralIn() && isCoralChange;
  }
}
