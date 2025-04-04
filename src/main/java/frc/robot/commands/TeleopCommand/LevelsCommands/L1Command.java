// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommand.LevelsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1Command extends Command {
  /** Creates a new L1Command. */
  Arm arm;
  ArmAngleChange armAngleChange;
  boolean isCoralChange;
  Elevator elevator;
  public 
  L1Command(ArmAngleChange armAngleChange, Arm arm, Elevator elevator) {
    this.arm = arm;
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;

    addRequirements(this.arm);
    addRequirements(this.elevator);
    addRequirements(this.armAngleChange);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      elevator.setL1Position();
      if (arm.isCoralIn()) {
        isCoralChange = true;
      }


      armAngleChange.setL1Position();


      if (Math.abs(RobotContainer.armAngleChange.getPosition()) > Math.abs( Constants.ArmAngleChangeConstants.L1_POSITION) - 1.5) {
          RobotContainer.arm.outPutL1();
       }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return !arm.isCoralIn() && isCoralChange;
  }
}
