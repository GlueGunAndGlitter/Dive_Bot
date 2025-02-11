// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.condition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutL1Command extends Command {
  /** Creates a new OutL1Cmmand. */
  boolean isCoralChange;
  Arm arm;
  public OutL1Command(Arm arm) {
    isCoralChange = false;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
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
    if (Math.abs(RobotContainer.armAngleChange.getPosition()) > Math.abs( Constants.ArmAngleChangeConstants.L1_POSITION) -0.5) {
       RobotContainer.arm.outPutL1();
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
    // !RobotContainer.arm.isCoralIn();
  }
}
