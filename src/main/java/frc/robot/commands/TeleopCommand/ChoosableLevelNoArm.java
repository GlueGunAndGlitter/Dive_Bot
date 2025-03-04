// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommand;

import java.util.logging.Level;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChoosableLevelNoArm extends Command {
  /** Creates a new ChoosableLevelCommand. */
  ArmAngleChange armAngleChange;
  Elevator elevator;

  boolean isCoralChange;

  public ChoosableLevelNoArm(ArmAngleChange armAngleChange, Elevator elevator) {
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(this.elevator);
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




    switch (Robot.level) {
      case 1:
        armAngleChange.setL1Position();


        break;
      case 2:
        elevator.setL2Position();
        armAngleChange.setL2Position();


        break;
      case 3:
        elevator.setL3Position();
        armAngleChange.setL3Position();


        break;
      case 4:
        elevator.setL4Position();

        if (RobotContainer.elevator.getPosition() > Constants.ArmAngleChangeConstants.CAN_OPEN_ARM) {
          armAngleChange.setL4Position();
        }
        break;


      default:
        elevator.setL3Position();
        armAngleChange.setL3Position();


        break;
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
