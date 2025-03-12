// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommand;

import java.util.logging.Level;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChoosableLevelCommand extends Command {
  /** Creates a new ChoosableLevelCommand. */
  Arm arm;
  ArmAngleChange armAngleChange;
  Elevator elevator;
  boolean L4Runing = false;
  boolean armIsRuning = false;
  boolean isCoralChange;

  public  ChoosableLevelCommand(Arm arm, ArmAngleChange armAngleChange, Elevator elevator) {
    this.arm = arm;
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(this.arm);
    addRequirements(this.elevator);
    addRequirements(this.armAngleChange);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCoralChange = false;
    L4Runing = false;
    armIsRuning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!armIsRuning) {
      arm.stopShoot();
    }
    if (arm.isCoralIn()) {
      isCoralChange = true;
    }



    switch (Robot.level) {
      case 1:
        elevator.setL1Position();
        armAngleChange.setL1Position();


        if (Math.abs(RobotContainer.armAngleChange.getPosition()) > Math.abs( Constants.ArmAngleChangeConstants.L1_POSITION) -1.5) {
            RobotContainer.arm.outPutL1();
            armIsRuning = true;

         }


        break;
      case 2:
        elevator.setL2Position();
        armAngleChange.setL2Position();


        if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L2_ANGLE_POSITION - 1
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L2_POSITION - 1) {
           armIsRuning = true;
           RobotContainer.arm.outPutL2L3();
        }


        break;
      case 3:
        elevator.setL3Position();
        armAngleChange.setL3Position();


        if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION -0.5
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L3_POSITION - 1) {
            armIsRuning = true;
            arm.outPutL2L3();
       }


        break;
      case 4:
        if (!L4Runing) {
          armAngleChange.getDown();
        }

        if (RobotContainer.armAngleChange.getPosition() > 0 || L4Runing) {
          L4Runing = true;
          elevator.setL4Position();
        }

        if (RobotContainer.elevator.getPosition() > Constants.ArmAngleChangeConstants.CAN_OPEN_ARM) {
          armAngleChange.setL4Position();
        }

        if (Math.abs(RobotContainer.armAngleChange.getPosition() -  Constants.ArmAngleChangeConstants.L4_POSITION) < 1.5
        && Math.abs(RobotContainer.elevator.getPosition() - Constants.ElevatorConstants.L4_POSITION) < 1.5) {
          armIsRuning = true;
          RobotContainer.arm.outPutL4();
        }else{
          armIsRuning = true;
          RobotContainer.arm.outPutL4Slowwww();
        }


        break;


      default:
        elevator.setL3Position();
        armAngleChange.setL3Position();


        if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION -0.5
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L3_POSITION - 1) {
            arm.outPutL2L3();
            armIsRuning = true;
       }


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
