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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOut extends Command {
  /** Creates a new ChoosableLevelCommand. */
  Arm arm;
  boolean L4Runing = false;
  boolean armIsRuning = false;
  boolean isCoralChange;

  public  CoralOut(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(this.arm);

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


        if (Math.abs(RobotContainer.armAngleChange.getPosition()) > Math.abs( Constants.ArmAngleChangeConstants.L1_POSITION) -1.5) {
            RobotContainer.arm.outPutL1();
            armIsRuning = true;

         }else{
          arm.defaultCommand();
         }


        break;
      case 2:


        if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L2_ANGLE_POSITION - 1
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L2_POSITION - 1) {
           armIsRuning = true;
           RobotContainer.arm.outPutL2L3();
        }else{
          arm.defaultCommand();
         }



        break;
      case 3:


        if (RobotContainer.armAngleChange.getPosition() > Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION -0.5
        && RobotContainer.elevator.getPosition() > Constants.ElevatorConstants.L3_POSITION - 1) {
            armIsRuning = true;
            arm.outPutL2L3();
       }else{
        arm.defaultCommand();
       }



        break;
      case 4:

        if (Math.abs(RobotContainer.armAngleChange.getPosition() -  Constants.ArmAngleChangeConstants.L4_POSITION) < 1.5
        && Math.abs(RobotContainer.elevator.getPosition() - Constants.ElevatorConstants.L4_POSITION) < 1.5) {
          armIsRuning = true;
          RobotContainer.arm.outPutL4();
        }else{
          armIsRuning = true;
          RobotContainer.arm.outPutL4Slowwww();
        }


        break;


      default:;


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
