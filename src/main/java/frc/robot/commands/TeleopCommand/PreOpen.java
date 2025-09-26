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
public class PreOpen extends Command {
  /** Creates a new ChoosableLevelCommand. */

  // else if (Robot.isRight) {
    //  outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.rightGetY(Id),-0.09);
    //  outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.rightGetX(Id), 0.53);
    // }else{
      //   outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.leftGetY(Id),0.11);
      //   outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.leftGetX(Id), 0.31);
  // }
  ArmAngleChange armAngleChange;
  Elevator elevator;
  boolean L4Runing = false;
  boolean isCoralChange;

  public  PreOpen(ArmAngleChange armAngleChange, Elevator elevator) {
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
    L4Runing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.arm.isCoralIn()) {
      isCoralChange = true;
    }



    switch (Robot.level) {
      case 1:
        if (isInRadus()) {
          elevator.setL1Position();
          armAngleChange.setL1Position();
        }else{
          elevator.defaultCommand();
          armAngleChange.defaultCommand();
        }



        break;
      case 2:
      if (isInRadus()) {

        elevator.setL2Position();
        armAngleChange.setL2Position();
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }


  

        break;
      case 3:

      if (isInRadus()) {
        elevator.setL3Position();
        armAngleChange.setL3Position();
        

      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }
      
      
      
      
      break;
      case 4:
      if (isInRadus()) {
        
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
        

      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
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
  
  private boolean isInRadus(){
    if (Robot.isRight) {
      if (RobotContainer.aprilTag.rightGetX() < Constants.OPEN_RADUS) {
        return true;
      }
      return false;
    }else{
      if (RobotContainer.aprilTag.leftGetX() < Constants.OPEN_RADUS) {
        return true;
      }
      return false;
    } 
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return !arm.isCoralIn() && isCoralChange;
  }
}
