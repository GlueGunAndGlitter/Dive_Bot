// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousPreOpen extends Command {
  /** Creates a new ChoosableLevelCommand. */
  
  ArmAngleChange armAngleChange;
  Elevator elevator;
  boolean L4Runing = false;
  boolean isRedAlliance;
  Pose2d target;
  boolean isLeft;
  int wig;
  int id;
  int level;

  public  AutonomousPreOpen(ArmAngleChange armAngleChange, Elevator elevator, boolean isLeft, int wig, int level) {
    this.armAngleChange = armAngleChange;
    this.elevator = elevator;
    this.isLeft = isLeft;
    this.wig = wig;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(this.elevator);
    addRequirements(this.armAngleChange);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    L4Runing = false;
       if (DriverStation.getAlliance().get() == Alliance.Red){
          isRedAlliance = true;
        }else{
          isRedAlliance = false;
        }
        id = getTargetID();
        target = getTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  



    switch (level) {
      case 1:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        elevator.setL1Position();
        armAngleChange.setL1Position();
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }

      break;
      case 2:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        elevator.setL2Position();
        armAngleChange.setL2Position();
        }else{
          elevator.defaultCommand();
          armAngleChange.defaultCommand();
        }
          break;
      case 3:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS){
        elevator.setL3Position();
        armAngleChange.setL3Position();
      }else{
        elevator.defaultCommand();
        armAngleChange.defaultCommand();
      }
      break;
      case 4:
      if (getDistanceFromTarget() < Constants.ReefAssistConstants.OPEN_RADIUS) {
        if (!L4Runing) {
          armAngleChange.getDown();
        }

        if (RobotContainer.armAngleChange.getPosition() > 0 || L4Runing) {
          L4Runing = true;
          elevator.setL4Position();
        }

        if (RobotContainer.elevator.getPosition() > Constants.ArmAngleChangeConstants.CAN_OPEN_ARM) {
      
            armAngleChange.setBeforL4Position();

          
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  

    private double pythagoras(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    private double getDistanceFromTarget(){
      return pythagoras(RobotContainer.s_Swerve.getPose().getX() - target.getX(),RobotContainer.s_Swerve.getPose().getY() - target.getY());
    }

    private int getTargetID(){
      if (isRedAlliance){
        switch (wig) {
          case 1: return 7;
          case 2: return 8;
          case 3: return 9;
          case 4: return 10;
          case 5: return 11;
          case 6: return 6;        
        }  
      }else{
        switch (wig) {    
          case 1: return 18;
          case 2: return 17;
          case 3: return 22;
          case 4: return 21;
          case 5: return 20;
          case 6: return 19; 
        }
      }
      return 7;
    }

    private Pose2d getTarget() {
        return isLeft ? 
            Constants.ReefAssistConstants.LEFT_TARGETS.get(id) : 
            Constants.ReefAssistConstants.RIGHT_TARGETS.get(id);
    }

}
