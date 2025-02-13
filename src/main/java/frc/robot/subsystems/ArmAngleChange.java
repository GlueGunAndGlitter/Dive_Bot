// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmAngleChange extends SubsystemBase {
  /** Creates a new ArmAngleChange. */
  
  TalonFX armAngleChangeMotor;
  PIDController positionPID;
  TalonFXConfiguration motorConfig;
  double wantedPosition;
  GenericEntry randomPosition;

  
  public ArmAngleChange() {
    
    armAngleChangeMotor = new TalonFX(Constants.ArmAngleChangeConstants.ArmAngleChange_Motor_ID);
    positionPID = new PIDController(Constants.ArmAngleChangeConstants.KP_POSITION_PID, Constants.ArmAngleChangeConstants.KI_POSITION_PID, Constants.ArmAngleChangeConstants.KD_POSITION_PID);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armAngleChangeMotor.getConfigurator().apply(motorConfig);
  }

    public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    motor.set(speed);
  }



  
  public Command zeroPositionCommad(){
    return this.run(()-> setPosition(armAngleChangeMotor, 0.7, 0));
  }

  public void setL1Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L1_POSITION);
  }
  
  public void setL2Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L2_ANGLE_POSITION);
  }
  
  public void setL3Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION);
  }
  
  public void setL4Position(){
    setPosition(armAngleChangeMotor, 0.7, Constants.ArmAngleChangeConstants.L4_POSITION);
  }
  
  public Command setIntakePositionCommand(){
    return this.run(() -> setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.INTAKE_POSITION));
  }
  public Command setL1PositionCommand(){
    return this.run(() -> setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L1_POSITION));
  }
  public Command setL4PositionCommand(){
    return this.run(() -> setPosition(armAngleChangeMotor, 0.7, Constants.ArmAngleChangeConstants.L4_POSITION));
  }

  public Command setL2PositionCommand() {
    return this.run(() -> setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L2_ANGLE_POSITION));
  }
  public Command setL3PositionCommand() {
    return this.run(() -> setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION));
  }
  public Command setLowAlgeaCommand(){
    return this.run(()-> setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.LOW_ALGEA_POSITION));
  }








  public double getPosition(){
    return armAngleChangeMotor.getPosition().getValueAsDouble();
  }




  @Override
  public void periodic() {
    System.out.println("level: " + RobotContainer.level + " isLeft: " + RobotContainer.isLeft);
    // This method will be called once per scheduler run
  }
}
