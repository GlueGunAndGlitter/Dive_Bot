// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  TalonFX climbMotor;
  // Servo servo;
  TalonFXConfiguration motorConfig;


  public Climb() {
    climbMotor = new TalonFX(19);
    // servo = new Servo(2);
    motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotor.getConfigurator().apply(motorConfig);
  }

  public void setMaxSpeedmnus(){
    climbMotor.set(1);
  }

  public void setMaxSpeed(){
    climbMotor.set(-1);
  }

  public Command rotateMotormunusCommand(){
    return this.run(() -> climbMotor.set(-1));
  }

  public Command rotateMotor(){
    return this.run(() -> climbMotor.set(1));
  }


  public Command defaultCommand(){
    return this.run(() -> climbMotor.set(RobotContainer.xboxController.getLeftTriggerAxis()));
  }

  @Override
  public void periodic() {
 }
}


