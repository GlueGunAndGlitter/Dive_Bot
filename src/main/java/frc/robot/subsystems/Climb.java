// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  // TalonFX climbMotor;
  // Servo servo;
  // TalonFXConfiguration motorConfig;


  public Climb() {
    // climbMotor = new TalonFX(20);
    // servo = new Servo(2);
    // motorConfig = new TalonFXConfiguration();

    // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // climbMotor.getConfigurator().apply(motorConfig);
  }

  // public void setMaxSpeed(){
  //   climbMotor.set(-1);
  // }
  


  // public Command rotateMotor(){
  //   return this.run(() -> setMaxSpeed());
  // }


  // public Command defaultCommand(){
  //   return this.run(() -> climbMotor.stopMotor());
  // }

  @Override
  public void periodic() {
    // climbMotor.set(RobotContainer.xboxController.getLeftTriggerAxis());
  }
}


