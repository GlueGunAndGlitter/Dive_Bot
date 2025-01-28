// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
  TalonFX motor1;
  TalonFX motor2;
  /** Creates a new Test. */
  public Climb() {
    motor1 = new TalonFX(Constants.ClimbConstants.MOTOR_1);
    motor2 = new TalonFX(Constants.ClimbConstants.MOTOR_2);
  }

  @Override
  public void periodic() {
    motor1.set(RobotContainer.xboxController.getLeftTriggerAxis());
    motor2.set(RobotContainer.xboxController.getLeftTriggerAxis());
    // This method will be called once per scheduler run
  }
}
