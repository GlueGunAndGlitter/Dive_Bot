// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    CANSparkFlex frontMotor = new CANSparkFlex(Constants.IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
    CANSparkFlex backMotor = new CANSparkFlex(Constants.IntakeConstants.BACK_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void input() {
    backMotor.set(Robot.IntakeBackMotorShufflebordSpeed.getDouble(0));
    frontMotor.set(Robot.IntakeFrontMotorShufflebordSpeed.getDouble(0));
  }

  private void outPut() {
    backMotor.set(Robot.IntakeBackMotorShufflebordSpeed.getDouble(0));
    frontMotor.set(Robot.IntakeFrontMotorShufflebordSpeed.getDouble(0));
  }

  private void stopMotors() {
    frontMotor.set(0);
    backMotor.set(0);
  }

  public Command inputCommand() {
    return this.run(() -> input());
  }

  public Command outPutCommand() {
    return this.run(() -> outPut());
  }

  public Command stopMotorsCommand() {
    return this.run(() -> stopMotors());
  }
}
