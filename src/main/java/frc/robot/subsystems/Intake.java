// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    TalonFX frontMotor; 
    TalonFX backMotor;

    public TalonFXConfiguration frontMotorConfiguration;
    public TalonFXConfiguration backMotorConfiguration;

  /** Creates a new Intake. */
  public Intake() {

    frontMotor = new TalonFX(Constants.IntakeConstants.FRONT_MOTOR_ID);
    backMotor = new TalonFX(Constants.IntakeConstants.BACK_MOTOR_ID);

    frontMotorConfiguration = new TalonFXConfiguration();
    backMotorConfiguration = new TalonFXConfiguration();

    frontMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    backMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void input() {
  //   frontMotor.set(Robot.intakeFrontMotorShufflebordSpeed.getDouble(0));
  //   backMotor.set(Robot.intakeBackMotorShufflebordSpeed.getDouble(0));
  }

  private void outPut() {

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