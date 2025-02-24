// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  // TalonFX climbMotor;
  // Servo servo;
  // TalonFXConfiguration motorConfig;


  public Climb() {
    // climbMotor = new TalonFX(Constants.ClimbConstants.MOTOR_ID);
    // servo = new Servo(Constants.ClimbConstants.SERVO_PORT);
    // motorConfig = new TalonFXConfiguration();

    // motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // climbMotor.getConfigurator().apply(motorConfig);
  }

  // public void setMaxSpeed(){
  //   climbMotor.set(1);
  // }
  


  // public Command rotateMotor(){
  //   return this.run(() -> setMaxSpeed());
  // }


  // public Command defaultCommand(){
  //   return this.run(() -> climbMotor.stopMotor());
  // }

  // @Override
  // public void periodic() {
  //   if (RobotContainer.soolyXboxControler.getRightTriggerAxis() > 0.5) {
  //       servo.setAngle(180);
  //   }else{
  //       servo.setAngle(0);
  //   }
  // }
}


