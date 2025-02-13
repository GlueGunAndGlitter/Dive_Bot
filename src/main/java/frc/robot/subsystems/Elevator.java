// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */


  TalonFX elevatorMotor1;
  TalonFX elevatorMotor2;

  public TalonFXConfiguration motorConfig;
  PIDController positionPID;

  public Elevator() {

    elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR1_ID);
    elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR2_ID);

    positionPID = new PIDController(Constants.ElevatorConstants.KP_POSITION_PID , 0, 0);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotor1.getConfigurator().apply(motorConfig);
    elevatorMotor2.getConfigurator().apply(motorConfig);


  }
  
  public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    motor.set(speed);
  }
  
  /**
   * the speedy version of setPosition
   * @param maxSpeed should beatwean 0-1
   */
  public void setPositionSpeedy(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    if (motor.getPosition().getValueAsDouble() < wantedPosition/2 && maxSpeed != 0){
      motor.set(1);
    }else{
      motor.set(speed);
    }
    // motor.set(speed);
  }

  public double getPosition(){
    return elevatorMotor1.getPosition().getValueAsDouble();
  }


  public void set2MotorsPositionSpeedy(double maxSpeed, double wantedPosition){
    setPositionSpeedy(elevatorMotor1, maxSpeed, wantedPosition);
    setPositionSpeedy(elevatorMotor2, maxSpeed, wantedPosition);
  }
  public void set2MotorsPosition(double maxSpeed, double wantedPosition){
    setPosition(elevatorMotor1, maxSpeed, wantedPosition);
    setPosition(elevatorMotor2, maxSpeed, wantedPosition);
  }


  private void zeroPosition(){
    if (Math.abs(RobotContainer.armAngleChange.getPosition()) < 8) {
      set2MotorsPositionSpeedy(0.1, 0);
    }else{
      set2MotorsPositionSpeedy(0, 0);

    }
  }
  public Command zeroPositionCommand(){
    return this.run(() -> zeroPosition());

  }
  public Command elevatorZeroCommand(){
    return this.run(()-> set2MotorsPosition(0.3, Constants.ElevatorConstants.ZERO_POSITION));
  }
  public Command elevatorL2Command(){
    return this.run(()-> set2MotorsPosition(0.3, Constants.ElevatorConstants.L2_POSITION));

  }

  public Command elevatorL3Command(){
    return this.run(()-> set2MotorsPositionSpeedy(0.3, Constants.ElevatorConstants.L3_POSITION));

  }
  
  public Command  elevatorL4Command(){
    return this.run(()-> set2MotorsPositionSpeedy(0.3, Constants.ElevatorConstants.L4_POSITION));

  }

  public Command  elevatorCommand(){
    return this.run(()-> set2MotorsPositionSpeedy(1, 14.8));
  }
  



  @Override
  public void periodic() {
   // System.out.println(getPosition());
    // This method will be called once per scheduler run
  }
}
