// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */


  TalonFX elevatorMotor;
  public TalonFXConfiguration motorConfig;
  PIDController positionPID;

  public Elevator() {

    elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);

    positionPID = new PIDController(Constants.ElevatorConstants.KP_POSITION_PID , 0, 0);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotor.getConfigurator().apply(motorConfig);


  }
  
  
  
  /**
   * @param maxSpeed should beatwean 0-1
   */
  public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    motor.set(speed);
  }

  public Command stopMotorCommand(){
    return this.run(()-> elevatorMotor.stopMotor());
  }


  public Command zeroPositionCommand(){
    return this.run(() -> setPosition(elevatorMotor, 0.2, 0.1));
  }
  
  public Command elevatorL2Command(){
    return this.run(()-> setPosition(elevatorMotor,0.2, Constants.ElevatorConstants.L2_POSITION));
  }

  public Command elevatorL3Command(){
    return this.run(()-> setPosition(elevatorMotor,0.2, Constants.ElevatorConstants.L3_POSITION));
  }
  
  public Command  elevatorL4Command(){
    return this.run(()-> setPosition(elevatorMotor,0.2, Constants.ElevatorConstants.L4_POSITION));
  }

  public Command  elevatorCommand(){
return this.run(()-> setPosition(elevatorMotor,1, 49));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
