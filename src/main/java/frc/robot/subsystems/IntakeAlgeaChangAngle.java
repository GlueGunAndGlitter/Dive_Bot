// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAlgeaChangAngle extends SubsystemBase {
    TalonFX intakeAlgeaChangAngleMotor; 
    PIDController positionPID;
    //TalonFX backMotor;

    
   TalonFXConfiguration intakeAlgeaChangAngleConfiguration;
   // public TalonFXConfiguration backMotorConfiguration;

  /** Creates a new Intake. */
  public IntakeAlgeaChangAngle() {

    intakeAlgeaChangAngleMotor = new TalonFX(Constants.IntakeAlgeaChangAngle.INTAKE_ALGEA_ID);

    intakeAlgeaChangAngleConfiguration = new TalonFXConfiguration();

    intakeAlgeaChangAngleConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeAlgeaChangAngleMotor.getConfigurator().apply(intakeAlgeaChangAngleConfiguration);
    
  } 
  
  public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
  }

  private void zeroPositionAlgea(){
      setPosition(intakeAlgeaChangAngleMotor, 0.1, 0);
  }

  public Command algeaIntakeCommand(){
    return this.run(()-> setPosition(intakeAlgeaChangAngleMotor, 0.7,5));

  }

  public Command zeroPositionAlgeaCommand(){
    return this.run(() -> zeroPositionAlgea());

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
    intakeAlgeaChangAngleMotor.set(0);
    //backMotor.set(0);
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