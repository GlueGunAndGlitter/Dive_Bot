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
    boolean algeaIn = false;
    
   TalonFXConfiguration intakeAlgeaChangAngleConfiguration;
   // public TalonFXConfiguration backMotorConfiguration;

  /** Creates a new Intake. */
  public IntakeAlgeaChangAngle() {
    positionPID = new PIDController(0.4, 0, 0);

    intakeAlgeaChangAngleMotor = new TalonFX(Constants.IntakeAlgeaChangAngle.INTAKE_ALGEA_ID);

    intakeAlgeaChangAngleConfiguration = new TalonFXConfiguration();

    intakeAlgeaChangAngleConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeAlgeaChangAngleMotor.getConfigurator().apply(intakeAlgeaChangAngleConfiguration);
    
  } 
  
  public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    motor.set(speed);
  }

  private void zeroPositionAlgea(){
      double pos;
      if (algeaIn) {
        pos = 0.8;
      }else{
        pos = 0;
      }
      setPosition(intakeAlgeaChangAngleMotor, 0.1, pos);
  }

  
  private void algeaIntake(){
    algeaIn = true;
    setPosition(intakeAlgeaChangAngleMotor, 0.1, 4);
}
  private void algeaOutake(){
    algeaIn = false;
  }


  public Command algeaIntakeCommand(){
    return this.run(()-> algeaIntake());
  }

  public Command algeaOutakeCommand(){
    return this.run(()-> algeaOutake());
  }



  public Command zeroPositionAlgeaCommand(){
    return this.run(() -> zeroPositionAlgea());

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}