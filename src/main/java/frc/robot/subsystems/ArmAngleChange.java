// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmAngleChange extends SubsystemBase {
  /** Creates a new ArmAngleChange. */
  
  TalonFX armAngleChangeMotor;
  PIDController positionPID;
  TalonFXConfiguration motorConfig;
  double wantedPosition;
  final TrapezoidProfile m_profile = new TrapezoidProfile(
   new TrapezoidProfile.Constraints(80, 160)
  );
  TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  public ArmAngleChange() {
    
    armAngleChangeMotor = new TalonFX(Constants.ArmAngleChangeConstants.ArmAngleChange_Motor_ID);
    positionPID = new PIDController(Constants.ArmAngleChangeConstants.KP_POSITION_PID, Constants.ArmAngleChangeConstants.KI_POSITION_PID, Constants.ArmAngleChangeConstants.KD_POSITION_PID);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


     motorConfig.Slot0.kP = 2; 
     motorConfig.Slot0.kI = 0; 
     motorConfig.Slot0.kD = 0.2; 

    armAngleChangeMotor.getConfigurator().apply(motorConfig);
  }

    public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
    double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
    double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
    motor.set(speed);
  }

  public void resetToAbsolute(){
    armAngleChangeMotor.setPosition(0);
  }




  public Command resetToAbsoluteCommand(){
    return this.run(() -> resetToAbsolute());
  }

  public void setIntakePosition(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.INTAKE_POSITION);
  }
  
  public void zeroPosition(){
    setPosition(armAngleChangeMotor, 0.7, 0);
  }
  public Command zeroPositionCommad(){
    return this.run(()-> zeroPosition());
  }


  public void setL1Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L1_POSITION);
  }
  public void algi(){
    setPosition(armAngleChangeMotor, 0.3, 10);
  }

  
  public void setL2Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L2_ANGLE_POSITION);
  }
  
  public void setL3Position(){
    setPosition(armAngleChangeMotor, 0.3, Constants.ArmAngleChangeConstants.L3_ANGLE_POSITION);
  }
  
  public void setL4Position(){
    m_setpoint = m_profile.calculate(0.020, m_setpoint, Constants.ArmAngleChangeConstants.L4_POSITION_TRAPEZ);
    
    m_request.Position = m_setpoint.position;
    m_request.Velocity = m_setpoint.velocity;
    armAngleChangeMotor.setControl(m_request); 
    }
  

  public double getPosition(){
    return armAngleChangeMotor.getPosition().getValueAsDouble();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
