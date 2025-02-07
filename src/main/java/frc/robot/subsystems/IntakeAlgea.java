// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAlgea extends SubsystemBase {
  TalonFX intakeMotor;
  public TalonFXConfiguration intakeConfiguration;

  /** Creates a new intakeAlgea. */
  public IntakeAlgea() {
    intakeMotor = new TalonFX(Constants.IntakeAlgeaConstants.INTAKE_ALGEA_ID_MOTOR);

    intakeConfiguration = new TalonFXConfiguration();

    intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeMotor.getConfigurator().apply(intakeConfiguration);
  }

  public Command stopMotorsCommand(){
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command intakeCommand(){
    return this.run(() -> intakeMotor.set(0.3));
  }

  public Command OutakeCommand(){
    return this.run(() -> intakeMotor.set(-0.3));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
