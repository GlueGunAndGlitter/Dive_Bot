// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
//   /** Creates a new Arm. */
    TalonFXConfiguration motorConfig;
    TalonFX motor = new TalonFX(Constants.ArmConstants.MOTOR_ID);
    DigitalInput input = new DigitalInput(Constants.ArmConstants.BEAM_BREAKE_ID);

      public Arm() {
        motorConfig =new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(motorConfig);
      }

      public void stopShoot(){
            motor.stopMotor();
      }


      public void outPutL4(){
        motor.set(-Constants.ArmConstants.OUT_SPEEED);
      }

      public void outPutL2L3(){
        motor.set(Constants.ArmConstants.OUT_SPEEED);
      }

      public void outPutL1(){
        motor.set(Constants.ArmConstants.L1_SPEEED);
      }

      public boolean isCoralIn(){
        return !input.get();
      }

      private double beamBreakeSpeedGive(){
        if (!isCoralIn()) {
          return Constants.ArmConstants.INTAKE_SPEED;
        }else{
          return 0;
        }
      }


      public Command intakCommand(){
        return this.run(()-> motor.set(beamBreakeSpeedGive()));

      }

      public Command L1Output(){
        return this.run(()-> motor.set(Constants.ArmConstants.L1_SPEEED));
    }

      public Command outCommand(){
          return this.run(()-> motor.set(-Constants.ArmConstants.OUT_SPEEED));
    }

    public Command normalOutCommand(){
        return this.run(()-> motor.set(Constants.ArmConstants.OUT_SPEEED));
    }

    
      public Command StopShootCommand(){
        return this.run(()-> stopShoot());
  }
  @Override
  public void periodic() {
    System.out.println(input.get());
    // This method will be called once per scheduler run
  }
}
