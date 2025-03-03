
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
//   /** Creates a new Arm. */
    TalonFXConfiguration motorConfig;
    TalonFXConfiguration i;
    double max;
    TalonFX motor = new TalonFX(Constants.ArmConstants.MOTOR_ID);
    TalonFX motorMashpeh = new TalonFX(62);

  // Final target of 200 rot, 0 rps
  // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(-37, 0);
  // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


  DigitalInput input = new DigitalInput(1);
  DigitalInput inputniga = new DigitalInput(3);

    //DigitalInput beamBrake = new DigitalInput(4);

    PIDController positionPID = new PIDController(Constants.ArmAngleChangeConstants.KP_POSITION_PID, Constants.ArmAngleChangeConstants.KI_POSITION_PID, Constants.ArmAngleChangeConstants.KD_POSITION_PID);


    
    public void setPosition(TalonFX motor, double maxSpeed, double wantedPosition){
      double PIDOutput = positionPID.calculate(motor.getPosition().getValueAsDouble(), wantedPosition);
      double speed = Math.signum(PIDOutput) * Math.min(maxSpeed, Math.abs(PIDOutput));
      motor.set(speed);
    }

      public Arm() {
        motorConfig =new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(motorConfig);

        // i = new TalonFXConfiguration();
        // i.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // i.Slot0.kP = 1; 
        // i.Slot0.kI = 0; 
        // i.Slot0.kD = 0.03; 

        // motorMashpeh.getConfigurator().apply(i);
      }

      public void stopShoot(){
            motorMashpeh.stopMotor();
            motor.stopMotor();
      }

    //   public void test(){
    //     // calculate the next profile setpoint    
    //     m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    
    // // send the request to the device
    //     m_request.Position = m_setpoint.position;
    //     m_request.Velocity = m_setpoint.velocity;
    //     motorMashpeh.setControl(m_request);
    //   }

      // public Command testCommand(){
      //   return this.run(()-> test());
      // }
      public Command motorzero(){
        return this.run(()-> motorMashpeh.setPosition(0));
      }

      public void outPutL4(){
        motor.set(-Constants.ArmConstants.OUT_SPEEED);
      }
      public void outPutL4Slowwww(){
        motor.set(-0.03);
      }


      public void outPutL2L3(){
        motor.set(Constants.ArmConstants.OUT_SPEEED);
      }
      public void setRumbleCoralIn(){
        if (isCoralIn()) {
        }
      }
      public Command stopCommand(){
        return this.run(() -> motor.set(0));
      }
      public void outPutL1(){
        motor.set(Constants.ArmConstants.L1_SPEEED);
      }

      public boolean isCoralIn(){
        // return false;
        return !inputniga.get();
      } 

      public void output(){
        motor.set(-0.08);
     }

     public void outputAlgi(){
      motor.set(0.7);
   }
     


      public void intake(){
         motor.set(Constants.ArmConstants.INTAKE_SPEED);
         motorMashpeh.set(0.6);
      }

      public void slowIntake(){
        motor.set(0.05);
     }

      public Command L1Output(){
        return this.run(()-> motor.set(Constants.ArmConstants.L1_SPEEED));
    }

    public Command setRumbleCoralInCommand(){
      return this.run(()-> setRumbleCoralIn());
  }
    public Command intakeOutPutCommand(){
      return this.run(() -> motor.set(Constants.ArmConstants.INTAKE_SPEED));
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
      SmartDashboard.putBoolean("limit swich state", input.get());
      SmartDashboard.putBoolean("niga swich", inputniga.get());

      SmartDashboard.putNumber("motor crent", motor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("motor posion",  motor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("number pose", motorMashpeh.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("seppint",  -37);

    //max = Math.max(max, input.getVoltage());
    // This method will be called once per scheduler run
  }
}
