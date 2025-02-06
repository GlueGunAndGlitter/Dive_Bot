// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
//   /** Creates a new Arm. */
    TalonFX motor = new TalonFX(61);
    DigitalInput input = new DigitalInput(1);

      public Arm() {
      }

      public void stopShoot(){
            motor.stopMotor();
      }


      private double ttt(){
        if (input.get()) {
          return 0.15;
        }else{
          return 0;
        }
      }
      public Command intakCommand(){
      if (input.get()) {
        return this.run(()-> motor.set(ttt()));
      }else{
        return this.run(() -> motor.set(ttt()));
      }
      }

      public Command L1Output(){
        return this.run(()-> motor.set(0.1));
    }

      public Command outCommand(){
          return this.run(()-> motor.set(-0.3));
    }

    public Command normalOutCommand(){
        return this.run(()-> motor.set(0.3));
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
