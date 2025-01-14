// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.management.MemoryType;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

public class Arm extends SubsystemBase {
//   /** Creates a new Arm. */
  static SparkFlex sparkFlex;
    public Arm() {
  
      sparkFlex = new SparkFlex(61, MotorType.kBrushless);
    }
    public static void stopShoot(){
          sparkFlex.stopMotor();
        }

      
      
      public Command intakCommand(){
          return this.run(()-> sparkFlex.set(0.3));
      }

      public Command L1Output(){
        return this.run(()-> sparkFlex.set(0.1));
    }

      public Command outCommand(){
        return this.run(()-> sparkFlex.set(-0.3));
    }
    
      public Command StopShootCommand(){
        return this.run(()-> stopShoot());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
