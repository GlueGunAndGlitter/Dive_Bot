// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {


  private static double targetVelocity = 0;
  private static double targetVelocity1 = 0;
  /** Creates a new Shooter. */
   private static TalonFX frontBottomMotor;
   private static CANSparkFlex frontTopMotor;
   private static CANSparkFlex backMotor;
 
   private static TalonFXConfiguration controller;
   private static VelocityVoltage velocityControl;
    private Encoder frontBottomMotorEncoder;
   private RelativeEncoder frontTopMotorEncoder;
   private RelativeEncoder backMotorEncoder;

   private static SparkPIDController frontPIDController;
   private static SparkPIDController backPIDController;



  public Shooter() {
    //motor declarations:
    frontBottomMotor = new TalonFX(Constants.Shooter.FRONT_BOTTOM_MOTOR_ID);
    frontTopMotor = new CANSparkFlex(Constants.Shooter.FRONT_TOP_MOTOR_ID, MotorType.kBrushless);
    backMotor = new CANSparkFlex(Constants.Shooter.BACK_MOTOR_ID, MotorType.kBrushless);

    frontPIDController = frontTopMotor.getPIDController();
    backPIDController = backMotor.getPIDController();
    controller = new TalonFXConfiguration();

    velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);

  
    controller.Slot0.kP = Constants.Shooter.shooterP;
    controller.Slot0.kI = Constants.Shooter.shooterI;
    controller.Slot0.kD = Constants.Shooter.shooterD;
    velocityControl.FeedForward = Constants.Shooter.shooterF;

    frontBottomMotor.getConfigurator().apply(controller);
    


    frontPIDController.setP(0);
    frontPIDController.setI(0);
    frontPIDController.setD(0);
    frontPIDController.setFF(0);

    backPIDController.setP(0);
    backPIDController.setI(0);
    backPIDController.setD(0);
    backPIDController.setFF(0);
    

    frontTopMotor.burnFlash();
    backMotor.burnFlash();

  }

  //setRPM for each sparkflex motors
  public static void setFrontTopRPM(Double velocity){
    targetVelocity = velocity;
    frontPIDController.setReference(targetVelocity,ControlType.kVelocity);
  }

  public static void setBackRPM(Double velocity){
    targetVelocity1 = velocity;
    backPIDController.setReference(targetVelocity1,ControlType.kVelocity);
  }
  //setRPM for talonFX motor
  public static void setFrontbottomRPM(double velocity, double feedForward){
    frontBottomMotor.setControl(velocityControl.withVelocity(velocity / 60.0).withFeedForward(feedForward));
  }


  //set commands
  public Command setBackRPMCommand() {
    return this.run(() -> setBackRPM(null));
  }

   public Command setFrontTopRPMCommand() {
    return this.run(() -> setFrontTopRPM(null));
  }

   public Command setFrontBottomRPMCommand() {
    return this.run(() -> setFrontTopRPM(null));
  }


  //get rpm
  public static double getFrontTopRPM(){
    return frontTopMotor.getEncoder().getVelocity(); 
  }

    public static double getFrontBottomRPM(){
      return frontBottomMotor.getVelocity().getValue() * 60;
  }

    public static double getBackRPM(){
      return backMotor.getEncoder().getVelocity(); 

  }

  //setspeedPercentage
  double maxMotorOutput;
  public static void setspeedPercentage(){
    
  }
  
    



  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
