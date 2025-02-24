
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer robotContainer;

  private Command autonomousCommand;

  boolean isRedAlliance;
  public static int level = 3;
  public static boolean isRight = true;
  public static GenericEntry intakeHigherMotorSpeed;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    robotContainer = new RobotContainer();
    smartDashboard();




    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      isRedAlliance = true;
    }else{
      isRedAlliance = false;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    // schedule the autonomous command (example)
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    System.out.println(level);
    if(RobotContainer.soolyXboxControler.getLeftBumperButton()){
      isRight = true;
    }else if(RobotContainer.soolyXboxControler.getRightBumperButton()){
      isRight = false;
    }

    if (RobotContainer.soolyXboxControler.getAButton()) {
      level = 1;
    }else if(RobotContainer.soolyXboxControler.getBButton()){
      level = 2;
    }else if(RobotContainer.soolyXboxControler.getYButton()){
      level = 3;
    }else if(RobotContainer.soolyXboxControler.getXButton()){
      level = 4;
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  public void smartDashboard() {
    intakeHigherMotorSpeed = Shuffleboard.getTab("Transportation").add("Higher motor speed", 0.7)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  }

}
