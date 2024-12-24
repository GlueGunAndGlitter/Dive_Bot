// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static GenericEntry IntakeFrontMotorShufflebordSpeed;
  public static GenericEntry IntakeBackMotorShufflebordSpeed;

  public static GenericEntry ShooterBackMotorShufflebordSpeed;
  public static GenericEntry ShooterFrontTopShufflebordSpeed;
  public static GenericEntry ShooterFrontBottomShufflebordSpeed;


  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer robotContainer;

  private Command autonomousCommand;

  boolean isRedAlliance;
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
    warmupCommand().schedule();



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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void smartDashboard () {
    IntakeFrontMotorShufflebordSpeed = Shuffleboard.getTab("Intake").add("backSpeed", 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    IntakeBackMotorShufflebordSpeed = Shuffleboard.getTab("Intake").add("frontSpeed", 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
        //shooter
    ShooterBackMotorShufflebordSpeed = Shuffleboard.getTab("Shooter").add("BackSpeed", 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    ShooterFrontTopShufflebordSpeed = Shuffleboard.getTab("Shooter").add("frontSpeed", 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    ShooterFrontBottomShufflebordSpeed = Shuffleboard.getTab("Shooter").add("frontSpeed", 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
  }

    public static Command warmupCommand() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(3.0, 3.0, new Rotation2d()), new Pose2d(6.0, 6.0, new Rotation2d()));
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(4.0, 4.0, 4.0, 4.0),
        new GoalEndState(0.0, Rotation2d.fromDegrees(90), true));

    return new FollowPathHolonomic(
        path,
        Pose2d::new,
        ChassisSpeeds::new,
        (speeds) -> {
        },
        new HolonomicPathFollowerConfig(4.5, 0.4, new ReplanningConfig()),
        () -> true)
        .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"))
        .ignoringDisable(true);
  }

}
