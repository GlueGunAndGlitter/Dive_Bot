// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ArmAngleChange;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefAsisstWithPreOpen extends ParallelDeadlineGroup {
  /** Creates a new E. */
  public ReefAsisstWithPreOpen(Swerve swerve, ArmAngleChange armAngleChange, int wig, boolean isLeft, Elevator elevator) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutonomousReefAssist(swerve, wig, isLeft));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PreOpenAutonous(armAngleChange, elevator));
  }
}
