package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RumbleForHalfSecend extends Command {
  private final Timer timer = new Timer();  // Timer to track the rumble duration
  
  /** Creates a new RumbleForHalfSecend. */
  public RumbleForHalfSecend() {
    // Use addRequirements() here to declare subsystem dependencies, if needed.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start the rumble
    RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 1);
    timer.reset();  // Reset the timer
    timer.start();  // Start the timer
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No additional logic is required in execute(), as the rumble is controlled in initialize() and is stopped after 0.5 seconds.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the rumble once the command finishes or is interrupted
    RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command when 0.5 seconds have passed
    return timer.get() >= 0.5;
  }
}
