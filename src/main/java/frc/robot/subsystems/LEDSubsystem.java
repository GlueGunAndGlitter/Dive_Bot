package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class LEDSubsystem extends SubsystemBase {
  Spark blinkin = new Spark(5);

  public LEDSubsystem() {
    blinkin.set(1);
  }

  @Override
  public void periodic() {
    // blinkin.set(RobotContainer.xboxController.getRightY());
    // System.out.println(RobotContainer.xboxController.getRightY());
  }
  

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */

}