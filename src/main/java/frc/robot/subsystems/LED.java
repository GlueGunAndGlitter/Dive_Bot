// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(Constants.LEDConstants.LED_PORT);
    
    ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    led.setLength(ledBuffer.getLength());

    led.start();
  }

  public void setLed(Color color) {
    LEDPattern ledColor = LEDPattern.solid(color);
    ledColor.applyTo(ledBuffer);

    led.setData(ledBuffer);
  }

  public Command setLedToRedCommand() {
    return this.run(() -> setLed(Color.kRed));
  }

  public Command setLedToGreenCommand() {
    return this.run(() -> setLed(Color.kGreen));
  }

  public Command setLedToBlueCommand() {
    return this.run(() -> setLed(Color.kBlue));
  }

  public Command setLedToPinkCommand() {
    return this.run(() -> setLed(Color.kPink));
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
