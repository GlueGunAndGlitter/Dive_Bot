package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private static final int LED_PORT = 2
    ; // Change this to your actual PWM port
    private static final int LED_LENGTH = 30; // Change this to match your LED strip length

    public LEDSubsystem() {
        led = new AddressableLED(LED_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(ledBuffer.getLength());

        // Set the default color to pink
        setColor(255, 20, 147); // RGB for Pink

        led.setData(ledBuffer);
        led.start();
    }

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        // Called once per scheduler run, can add animations here if needed
    }
}
