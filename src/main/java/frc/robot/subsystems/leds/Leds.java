package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

/**
 * Leds subsystem manages all LED lights on the robot. Much of this code is based on Team 6328 --
 * thank you!
 */
public class Leds extends SubsystemBase {
  // Only have one instance of the LED subsystem that all subsystems access
  private static final Leds instance = new Leds();

  // Subsystems can use Leds.getInstance() to access the subsystem instead of needing to pass the
  // LED subsystem as a parameter
  public static Leds getInstance() {
    return instance;
  }

  // WPILib LED objects
  private final AddressableLED leds = new AddressableLED(LedConstants.kPort);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedConstants.kFullLength);

  // Section record is used to represent specific strips of LEDs on the robot
  public record Section(int start, int end) {}

  // This was originally filled with different enums representing different LED strips
  // Because we eventually took them all off and only used one strip this became redundant, but we
  // just never removed it
  public enum LedSection {
    ALL(new Section(0, LedConstants.kFullLength - 1));

    private final Section section;

    LedSection(Section section) {
      this.section = section;
    }

    public Section getSection() {
      return section;
    }
  }

  private Leds() {
    leds.setLength(buffer.getLength());
    // Set the specific pixel values to the buffer and then apply the buffer to the AddressableLED
    // object
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    if (RobotState.isAutonomous()) {
      // LED pattern for auto (rainbow!)
      rainbow(LedSection.ALL, LedConstants.kRainbowCycleLength, LedConstants.kRainbowDuration);
    } else if (RobotState.isDisabled()) {
      solidRGB(LedSection.ALL, 0, 255, 0);
    } else {
      // American flag during teleop
      stripes(LedSection.ALL, List.<Color>of(Color.kRed, Color.kWhite, Color.kBlue), 5, 1);
    }
    // solid(LedSection.TOP_LEFT_TURRET, Color.kLimeGreen);
    // solid(LedSection.BOTTOM_LEFT_TURRET, Color.kYellow);
    // solid(LedSection.BOTTOM_RIGHT_TURRET, Color.kSkyBlue);
    leds.setData(buffer);
  }

  /** Sets the given LED section to the specified color (represented by the WPILib class). */
  public void solid(LedSection section, Color color) {
    Section s = section.getSection();
    for (int i = s.start(); i < s.end(); i++) {
      buffer.setLED(i, color);
    }
  }

  /** Sets the given LED section to the specified color (represented by R, G, and B values). */
  public void solidRGB(LedSection section, int r, int g, int b) {
    Section s = section.getSection();
    for (int i = s.start(); i < s.end(); i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  /** Flashes between the two specified colors at the specified duration. */
  public void strobe(LedSection section, Color c1, Color c2, double duration) {
    boolean useFirst = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    solid(section, useFirst ? c1 : c2);
  }

  /** Slowly fades between the two specified colors at the specified duration. */
  public void breath(LedSection section, Color c1, Color c2, double duration) {
    double x = ((Timer.getTimestamp() % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;

    Color mixed =
        new Color(
            c1.red * (1 - ratio) + c2.red * ratio,
            c1.green * (1 - ratio) + c2.green * ratio,
            c1.blue * (1 - ratio) + c2.blue * ratio);

    solid(section, mixed);
  }

  /** Makes a cycling LED pattern that rotates through at the specified duration. */
  public void rainbow(LedSection section, double cycleLength, double duration) {
    Section s = section.getSection();
    double baseHue = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double huePerLed = 180.0 / cycleLength;

    for (int i = s.start(); i < s.end(); i++) {
      int hue = (int) ((baseHue + huePerLed * (i - s.start())) % 180);
      buffer.setHSV(i, hue, 255, 255);
    }
  }

  /** Makes a wave pattern with the two specified colors that cycles at the specified duration. */
  public void wave(LedSection section, Color c1, Color c2, double cycleLength, double duration) {
    Section s = section.getSection();
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiff = (2.0 * Math.PI) / cycleLength;

    for (int i = s.start(); i < s.end(); i++) {
      double ratio = (Math.pow(Math.sin(x), LedConstants.kWaveExponent) + 1.0) / 2.0;

      Color mixed =
          new Color(
              c1.red * (1 - ratio) + c2.red * ratio,
              c1.green * (1 - ratio) + c2.green * ratio,
              c1.blue * (1 - ratio) + c2.blue * ratio);

      buffer.setLED(i, mixed);
      x += xDiff;
    }
  }

  /**
   * Makes a stripe pattern with the list of colors, with each stripe having the specified length
   * and shifting at the specified duration.
   */
  public void stripes(LedSection section, List<Color> colors, int stripeLength, double duration) {
    Section s = section.getSection();
    int offset =
        (int) ((Timer.getTimestamp() % duration) / duration * stripeLength * colors.size());

    for (int i = s.start(); i < s.end(); i++) {
      int index =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      buffer.setLED(i, colors.get(index));
    }
  }
}
