package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Leds extends SubsystemBase {

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) instance = new Leds();
    return instance;
  }

  private final AddressableLED leds = new AddressableLED(LedConstants.kPort);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedConstants.kFullLength);

  public record Section(int start, int end) {}

  public enum LedSection {
    ALL(new Section(0, LedConstants.kFullLength)),
    ALL_LEFT(
        new Section(
            0, LedConstants.kLeftTurretBottomLength + LedConstants.kLeftTurretTopLength - 1)),
    ALL_RIGHT(
        new Section(
            LedConstants.kLeftTurretBottomLength + LedConstants.kLeftTurretTopLength,
            LedConstants.kFullLength)),
    BOTTOM_LEFT_TURRET(new Section(0, LedConstants.kLeftTurretBottomLength - 1)),
    TOP_LEFT_TURRET(
        new Section(
            LedConstants.kLeftTurretBottomLength,
            LedConstants.kLeftTurretBottomLength + LedConstants.kLeftTurretTopLength - 1)),
    TOP_RIGHT_TURRET(
        new Section(
            LedConstants.kLeftTurretBottomLength + LedConstants.kLeftTurretTopLength,
            LedConstants.kLeftTurretBottomLength
                + LedConstants.kLeftTurretTopLength
                + LedConstants.kRightTurretTopLength
                - 1)),
    BOTTOM_RIGHT_TURRET(
        new Section(
            LedConstants.kLeftTurretBottomLength
                + LedConstants.kLeftTurretTopLength
                + LedConstants.kRightTurretTopLength,
            LedConstants.kFullLength));

    private final Section section;

    private LedSection(Section section) {
      this.section = section;
    }

    public Section getSection() {
      return section;
    }
  }

  public Leds() {
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // Default pattern (change this however you want)
    solid(LedSection.ALL, Color.kAquamarine);
  }

  public void solid(LedSection section, Color color) {
    for (int i = section.section.start(); i < section.section.end(); i++) {
      buffer.setLED(i, color);
    }
  }

  public void strobe(LedSection section, Color c1, Color c2, double duration) {
    boolean useFirst = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    solid(section, useFirst ? c1 : c2);
  }

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

  public void rainbow(LedSection section, double cycleLength, double duration) {
    double baseHue = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double huePerLed = 180.0 / cycleLength;

    for (int i = section.section.start(); i < section.section.end(); i++) {
      int hue = (int) ((baseHue + huePerLed * (i - section.section.start())) % 180);
      buffer.setHSV(i, hue, 255, 255);
    }
  }

  public void wave(LedSection section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiff = (2.0 * Math.PI) / cycleLength;

    for (int i = section.section.start(); i < section.section.end(); i++) {
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

  public void stripes(LedSection section, List<Color> colors, int stripeLength, double duration) {
    int offset =
        (int) ((Timer.getTimestamp() % duration) / duration * stripeLength * colors.size());

    for (int i = section.section.start(); i < section.section.end(); i++) {
      int index =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      buffer.setLED(i, colors.get(index));
    }
  }
}
