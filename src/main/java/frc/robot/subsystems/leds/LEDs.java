// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import java.util.random.RandomGenerator.LeapableGenerator;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * This class contains an enum which specifies all the possible LED states with
 * parameters, creates objects for an LED strip and buffer, sets the LED strip
 * based on the buffer, a method which implements all these parameters, and a
 * command which calls the method.
 * 
 * @author Ryan Hefferon
 * @author Julien Precourt
 */
public class LEDs extends SubsystemBase {

  public static LEDs instance;

  public static AddressableLED ledStrip = new AddressableLED(LEDConstants.kLEDStripPort);
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDBufferLength);

  /**
   * This enum contains parameters for the type of LED effect, two colors, and two
   * time periods. It then specifies possible states with appropriate parameters,
   * and contains a method to initialize all the parameters. It also has five
   * methods to return the values of each parameter.
   */
  public enum LEDMode { // temporary states to be changed later
    // if only one period of time is specified, the default will be blinking
    // both on and off for this amount of time

    // #region param_desc
    /*
     * param 1: light type
     * param 2: color
     * param 3: color2
     * param 4: period 1
     * param 5: period 2
     */
    // #endregion param_desc
    FLASH_WHITE("FLASH", Color.kWhite, null, 1.5, 0),
    FLASH_YELLOW("FLASH", Color.kYellow, null, 0, 0),
    FLASH_RED("FLASH", Color.kRed, null, 0, 0),
    FLASH_BLUE("FLASH", Color.kBlue, null, 0, 0),
    FLASH_PINK("FLASH", Color.kPink, null, 0, 0),
    FLASH_GREEN("FLASH", Color.kGreen, null, 0, 0),
    FLASH_PURPLE("FLASH", Color.kPurple, null, 0, 0),
    FLASH_ORANGE("FLASH", Color.kOrange, null, 0, 0),
    SOLID_RED("SOLID", Color.kRed, null, 0, 0),
    SOLID_WHITE("SOLID", Color.kWhite, null, 0, 0),
    SOLID_GREEN("SOLID", Color.kGreen, null, 0, 0),
    SOLID_BLUE("SOLID", Color.kBlue, null, 0, 0),
    SOLID_YELLOW("SOLID", Color.kYellow, null, 0, 0),
    SOLID_PINK("SOLID", Color.kPink, null, 0, 0),
    SOLID_PURPLE("SOLID", Color.kPurple, null, 0, 0),
    SOLID_ORANGE("SOLID", Color.kOrange, null, 0, 0),
    RED_AND_PURPLE_GRADIENT("GRADIENT", Color.kRed, Color.kPurple, 0, 0),
    ORANGE_AND_GREEN_GRADIENT("GRADIENT", Color.kOrange, Color.kGreen, 0, 0),
    SCHOOL_COLORS_GRADIENT("GRADIENT", Color.kYellow, Color.kLightBlue, 0, 0),
    RED_AND_PURPLE_BREATH("BREATH", Color.kRed, Color.kPurple, 0, 0),
    ORANGE_AND_GREEN_BREATH("BREATH", Color.kOrange, Color.kGreen, 0, 0),
    SCHOOL_COLORS_BREATH("BREATH", Color.kYellow, Color.kLightBlue, 1.5, 0),
    RAINBOW_BREATH("RAINBOWBREATH", null, null, 1.5, 0),
    RAINBOW_CHROMA("RAINBOWCHROMA", null, null, 0, 0),
    BREATH("BREATH", Color.kRed, Color.kBlue, 0, 0),
    OFF("OFF", null, null, 0, 0);

    private final String LEDtype;
    private final Color LEDcolor1;
    private final Color LEDcolor2;
    private final double LEDperiod1;
    private final double LEDperiod2;

    private LEDMode(String _type, Color _color1, Color _color2, double _period1Sec, double _period2Sec) {
      this.LEDtype = _type;
      this.LEDcolor1 = _color1;
      this.LEDcolor2 = _color2;
      this.LEDperiod1 = _period1Sec;
      this.LEDperiod2 = _period2Sec;

    }

    public String getType() {
      return LEDtype;
    }

    // #endregion enum_setup

    public LEDMode _curretstart;

    public Color getColor1() {
      return LEDcolor1;
    }

    public Color getColor2() {
      return LEDcolor2;
    }

    public double getPeriod1() {
      return LEDperiod1;
    }

    public double getPeriod2() {
      return LEDperiod2;
    }

  }
  // #endregion enum_setup

  /** This is the enum object. */
  public LEDMode _curretStart;

  /**
   * Creates a new LED subsystem object if there is not already one.
   * 
   * @return The new LED subsystem object.
   */
  public static LEDs getInstance(LEDMode mode) {
    if (instance == null) {
      instance = new LEDs(mode);
    }
    return instance;
  }

  /** Creates a new LEDs. */
  public LEDs(LEDMode mode) {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public LEDPattern pattern;

  /**
   * This method sets the value of an LED pattern object depending on the
   * parameters specified in the enum.
   */
  public LEDPattern Start() {

    String type = _curretStart.getType();
    Color color1 = _curretStart.getColor1();
    Color color2 = _curretStart.getColor2();
    double period1 = _curretStart.getPeriod1();
    double period2 = _curretStart.getPeriod2();

    System.out.print(type);

    if (type == "SOLID") {
      LEDPattern pattern = LEDPattern.solid(color1);
    }

    else if (type == "FLASH" && period2 == 0) {
      LEDPattern flashBase = LEDPattern.solid(color1);
      LEDPattern pattern = flashBase.blink(Seconds.of(period1));
    }

    else if (type == "FLASH" && period2 != 0) {
      LEDPattern flashBase = LEDPattern.solid(color1);
      LEDPattern pattern = flashBase.blink(Seconds.of(period1), Seconds.of(period2));
    }

    else if (type == "GRADIENT") {
      LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2);
    }

    else if (type == "RAINBOWCHROMA") {
      LEDPattern pattern = LEDPattern.rainbow(0, 0);
    }

    else if (type == "BREATH") {
      LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2)
          .breathe(Seconds.of(period1));
    }

    else if (type == "RAINBOWBREATH") {
      LEDPattern pattern = LEDPattern.rainbow(0, 0).breathe(Seconds.of(period1));
    } else {
      LEDPattern pattern = LEDPattern.solid(Color.kBlack);
    }
    return pattern;
  }

  /**
   * This command implements the start method above and sets data to the LED
   * buffer depending on what the LED pattern is.
   */
  public Command setLEDPattern() {
    LEDPattern pattern = Start();
    return run(() -> pattern.applyTo(ledBuffer));
  }

  @Override
  public void periodic() {

    if (DriverStation.isEStopped()) {
      LEDPattern eStoppedPattern = LEDPattern.solid(Color.kRed);
      eStoppedPattern.applyTo(ledBuffer);
    }

    ledStrip.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
}