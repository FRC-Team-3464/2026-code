// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.leds.LEDs.LEDMode;

import static edu.wpi.first.units.Units.Seconds;

public class LEDbutMethods extends SubsystemBase {

  public static LEDs instance;

  public static AddressableLED ledStrip = new AddressableLED(LEDConstants.kLEDStripPort);
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDBufferLength);

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

  /** Creates a new LEDbutMethods. */
  public LEDbutMethods() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public LEDPattern pattern;
  public String _type;
  public Color _color1;
  public Color _color2;
  public double _period1;
  public double _period2;
  
  
  public void setSolidColor(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
  }

  public void setSymmetricFlashing(Color color, double period) {
    LEDPattern flashBase = LEDPattern.solid(color);
    LEDPattern pattern = flashBase.blink(Seconds.of(period));
  }
  public void setAsymmetricFlashing(Color color, double period1, double period2) {
    LEDPattern flashBase = LEDPattern.solid(color);
    LEDPattern pattern = flashBase.blink(Seconds.of(period1), Seconds.of(period2));
  }

  public void setType(String _giveType){
    _type = _giveType;
  }

  public void setColor1(Color _giveColor){
    _color1 = _giveColor;
  }

  public void setColor2(Color _giveColor){
    _color2 = _giveColor;
  }

    public void setPeriod1(double _givePeriod){
    _period1 = _givePeriod;
  }

  public void setPeriod2(double _givePeriod){
    _period2 = _givePeriod;
  }

  /**
   * This method sets the value of an LED pattern object depending on the
   * parameters specified in the enum.
   */
  public LEDPattern Start() {

    System.out.print(_type);

    if (_type == "SOLID") {
      LEDPattern pattern = LEDPattern.solid(_color1);
    }

    else if (_type == "FLASH" && _period2 == 0) {
      LEDPattern flashBase = LEDPattern.solid(_color1);
      LEDPattern pattern = flashBase.blink(Seconds.of(_period1));
    }

    else if (_type == "FLASH" && _period2 != 0) {
      LEDPattern flashBase = LEDPattern.solid(_color1);
      LEDPattern pattern = flashBase.blink(Seconds.of(_period1), Seconds.of(_period2));
    }

    else if (_type == "GRADIENT") {
      LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, _color1, _color2);
    }

    else if (_type == "RAINBOWCHROMA") {
      LEDPattern pattern = LEDPattern.rainbow(0, 0);
    }

    else if (_type == "BREATH") {
      LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, _color1, _color2)
          .breathe(Seconds.of(_period1));
    }

    else if (_type == "RAINBOWBREATH") {
      LEDPattern pattern = LEDPattern.rainbow(0, 0).breathe(Seconds.of(_period1));
    } else {
      LEDPattern pattern = LEDPattern.solid(Color.kBlack);
    }
    return pattern;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledStrip.setData(ledBuffer);
  }
}
