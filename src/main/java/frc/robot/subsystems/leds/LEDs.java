// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

  public static LEDs instance;

  public static AddressableLED ledStrip = new AddressableLED(LEDConstants.kLEDStripPort);
  public static AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(LEDConstants.kLEDBufferLength);

  public enum LEDMode { // temporary states to be changed later
    FLASH_WHITE("FLASH", "WHITE"),
    SOLID_RED("SOLIDRED"),
    RAINBOW_CHROMA("RAINBOWCHROMA"),
    OFF("OFF");

    private final String LEDColor;

    private LEDMode(String color) {
      this.LEDtype = LEDtype;
    }

    public String getType() {
      return LEDtype;
    }
  }

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

  public void solidColor(Color color) {
    LEDPattern solidBase = LEDPattern.solid(color);
    solidBase.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void flashColor(Color color) {
    LEDPattern flashBase = LEDPattern.solid(color);
    LEDPattern flashAsymmetric = flashBase.blink(Seconds.of(2), Seconds.of(1));
  }

  LEDPattern rainbowBase = LEDPattern.rainbow(0, 0);

  // Fix later

  // switch(mode) {
  //   case SOLIDRED:
  //     solidColor(Color.kRed);
  //     break;
  //   case FLASHWHITE:
  //     break;
  // }

  public Command setLEDPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(ledBuffer));
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
}
