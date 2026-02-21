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
  public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDBufferLength);
  
  //region enum_setup
  public enum LEDMode { //temporary states to be changed later
    //#region param_desc
    /*
    param 1: light type
    param 2: color
    param 3: color2
    param 4: period 1
    param 5: period 2
    */
    //#endregion param_desc
    FLASH_WHITE("FLASH", Color.kWhite),
    SOLID_RED("SOLID", Color.kBlue),
    RAINBOW_CHROMA("RAINBOWCHROMA", null),
    OFF("OFF", null);

    private final String LEDtype;
    private final Color LEDcolor;

    private LEDMode(String _type, Color _color) {
      this.LEDtype = _type;
      this.LEDcolor = _color;
    }

    public String getType() {
      return LEDtype;
    }

    public Color getColor() {
      return LEDcolor;
    }
  }
  //#endregion enum_setup

  public LEDMode _curretstart;

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

  /*public void flashColor(Color color) {
    LEDPattern flashBase = LEDPattern.solid(color);
    LEDPattern flashAsymmetric = flashBase.blink(Seconds.of(2), Seconds.of(1));
  }*/

  public void Start(){
    System.out.print(_curretstart.getType());
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