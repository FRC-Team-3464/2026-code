// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Sends LED frames to the physical addressable strip connected to the roboRIO. */
public final class LedsIOAddressable implements LedsIO {
  private final AddressableLED leds = new AddressableLED(LedConstants.kPort);

  /** Creates the physical LED adapter without starting output. */
  public LedsIOAddressable() {}

  /** Configures the strip length, writes the initial frame, and starts physical output. */
  @Override
  public void start(AddressableLEDBuffer initialData) {
    leds.setLength(initialData.getLength());
    leds.setData(initialData);
    leds.start();
  }

  /** Writes the latest frame to the physical LED strip. */
  @Override
  public void setData(AddressableLEDBuffer data) {
    leds.setData(data);
  }
}
