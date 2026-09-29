// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

/**
 * Hardware-free LED adapter for desktop simulation.
 *
 * <p>{@link Leds} renders the normal patterns, and this adapter publishes each completed frame to
 * WPILib's simulated addressable LED device.
 */
public final class LedsIOSim implements LedsIO {
  private final AddressableLEDSim leds = AddressableLEDSim.createForIndex(0);

  /** Creates and initializes the simulated LED controller on the configured PWM port. */
  public LedsIOSim() {
    leds.setOutputPort(LedConstants.kPort);
    leds.setInitialized(true);
  }

  /** Configures the simulated strip length, publishes its initial frame, and starts output. */
  @Override
  public void start(AddressableLEDBuffer initialData) {
    leds.setLength(initialData.getLength());
    leds.setData(toHalData(initialData));
    leds.setRunning(true);
  }

  /** Publishes the latest rendered frame to WPILib simulation. */
  @Override
  public void setData(AddressableLEDBuffer data) {
    leds.setData(toHalData(data));
  }

  private static byte[] toHalData(AddressableLEDBuffer data) {
    byte[] halData = new byte[data.getLength() * 4];
    for (int index = 0; index < data.getLength(); index++) {
      halData[index * 4] = (byte) data.getBlue(index);
      halData[index * 4 + 1] = (byte) data.getGreen(index);
      halData[index * 4 + 2] = (byte) data.getRed(index);
    }
    return halData;
  }
}
