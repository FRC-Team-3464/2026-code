// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Hardware-free LED adapter for desktop simulation.
 *
 * <p>{@link Leds} renders the normal patterns, and this adapter publishes each completed frame to
 * WPILib's simulated addressable LED device.
 */
public final class LedsIOSim implements LedsIO {
  // On desktop, WPILib routes this device through its simulated HAL. Writing the normal buffer
  // exercises the public output API without duplicating WPILib's internal pixel byte layout.
  // AddressableLEDSim can observe this device by PWM channel in diagnostics.
  private final AddressableLED leds = new AddressableLED(LedConstants.kPort);

  /** Creates and initializes the simulated LED controller on the configured PWM port. */
  public LedsIOSim() {}

  /** Configures the simulated strip length, publishes its initial frame, and starts output. */
  @Override
  public void start(AddressableLEDBuffer initialData) {
    leds.setLength(initialData.getLength());
    leds.setData(initialData);
    leds.start();
  }

  /** Publishes the latest rendered frame to WPILib simulation. */
  @Override
  public void setData(AddressableLEDBuffer data) {
    leds.setData(data);
  }
}
