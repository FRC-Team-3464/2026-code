// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Defines how the LED subsystem sends rendered pixel data to the selected runtime adapter. */
public interface LedsIO {
  /** Initializes the adapter with the first complete LED frame. */
  default void start(AddressableLEDBuffer initialData) {}

  /** Sends the latest complete LED frame to the adapter. */
  default void setData(AddressableLEDBuffer data) {}
}
