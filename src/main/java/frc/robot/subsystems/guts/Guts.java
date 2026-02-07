// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.guts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class updates and stores the values of the inputs periodically, and
 * contains commands to run the gut motors forward and backward.
 * @author Ryan Hefferon
 */
public class Guts extends SubsystemBase {

  public final GutsIO io;
  public GutsIOInputsAutoLogged inputs = new GutsIOInputsAutoLogged();

  /** Creates a new Guts. */
  public Guts(GutsIO io) {
    this.io = io;
  }

  /** Runs the left gut motor forward at 0.5 speed, then stops it when finished. */
  public Command runLeftGutForward() {
    return Commands.runEnd(
        () -> io.setLeftGutMotorSpeed(0.5),
        () -> io.setLeftGutMotorSpeed(0),
        this);
  }

  /** Runs the right gut motor forward at 0.5 speed, then stops it when finished. */
  public Command runRightGutForward() {
    return Commands.runEnd(
        () -> io.setRightGutMotorSpeed(0.5),
        () -> io.setRightGutMotorSpeed(0),
        this);
  }

  /** Runs the left gut motor backward at 0.5 speed, then stops it when finished. */
  public Command runLeftGutBackward() {
    return Commands.runEnd(
        () -> io.setLeftGutMotorSpeed(-0.5),
        () -> io.setLeftGutMotorSpeed(0),
        this);
  }

  /** Runs the right gut motor backward at 0.5 speed, then stops it when finished. */
  public Command runRightGutBackward() {
    return Commands.runEnd(
        () -> io.setRightGutMotorSpeed(-0.5),
        () -> io.setRightGutMotorSpeed(0),
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Guts", inputs);
    // This method will be called once per scheduler run
  }
}