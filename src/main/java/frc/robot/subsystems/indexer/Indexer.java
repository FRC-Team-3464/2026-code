// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The Indexer subsystem controls the feeding of the fuel from the hopper to the shooter. */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update the stats for the subsystem
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command index() {
    // Make a command which has two lambda expressions
    // The first expression will run immediately when the command starts
    // The second expression will run immediately when the command ends
    // We don't need a run command because we only need to set speeds once
    return Commands.startEnd(
        () -> {
          // Set the throat and tongue motor to their respective speeds
          io.setThroatOpenLoop(-IndexerConstants.kGutsMotorSpeed);
          io.setTongueOpenLoop(IndexerConstants.kGutsMotorSpeed);
        },
        () -> {
          // Stop the motor
          io.stop();
        },
        this); // Subsystem requirements
  }

  public Command indexReverse() {
    // See above documentation for an explanation of this method
    // It's exactly the same thing, just running in the other direction
    return Commands.startEnd(
        () -> {
          io.setThroatOpenLoop(IndexerConstants.kGutsMotorSpeed);
          io.setTongueOpenLoop(-IndexerConstants.kGutsMotorSpeed);
        },
        () -> {
          io.stop();
        },
        this);
  }

  /** Sets the throat motor to a specified speed. */
  public void setThroatOpenLoop(double output) {
    io.setThroatOpenLoop(output);
  }

  /** Sets the tongue motor to a specified speed. */
  public void setTongueOpenLoop(double output) {
    io.setTongueOpenLoop(output);
  }

  /** Stops all indexer motors. */
  public void stop() {
    io.stop();
  }
}
