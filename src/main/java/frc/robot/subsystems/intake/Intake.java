// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final IntakeIO io;
  private final IntakeIOAutoLogged inputs = new IntakeIOAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }
/**
 * Command to run the arm
 * @param speed runs the arm at a set speed
 * @return runs the arm at a speed on every iteration until end when it stops the running
 */
  public Command runArm(double speed) {
    return Commands.runEnd(
        () -> io.setArmSpeed(speed),
        () -> io.setArmSpeed(0.0),
        this);
  }
/**
 * Command to run the feeder
 * @param speed runs the feeder at a set speed
 * @return runs the feeder at a speed on every iteration until end when it stops the running
 */
  public Command runFeeder(double speed) {
    return Commands.runEnd(
        () -> io.setWheelSpeed(speed),
        () -> io.setWheelSpeed(0.0),
        this);
  }
//potential sequences for commands in future


  // public Command extendArmSequence() {
  //   return Commands.run(() -> runArm(.5), this)
  //       .andThen(Commands.waitUntil())
  //       .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  // public Command retractArmSequence() {
  //   return Commands.run(() -> runArm(-0.5), this)
  //       .andThen(Commands.waitUntil())
  //       .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  // public Command runFeederSequence() {
  //   return Commands.run(() -> runFeeder(.5), this)
  //       .andThen(Commands.waitUntil())
  //       .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
