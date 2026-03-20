// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /**
   * Command to run the pivot
   *
   * @return runs the pivot at a speed on every iteration until end when it stops the running
   */
  public Command deployOpenLoop() {
    return Commands.runEnd(
        () -> io.setPivotSpeed(IntakeConstants.kPivotMotorSpeed),
        () -> io.setPivotSpeed(0.0),
        this);
  }

  /**
   * Command to run the pivot back
   *
   * @return runs the pivot at a speed on every iteration until end when it stops the running
   */
  public Command retractOpenLoop() {
    return Commands.runEnd(
        () -> io.setPivotSpeed(-IntakeConstants.kPivotMotorSpeed),
        () -> io.setPivotSpeed(0.0),
        this);
  }

  public Command deployPosition() {
    return Commands.run(
        () -> io.setPivotPosition(IntakeConstants.kExtensionPositionRotations), this);
  }

  public Command retractPosition() {
    return Commands.run(() -> io.setPivotPosition(0), this);
  }

  /**
   * Command to run the feeder
   *
   * @return runs the feeder at a speed on every iteration until end when it stops the running
   */
  public Command intake() {
    return Commands.runEnd(
        () -> io.setWheelSpeed(IntakeConstants.kRollerMotorSpeed),
        () -> io.setWheelSpeed(0.0),
        this);
  }

  public Command intakeSignificantlyFaster() {
    return Commands.runEnd(
        () -> io.setWheelSpeed(IntakeConstants.kRollerMotorSpeed),
        () -> io.setWheelSpeed(0.0),
        this);
  }

  /**
   * Command to run the feeder backward
   *
   * @return runs the feeder at a speed on every iteration until end when it stops the running
   */
  public Command outtake() {
    return Commands.runEnd(
        () -> io.setWheelSpeed(-IntakeConstants.kRollerMotorSpeed),
        () -> io.setWheelSpeed(0.0),
        this);
  }
  // potential sequences for commands in future

  // public Command extendArmSequence() {
  // return Commands.run(() -> runArm(.5), this)
  // .andThen(Commands.waitUntil())
  // .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  // public Command retractArmSequence() {
  // return Commands.run(() -> runArm(-0.5), this)
  // .andThen(Commands.waitUntil())
  // .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  // public Command runFeederSequence() {
  // return Commands.run(() -> runFeeder(.5), this)
  // .andThen(Commands.waitUntil())
  // .finallyDo(Commands.runOnce(() -> runArm(0)));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
