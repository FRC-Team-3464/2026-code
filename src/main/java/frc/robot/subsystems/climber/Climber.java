// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

/**
 * This class contains the objects of the io interface and autologged class of
 * inputs, and takes the io object as a requirement. It also updates and
 * processes the values of these inputs using the Logger to be used in Advantage
 * Scope. Furthermore, it contains the commands to run the climber motor up and
 * down until the top and bottom limit switches respectively are hit, using
 * Triggers.
 * 
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Owen Biamonte
 */
public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Trigger topLimitHit = new Trigger(() -> io.topLimitHit());
  public Trigger bottomLimitHit = new Trigger(() -> io.bottomLimitHit());

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  /**
   * Runs the climber up at 0.5 speed until the top limit switch is hit, then
   * stops the motor.
   */
  public Command runClimberUp() {
    return Commands.run(() -> io.setClimberSpeed(ClimberConstants.climberSpeed), this)
        .until(topLimitHit).finallyDo(() -> io.setClimberSpeed(0));
  }

  /**
   * Runs the climber down at 0.5 speed until the bottom limit switch is hit, then
   * stops the motor.
   */
  public Command runClimberDown() {
    return Commands.run(() -> io.setClimberSpeed(-(ClimberConstants.climberSpeed)), this)
        .until(bottomLimitHit).finallyDo(() -> io.setClimberSpeed(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    // This method will be called once per scheduler run
  }

}