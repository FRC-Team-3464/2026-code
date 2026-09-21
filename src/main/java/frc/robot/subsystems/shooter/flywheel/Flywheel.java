// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // IO representation + inputs for the flywheel
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Boolean representing if the flywheel is at its target RPM
  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private double goalRPM = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Typical IO input cycle
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    // Uses the debouncer to determine if the flywheel has been at the goal RPM for enough time
    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(goalRPM) - inputs.velocityRadPerSec)
                < FlywheelConstants.kSpeedTolerance);

    // Log the flywheel target separately
    Logger.recordOutput("Shooter/Flywheel/AtGoal", atGoal);

    // Used for PID + FF tuning
    SmartDashboard.putNumber("Flywheel Velo", getVelocity());
    SmartDashboard.putNumber("Flywheel Setpoint", goalRPM);
  }

  public Command runVelocity(double velocityRPM) {
    return Commands.startEnd(
        () -> {
          // At the start of the command, set the target velocity
          setVelocity(velocityRPM);
        },
        () -> {
          // Stop the flywheel when the command ends
          stop();
        },
        this); // Reference to the flywheel subsystem instance
  }

  public void setVelocity(double velocityRPM) {
    // Save the goal RPM so we can calculate it later
    goalRPM = velocityRPM;
    // Rotations per minute -> rotations per second
    io.setVelocity(velocityRPM / 60.0);
  }

  /** Runs the flywheel motor at the specified open loop value. */
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  /** Stops the flywheel motor. */
  public void stop() {
    io.stop();
    goalRPM = 0.0;
  }

  /**
   * Gets the current velocity of the flywheel
   *
   * @return A double representing the speed of the flywheel (in RPM).
   */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns true if the flywheel is at its target RPM. */
  public boolean atGoal() {
    return atGoal;
  }
}
