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
  private final Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private double goalRPM = 0.0;
  private boolean velocityControlActive = false;

  /**
   * Creates a flywheel subsystem using the selected hardware or simulation adapter.
   *
   * @param io adapter that reads flywheel inputs and applies motor requests
   */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  /** Refreshes flywheel inputs and updates readiness from the current velocity-control request. */
  @Override
  public void periodic() {
    // Typical IO input cycle
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    // A stopped, open-loop, or disconnected flywheel cannot be ready to shoot, even when its
    // measured speed happens to match the saved goal. The rising-edge debounce requires valid
    // closed-loop feedback to remain within tolerance before readiness becomes true.
    atGoal = atGoalDebouncer.calculate(velocityControlActive && measurementMatchesGoal(goalRPM));

    // Log the flywheel target separately
    Logger.recordOutput("Shooter/Flywheel/AtGoal", atGoal);

    // Used for PID + FF tuning
    SmartDashboard.putNumber("Flywheel Velo", getVelocity());
    SmartDashboard.putNumber("Flywheel Setpoint", goalRPM);
  }

  /**
   * Creates a command that requests a fixed flywheel velocity and stops the flywheel when the
   * command ends.
   *
   * @param velocityRPM requested flywheel velocity in rotations per minute
   * @return command requiring this flywheel subsystem
   */
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

  /**
   * Requests closed-loop velocity control and clears stale readiness when the current measurement
   * does not satisfy the new goal.
   *
   * @param velocityRPM requested flywheel velocity in rotations per minute
   */
  public void setVelocity(double velocityRPM) {
    double goalChangeRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(Math.abs(velocityRPM - goalRPM));
    boolean measurementMatchesNewGoal = measurementMatchesGoal(velocityRPM);
    // Starting velocity control needs a fresh settling period. A large target change does too;
    // smaller changes retain readiness only when the latest connected measurement also satisfies
    // the new target.
    if (!velocityControlActive
        || goalChangeRadPerSec >= FlywheelConstants.kSpeedTolerance
        || !measurementMatchesNewGoal) {
      clearReadiness();
    }

    velocityControlActive = true;
    // Save the goal RPM so we can calculate it later
    goalRPM = velocityRPM;
    // Rotations per minute -> rotations per second
    io.setVelocity(velocityRPM / 60.0);
  }

  /**
   * Runs the flywheel in open-loop mode and clears its velocity target and readiness.
   *
   * @param output motor duty cycle, where {@code -1.0} to {@code 1.0} represents reverse to forward
   *     full output
   */
  public void setOpenLoop(double output) {
    velocityControlActive = false;
    goalRPM = 0.0;
    clearReadiness();
    io.setOpenLoop(output);
  }

  /** Stops the flywheel and clears its velocity target, readiness, and debounce progress. */
  public void stop() {
    velocityControlActive = false;
    goalRPM = 0.0;
    clearReadiness();
    io.stop();
  }

  /** Clears both the published result and any partially accumulated debounce time. */
  private void clearReadiness() {
    atGoal = false;
    atGoalDebouncer.calculate(false);
  }

  /** Checks the latest connected measurement against a nonzero velocity goal. */
  private boolean measurementMatchesGoal(double velocityRPM) {
    return inputs.connected
        && velocityRPM != 0.0
        && Math.abs(
                Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM) - inputs.velocityRadPerSec)
            < FlywheelConstants.kSpeedTolerance;
  }

  /** Returns the latest measured flywheel velocity in rotations per minute. */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns true when connected velocity control has held a nonzero target within tolerance. */
  public boolean atGoal() {
    return atGoal;
  }
}
