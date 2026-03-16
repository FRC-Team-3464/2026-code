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

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kFalling);
  private double goalRPM = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
    Logger.recordOutput("Shooter/Flywheel/AtGoal", atGoal);

    SmartDashboard.putNumber("Flywheel Velo", getVelocity());
    SmartDashboard.putNumber("Flywheel Setpoint", goalRPM);
  }

  public Command runVelocity(double velocityRPM) {
    return Commands.startEnd(
        () -> {
          setVelocity(velocityRPM);
        },
        () -> {
          stop();
        },
        this);
  }

  public void setVelocity(double velocityRPM) {
    goalRPM = velocityRPM;
    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(
                    Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM)
                        - inputs.velocityRadPerSec)
                < FlywheelConstants.kSpeedTolerance);
    // Rotations per minute -> rotations per second
    io.setVelocity(velocityRPM / 60.0);
  }

  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  public void stop() {
    io.stop();
  }

  /**
   * Gets the current velocity of the flywheel
   *
   * @return A double representing the speed of the flywheel (in RPM).
   */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
