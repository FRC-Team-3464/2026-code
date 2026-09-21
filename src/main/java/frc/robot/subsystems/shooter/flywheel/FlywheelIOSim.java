package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  // DCMotor object representing a KrakenX44 (what's used for the flywheel)
  private final DCMotor gearbox = DCMotor.getKrakenX44(1);
  private final DCMotorSim sim;

  // PID gains for the simulation
  private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, FlywheelConstants.kGearRatio),
            gearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Calculate the amount of power to apply using the PID controller
    double currentOutput = pid.calculate(sim.getAngularVelocityRPM());
    // Make sure that we don't send more than 12 volts to the motors
    appliedVolts = MathUtil.clamp(currentOutput, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    // Update IO input values
    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocity) {
    pid.setSetpoint(velocity);
  }

  @Override
  public void setOpenLoop(double output) {
    // If the maximum is 12 volts, and the absolute value of speed is <=1, then multiply by 12 to
    // get the percentage of max voltage
    appliedVolts = 12.0 * output;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
