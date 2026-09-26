package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  // Only the selected mode may request voltage. A saved PID target must not undo stop or open loop.
  private enum ControlMode {
    STOPPED,
    OPEN_LOOP,
    VELOCITY
  }

  // DCMotor object representing a KrakenX44 (what's used for the flywheel)
  private final DCMotor gearbox = DCMotor.getKrakenX44(1);
  private final DCMotorSim sim;

  // PID gains for the simulation
  private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

  private ControlMode controlMode = ControlMode.STOPPED;
  private double openLoopVolts = 0.0;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, FlywheelConstants.kGearRatio),
            gearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // FlywheelIO targets are RPS; DCMotorSim reports RPM, so convert feedback before PID.
    double requestedVolts =
        switch (controlMode) {
          case STOPPED -> 0.0;
          case OPEN_LOOP -> openLoopVolts;
          case VELOCITY -> pid.calculate(sim.getAngularVelocityRPM() / 60.0);
        };
    appliedVolts = MathUtil.clamp(requestedVolts, -12.0, 12.0);

    // Advance the motor model once per robot loop using the same period as the controller.
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.kLoopPeriodSeconds);

    // Update IO input values
    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocity) {
    pid.setSetpoint(velocity);
    controlMode = ControlMode.VELOCITY;
  }

  @Override
  public void setOpenLoop(double output) {
    // Open-loop output is a fraction of full voltage, not a velocity target.
    openLoopVolts = 12.0 * output;
    controlMode = ControlMode.OPEN_LOOP;
  }

  @Override
  public void stop() {
    // The model can coast, but subsequent steps must receive zero volts until a new request.
    controlMode = ControlMode.STOPPED;
    appliedVolts = 0.0;
  }
}
