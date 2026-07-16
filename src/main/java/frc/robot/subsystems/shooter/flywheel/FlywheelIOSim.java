package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor gearbox = DCMotor.getKrakenX44(1);
  private final DCMotorSim sim;

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
    double currentOutput = pid.calculate(sim.getAngularVelocityRPM());
    appliedVolts = MathUtil.clamp(currentOutput, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

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
    appliedVolts = 12.0 * output;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
