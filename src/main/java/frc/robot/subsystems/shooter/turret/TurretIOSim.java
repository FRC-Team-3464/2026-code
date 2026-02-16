package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final DCMotorSim sim;

  private PIDController pid = new PIDController(2, 0, 0.3, Constants.kLoopPeriodSeconds);

  private double appliedVolts = 0.0;

  public TurretIOSim() {
    pid.reset();
    pid.enableContinuousInput(-Math.PI, Math.PI);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, TurretConstants.kGearRatio),
            gearbox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(volts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(Rotation2d position) {
    pid.setSetpoint(position.getRadians());
    appliedVolts = pid.calculate(sim.getAngularPositionRad());
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
