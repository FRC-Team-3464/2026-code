package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final DCMotorSim sim;

  private PIDController pid = new PIDController(10, 0, 0.3, Constants.kLoopPeriodSeconds);

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
    double currentOutput = pid.calculate(sim.getAngularPositionRad());
    double volts = MathUtil.clamp(currentOutput, -12.0, 12.0);

    sim.setInputVoltage(volts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(Rotation2d position) {
    pid.setSetpoint(position.getRadians());
  }
}
