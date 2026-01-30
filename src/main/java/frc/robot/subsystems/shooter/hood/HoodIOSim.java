package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOInputs;

public class HoodIOSim implements HoodIO {
  private final DCMotor gearbox = DCMotor.getNeo550(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.025, HoodConstants.kGearRatio), gearbox);

  private PIDController pid = new PIDController(1, 0, 0);

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double currentOutput = pid.calculate(sim.getAngularPositionRad() / HoodConstants.kGearRatio);
    double volts = MathUtil.clamp(currentOutput, -12.0, 12.0);

    sim.setInputVoltage(volts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.angleRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setAngle(double angle) {
    pid.setSetpoint(angle);
  }
}
