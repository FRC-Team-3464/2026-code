package frc.robot.subsystems.guts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GutsIOSim implements GutsIO {
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final DCMotorSim sim;

  // private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

  private double appliedVolts = 0.0;

  public GutsIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, GutsConstants.kGutMotorGearRatio),
            gearbox);
  }

  @Override
  public void updateInputs(GutsIOInputs inputs) {

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRotations();
    inputs.velocityRadPerSec = sim.getAngularVelocityRPM();
  }

  @Override
  public void setGutMotorSpeed(double speed) {
    appliedVolts = 12 * speed;
  }
}
