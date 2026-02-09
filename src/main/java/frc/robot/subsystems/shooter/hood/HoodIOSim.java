package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.HoodConstants;

public class HoodIOSim implements HoodIO {
  private final DCMotor gearbox = DCMotor.getNeo550(1);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          gearbox,
          HoodConstants.kGearRatio,
          0.025,
          Units.inchesToMeters(7),
          HoodConstants.kMinAngleRad,
          HoodConstants.kMaxAngleRad,
          true,
          0);

  private final PIDController pid = new PIDController(1.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);

  private double appliedVolts = 0.0;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setAngle(double angle) {
    angle = MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads(), angle), -12.0, 12.0);
  }
}
