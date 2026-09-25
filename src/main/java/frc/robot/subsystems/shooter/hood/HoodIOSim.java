package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

public class HoodIOSim implements HoodIO {
  // Use the Neo550 motor simulation class
  private final DCMotor gearbox = DCMotor.getNeo550(1);
  private final SingleJointedArmSim sim;

  private final PIDController pid = new PIDController(1.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);

  private double appliedVolts = 0.0;

  public HoodIOSim() {
    // The hood behaves somewhat like an arm, so we can use the arm class to get a more accurate
    // simulation
    sim =
        new SingleJointedArmSim(
            gearbox,
            HoodConstants.kGearRatio,
            0.025,
            Units.inchesToMeters(7),
            HoodConstants.kMinAngleRad,
            HoodConstants.kMaxAngleRad,
            true,
            0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Limit the amount of voltage applied to the motor
    double volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(volts);
    sim.update(0.02); // Move the simulation forward by 20ms

    // Update inputs
    inputs.connected = true;
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setAngle(double angle) {
    // Make sure we don't try to go to an impossible angle
    angle = MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    // Use the PID controller to calculate how much voltage we need to apply to get to the target
    appliedVolts = pid.calculate(sim.getAngleRads(), angle);
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
