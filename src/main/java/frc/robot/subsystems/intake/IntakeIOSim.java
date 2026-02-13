package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private final DCMotor pivotGearbox = DCMotor.getNEO(1);
  private final DCMotor wheelGearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim pivotSim;
  private final DCMotorSim wheelSim;

  // private final PIDController pid = new PIDController(1, 0, 0,
  // Constants.kLoopPeriodSeconds);

  private double pivotAppliedVolts = 0.0;
  private double wheelAppliedVolts = 0.0;

  public IntakeIOSim() {
    pivotSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            pivotGearbox, 0.025, IntakeConstants.kPivotMotorGearRatio),
        pivotGearbox);

    wheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            wheelGearbox, 0.025, IntakeConstants.kRollerMotorGearRatio),
        wheelGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
    wheelAppliedVolts = MathUtil.clamp(wheelAppliedVolts, -12.0, 12.0);

    pivotSim.setInputVoltage(pivotAppliedVolts);
    pivotSim.update(0.02);

    wheelSim.setInputVoltage(wheelAppliedVolts);
    wheelSim.update(0.02);

    inputs.pivotPositionRad = Units.rotationsToRadians(pivotSim.getAngularPositionRotations());
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotSim.getAngularVelocityRPM());

    inputs.wheelPositionRad = Units.rotationsToRadians(wheelSim.getAngularPositionRotations());
    inputs.wheelVelocityRadPerSec = Units.rotationsToRadians(wheelSim.getAngularVelocityRPM());
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotAppliedVolts = 12 * speed;
  }

  @Override
  public void setWheelSpeed(double speed) {
    wheelAppliedVolts = 12 * speed;
  }
}
