package frc.robot.subsystems.intake;

/** Simulation implementation of the IntakeIO interface. */
public class IntakeIOSim implements IntakeIO {
  // // Make motor simulation objects
  // private final DCMotor pivotGearbox = DCMotor.getNEO(1);
  // private final DCMotor wheelGearbox = DCMotor.getKrakenX60(1);
  // private final DCMotorSim pivotSim;
  // private final DCMotorSim wheelSim;

  // // private final PIDController pid = new PIDController(1, 0, 0,
  // // Constants.kLoopPeriodSeconds);

  // private double pivotAppliedVolts = 0.0;
  // private double wheelAppliedVolts = 0.0;

  // public IntakeIOSim() {
  //   // Make simulation objects that will represent the motor systems
  //   pivotSim =
  //       new DCMotorSim(
  //           LinearSystemId.createDCMotorSystem(
  //               pivotGearbox, 0.025, IntakeConstants.kPivotMotorGearRatio),
  //           pivotGearbox);

  //   wheelSim =
  //       new DCMotorSim(
  //           LinearSystemId.createDCMotorSystem(
  //               wheelGearbox, 0.025, IntakeConstants.kRollerMotorGearRatio),
  //           wheelGearbox);
  // }

  // @Override
  // public void updateInputs(IntakeIOInputs inputs) {
  //   // Make sure that we don't send more than 12 volts to the motors
  //   pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
  //   wheelAppliedVolts = MathUtil.clamp(wheelAppliedVolts, -12.0, 12.0);

  //   pivotSim.setInputVoltage(pivotAppliedVolts);
  //   pivotSim.update(0.02);

  //   wheelSim.setInputVoltage(wheelAppliedVolts);
  //   wheelSim.update(0.02);

  //   // Update IO input values
  //   inputs.pivotPositionRad = Units.rotationsToRadians(pivotSim.getAngularPositionRotations());
  //   inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotSim.getAngularVelocityRPM());

  //   inputs.wheelPositionRad = Units.rotationsToRadians(wheelSim.getAngularPositionRotations());
  //   inputs.wheelVelocityRadPerSec = Units.rotationsToRadians(wheelSim.getAngularVelocityRPM());
  // }

  // @Override
  // public void setPivotSpeed(double speed) {
  //   // If the maximum is 12 volts, and the value of speed is <=1, then multiply by 12 to get the
  // percentage of max voltage
  //   pivotAppliedVolts = 12 * speed;
  // }

  // @Override
  // public void setWheelSpeed(double speed) {
  //   wheelAppliedVolts = 12 * speed;
  // }
}
