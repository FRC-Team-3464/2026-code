package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/** Captures the real adapter's intake requests without modeling mechanism or fuel movement. */
public class IntakeIOSim implements IntakeIO {
  private boolean pivotPositionControl;
  private double pivotTargetMotorRotations;
  private double leftPivotDutyCycle;
  private double rightPivotDutyCycle;
  private double rollerDutyCycle;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Match IndexerIOSim: voltage fields represent requested duty cycle times battery voltage,
    // not measured motor output. Leave connection, velocity, and current feedback at their
    // defaults.
    double batteryVolts = RobotController.getBatteryVoltage();
    inputs.leftPivotAppliedVolts = leftPivotDutyCycle * batteryVolts;
    inputs.rightPivotAppliedVolts = rightPivotDutyCycle * batteryVolts;
    inputs.driveAppliedVolts = rollerDutyCycle * batteryVolts;

    Logger.recordOutput("Intake/Sim/PivotPositionControl", pivotPositionControl);
    Logger.recordOutput("Intake/Sim/PivotTargetMotorRotations", pivotTargetMotorRotations);
    Logger.recordOutput("Intake/Sim/LeftPivotDutyCycle", leftPivotDutyCycle);
    Logger.recordOutput("Intake/Sim/RightPivotDutyCycle", rightPivotDutyCycle);
    Logger.recordOutput("Intake/Sim/RollerDutyCycle", rollerDutyCycle);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotPositionControl = false;
    pivotTargetMotorRotations = 0.0;
    // Preserve the independent motor requests in IntakeIOTalonFX, including its sign and scaling.
    leftPivotDutyCycle = MathUtil.clamp(speed, -1.0, 1.0);
    rightPivotDutyCycle = MathUtil.clamp(speed * -0.95, -1.0, 1.0);
  }

  @Override
  public void setWheelSpeed(double speed) {
    rollerDutyCycle = MathUtil.clamp(speed, -1.0, 1.0);
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    // REAL sends this same motor-rotation target to both pivots. These unused helpers currently
    // have zero PID/feedforward gains; record the target without inventing a position response.
    pivotPositionControl = true;
    pivotTargetMotorRotations = positionRotations;
    leftPivotDutyCycle = 0.0;
    rightPivotDutyCycle = 0.0;
  }
}
