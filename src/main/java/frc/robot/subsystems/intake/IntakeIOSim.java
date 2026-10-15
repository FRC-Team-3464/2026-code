package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Records intake requests and approximates sliding travel for visualization, not fuel physics. */
public class IntakeIOSim implements IntakeIO {
  // Provisional display timing: full travel at the normal deployment duty cycle.
  private static final double FULL_TRAVEL_SECONDS = 2.0;
  private double extensionFraction;
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
    boolean enabled = DriverStation.isEnabled();
    if (!enabled) {
      // Disabled mechanisms hold their display position; stale requests must not resume on enable.
      leftPivotDutyCycle = 0.0;
      rightPivotDutyCycle = 0.0;
      rollerDutyCycle = 0.0;
    }
    double batteryVolts = enabled ? RobotController.getBatteryVoltage() : 0.0;
    if (!pivotPositionControl && batteryVolts > 0.0) {
      extensionFraction =
          MathUtil.clamp(
              extensionFraction
                  + (leftPivotDutyCycle / IntakeConstants.kPivotMotorSpeed)
                      * Constants.kLoopPeriodSeconds
                      / FULL_TRAVEL_SECONDS,
              0.0,
              1.0);
    }
    inputs.simExtensionFraction = extensionFraction;
    inputs.simExtensionAvailable = true;
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
