package frc.robot.subsystems.shooter.turret;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants.DeviceIDs;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import java.util.function.DoubleSupplier;

public class TurretIOSparkMax implements TurretIO {
  private final SparkMax motor;
  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController motorController;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public TurretIOSparkMax() {
    motor = new SparkMax(DeviceIDs.kTurretAzimuth, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    motorController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    config.closedLoop.positionWrappingEnabled(true).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.softLimit.reverseSoftLimitEnabled(false).forwardSoftLimitEnabled(false);

    config.closedLoop.feedForward.kS(0.025 * 12);
    config.closedLoop.p(0.1);
    config.closedLoop.d(0.01);
    config.closedLoop.allowedClosedLoopError(
        TurretConstants.kAngleTolerance, ClosedLoopSlot.kSlot0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentDrawAmps = value);
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case CLOSED_LOOP -> {
        double clampedPosition =
            MathUtil.clamp(
                outputs.closedLoopTarget.getRadians(),
                TurretConstants.kMinTurretAngleRad,
                TurretConstants.kMaxTurretAngleRad);

        motorController.setSetpoint(clampedPosition, ControlType.kPosition);
      }
      case OPEN_LOOP -> {
        motor.set((MathUtil.clamp(outputs.openLoopOutput, -1.0, 1.0)));
      }
    }
  }
}
