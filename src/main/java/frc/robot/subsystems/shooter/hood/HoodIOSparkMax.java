package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import java.util.function.DoubleSupplier;

public class HoodIOSparkMax implements HoodIO {
  // A NEO motor is represented by the SparkMax class
  private final SparkMax motor;
  // Encoder object built in to the motor
  private final RelativeEncoder encoder;
  // PID controller built into the SparkMax (higher refresh rate than WPILib PID controller)
  private final SparkClosedLoopController motorController;
  // Debouncer to make sure we're at the right hood angle for long enough
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  public HoodIOSparkMax() {
    motor = new SparkMax(DeviceIDs.kTurretHood, MotorType.kBrushless);
    // Get the built-in encoder
    encoder = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    // When we're not actively running the motor, just let it rest
    config.idleMode(IdleMode.kCoast);

    config
        .encoder
        .positionConversionFactor(2 * Math.PI / HoodConstants.kGearRatio) // No absolute encoder...
        .velocityConversionFactor(2 * Math.PI / HoodConstants.kGearRatio / 60.0);

    // TODO: Tune
    config.closedLoop.feedForward.kS(
        0.015 * 12); // Static gain, or what is always applied to stay at the target
    config.closedLoop.p(1); // Corrects error

    config.closedLoop.allowedClosedLoopError(HoodConstants.kAngleTolerance, ClosedLoopSlot.kSlot0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    encoder.setPosition(0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
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
  public void setAngle(double angle) {
    // Make sure we don't go to an impossible angle
    double clampedPosition =
        MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    // Tell the motor to go to the specified angle
    motorController.setSetpoint(clampedPosition, ControlType.kPosition);
  }

  @Override
  public void setOpenLoop(double output) {
    motor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
