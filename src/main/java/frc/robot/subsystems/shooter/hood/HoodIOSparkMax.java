package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import java.util.function.DoubleSupplier;

public class HoodIOSparkMax implements HoodIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController motorController;
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public HoodIOSparkMax(ShooterSide side) {
    motor =
        new SparkMax(
            side == ShooterSide.LEFT ? DeviceIDs.kLeftTurretHood : DeviceIDs.kRightTurretHood,
            MotorType.kBrushless);
    encoder = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    config.inverted(side == ShooterSide.RIGHT);

    config
        .encoder
        .positionConversionFactor(2 * Math.PI / HoodConstants.kGearRatio) // No absolute encoder...
        .velocityConversionFactor(2 * Math.PI / HoodConstants.kGearRatio / 60.0);

    config.closedLoop.feedForward.kS(0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0));
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
    double clampedPosition =
        MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

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
