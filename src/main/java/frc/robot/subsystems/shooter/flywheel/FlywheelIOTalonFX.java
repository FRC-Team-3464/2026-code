package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  // Kraken motors are represented by TalonFX motor controller class
  private final TalonFX motor;
  // Configurations for the Kraken motors.
  private final TalonFXConfiguration motorConfig;

  // StatusSignals which obtain statistics from each motor
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  // Request that tells the motor to run at the specified velocity using the PID gains in slot 0
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public FlywheelIOTalonFX() {
    motor = new TalonFX(DeviceIDs.kTurretFlywheel);
    motorConfig =
        new TalonFXConfiguration()
            .withSlot0(FlywheelConstants.kGains)
            // .withCurrentLimits(
            //     new CurrentLimitsConfigs()
            //         .withSupplyCurrentLimit(Amps.of(50))
            //         .withStatorCurrentLimit(Amps.of(50)))
            .withMotorOutput(FlywheelConstants.kOutputConfigs);
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));

    velocitySignal = motor.getVelocity();
    accelerationSignal = motor.getAcceleration();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    // Configure all StatusSignals to update every 20ms
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocitySignal, accelerationSignal, voltageSignal, currentSignal);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // If every Status signal comes back OK, then it's connected
    inputs.connected =
        BaseStatusSignal.refreshAll(
                velocitySignal, accelerationSignal, voltageSignal, currentSignal)
            .isOK();

    // Update IO input values
    inputs.velocityRadPerSec = velocitySignal.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentDrawAmps = currentSignal.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    // Tell the motor to follow the velocity target
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setOpenLoop(double output) {
    motor.set(output);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
