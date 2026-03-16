package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX throatMotor = new TalonFX(DeviceIDs.kIndexer);
  private final TalonFX toungeMotor = new TalonFX(DeviceIDs.kIndexer);

  private final StatusSignal<AngularVelocity> throatVelocity;
  private final StatusSignal<AngularAcceleration> throatAcceleration;
  private final StatusSignal<Voltage> throatVoltage;
  private final StatusSignal<Current> throatCurrent;

  private final StatusSignal<AngularVelocity> toungeVelocity;
  private final StatusSignal<AngularAcceleration> toungeAcceleration;
  private final StatusSignal<Voltage> toungeVoltage;
  private final StatusSignal<Current> toungeCurrent;

  public IndexerIOTalonFX() {
    TalonFXConfiguration throatMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration toungeMotorConfig = new TalonFXConfiguration();

    throatVelocity = throatMotor.getVelocity();
    throatAcceleration = throatMotor.getAcceleration();
    throatVoltage = throatMotor.getMotorVoltage();
    throatCurrent = throatMotor.getSupplyCurrent();

    toungeVelocity = toungeMotor.getVelocity();
    toungeAcceleration = toungeMotor.getAcceleration();
    toungeVoltage = toungeMotor.getMotorVoltage();
    toungeCurrent = toungeMotor.getSupplyCurrent();

    tryUntilOk(5, () -> throatMotor.getConfigurator().apply(throatMotorConfig));
    tryUntilOk(5, () -> toungeMotor.getConfigurator().apply(toungeMotorConfig));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        throatVelocity,
        throatAcceleration,
        throatVoltage,
        throatCurrent,
        toungeVelocity,
        toungeAcceleration,
        toungeCurrent);
    ParentDevice.optimizeBusUtilizationForAll(throatMotor, toungeMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.throatConnected =
        BaseStatusSignal.refreshAll(
                throatVelocity, throatAcceleration, throatVoltage, throatCurrent)
            .isOK();
    inputs.throatVelocityRadPerSec = throatVelocity.getValue().in(RadiansPerSecond);
    inputs.throatAppliedVolts = throatAcceleration.getValueAsDouble();
    inputs.throatCurrentDrawAmps = throatCurrent.getValueAsDouble();

    inputs.toungeConnected =
        BaseStatusSignal.refreshAll(
                toungeVelocity, toungeAcceleration, toungeVoltage, toungeCurrent)
            .isOK();
    inputs.toungeVelocityRadPerSec = toungeVelocity.getValue().in(RadiansPerSecond);
    inputs.toungeAppliedVolts = toungeAcceleration.getValueAsDouble();
    inputs.toungeCurrentDrawAmps = toungeCurrent.getValueAsDouble();
  }

  @Override
  public void setThroatOpenLoop(double output) {
    throatMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  @Override
  public void setToungeOpenLoop(double output) {
    toungeMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  @Override
  public void stop() {
    throatMotor.stopMotor();
    toungeMotor.stopMotor();
  }
}
