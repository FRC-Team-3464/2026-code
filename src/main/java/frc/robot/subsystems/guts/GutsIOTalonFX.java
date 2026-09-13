package frc.robot.subsystems.guts;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;

/**
 * This class contains all the physical objects: one motor and its corresponding encoder. It also
 * implements the default methods specified in the IO interface to set the speed of the physical
 * motor and update the input values using the encoders.
 *
 * @author Ryan Hefferon
 */
public class GutsIOTalonFX implements GutsIO {
  // Use the CAN device in the constants file
  private final TalonFX motor = new TalonFX(DeviceIDs.kGuts);

  // StatusSignals are used to help read motor stats
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  public GutsIOTalonFX() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getSupplyCurrent();

    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));

    // Update StatusSignals 50 times per second (every 20ms)
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocitySignal, voltageSignal, currentSignal);
    motor.optimizeBusUtilization();
  }

  @Override
  public void setOpenLoop(double speed) {}

  @Override
  public void updateInputs(GutsIOInputs inputs) {}
}
