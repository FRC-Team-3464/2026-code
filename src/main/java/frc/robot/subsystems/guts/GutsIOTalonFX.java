package frc.robot.subsystems.guts;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;

/**
 * This class contains all of the physical objects: one motor and its corresponding encoder. It also
 * implements the default methods specified in the IO interface to set the speed of the physical
 * motor and update the input values using the encoders.
 *
 * @author Ryan Hefferon
 */
public class GutsIOTalonFX implements GutsIO {

  private final TalonFX motor = new TalonFX(DeviceIDs.kGuts);
  
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  public GutsIOTalonFX() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getSupplyCurrent();
    
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));

    BaseStatusSignal.setUpdateFrequencyForAll(50, velocitySignal, voltageSignal, currentSignal);
    motor.optimizeBusUtilization();
  }

  @Override
  public void setOpenLoop(double speed) {
  }

  @Override
  public void updateInputs(GutsIOInputs inputs) {

  }
}
