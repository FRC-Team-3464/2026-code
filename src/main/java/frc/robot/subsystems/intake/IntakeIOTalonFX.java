package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;

/** Hardware implementation of the IntakeIO interface. */
public class IntakeIOTalonFX implements IntakeIO {
  // Kraken motors are represented by TalonFX motor controller class
  private TalonFX leftPivotMotor = new TalonFX(DeviceIDs.kLeftIntakePivot);
  private TalonFX rightPivotMotor = new TalonFX(DeviceIDs.kRightIntakePivot);
  private TalonFX driveMotor = new TalonFX(DeviceIDs.kIntakeDrive);

  private Follower rightPivotFollower =
      new Follower(DeviceIDs.kLeftIntakePivot, MotorAlignmentValue.Opposed);

  // Configurations for the Kraken motors.
  private TalonFXConfiguration leftPivotConfig;
  private TalonFXConfiguration rightPivotConfig;
  private TalonFXConfiguration driveMotorConfig;

  // StatusSignals which obtain statistics from each motor
  private final StatusSignal<AngularVelocity> leftPivotVelocity;
  private final StatusSignal<Voltage> leftPivotVoltage;
  private final StatusSignal<Current> leftPivotCurrent;

  private final StatusSignal<AngularVelocity> rightPivotVelocity;
  private final StatusSignal<Voltage> rightPivotVoltage;
  private final StatusSignal<Current> rightPivotCurrent;

  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveVoltage;
  private final StatusSignal<Current> driveCurrent;

  // Request that tells
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public IntakeIOTalonFX() {
    // Configure the left pivot motor to use the specified PID + FF gains
    leftPivotConfig = new TalonFXConfiguration().withSlot0(IntakeConstants.kPivotGains);
    // Configure the right pivot motor to use the specified PID + FF gains and to reverse its
    // direction
    // Reversing direction makes it so that positive for left is the same direction as positive for
    // right
    rightPivotConfig =
        new TalonFXConfiguration()
            .withSlot0(IntakeConstants.kPivotGains)
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    driveMotorConfig = new TalonFXConfiguration();

    // Tells the motor on startup that is at position 0
    leftPivotMotor.setPosition(0);
    rightPivotMotor.setPosition(0);

    // Apply configurations
    leftPivotMotor.getConfigurator().apply(leftPivotConfig);
    rightPivotMotor.getConfigurator().apply(rightPivotConfig);
    // driveMotor.getConfigurator().apply(driveMotorConfig);

    leftPivotVelocity = leftPivotMotor.getVelocity();
    leftPivotVoltage = leftPivotMotor.getMotorVoltage();
    leftPivotCurrent = leftPivotMotor.getSupplyCurrent();

    rightPivotVelocity = rightPivotMotor.getVelocity();
    rightPivotVoltage = rightPivotMotor.getMotorVoltage();
    rightPivotCurrent = rightPivotMotor.getSupplyCurrent();

    driveVelocity = driveMotor.getVelocity();
    driveVoltage = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();

    // Configure all StatusSignals to update every 20ms
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPivotVelocity,
        leftPivotVoltage,
        leftPivotCurrent,
        rightPivotVelocity,
        rightPivotVoltage,
        rightPivotCurrent,
        driveVelocity,
        driveVoltage,
        driveCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // If every Status signal comes back OK, then it's connected
    inputs.leftPivotConnected =
        BaseStatusSignal.refreshAll(leftPivotVelocity, leftPivotVoltage, leftPivotCurrent).isOK();
    inputs.leftPivotVelocityRadPerSec = leftPivotVelocity.getValue().in(RadiansPerSecond);
    inputs.leftPivotAppliedVolts = leftPivotVoltage.getValueAsDouble();
    inputs.leftPivotCurrentDrawAmps = leftPivotCurrent.getValueAsDouble();

    inputs.rightPivotConnected =
        BaseStatusSignal.refreshAll(rightPivotVelocity, rightPivotVoltage, rightPivotCurrent)
            .isOK();
    inputs.rightPivotVelocityRadPerSec = rightPivotVelocity.getValue().in(RadiansPerSecond);
    inputs.rightPivotAppliedVolts = rightPivotVoltage.getValueAsDouble();
    inputs.rightPivotCurrentDrawAmps = rightPivotCurrent.getValueAsDouble();

    inputs.driveConnected =
        BaseStatusSignal.refreshAll(driveVelocity, driveVoltage, driveCurrent).isOK();
    inputs.driveVelocityRadPerSec = driveVelocity.getValue().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
    inputs.driveCurrentDrawAmps = driveCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    // Use closed-loop/PID + FF control by applying the position request to the motors
    leftPivotMotor.setControl(positionRequest.withPosition(positionRotations));
    rightPivotMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setPivotSpeed(double speed) {
    // The motors have slightly different gear ratios so run them at slightly different speeds
    leftPivotMotor.set(speed);
    rightPivotMotor.set(speed * -0.95);
  }

  /**
   * Set the drive (intake wheel) motor to open-loop control
   *
   * @param speed determines the speed of the wheel on a scale of -1 to 1
   */
  @Override
  public void setWheelSpeed(double speed) {
    driveMotor.set(speed);
  }
}
