package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX leftPivotMotor = new TalonFX(DeviceIDs.kLeftIntakePivot);
  private TalonFX rightPivotMotor = new TalonFX(DeviceIDs.kLeftIntakePivot);
  private TalonFX driveMotor = new TalonFX(DeviceIDs.kIntakeDrive);

  private Follower rightPivotFollower =
      new Follower(DeviceIDs.kLeftIntakePivot, MotorAlignmentValue.Opposed);

  private TalonFXConfiguration leftPivotConfig;
  private TalonFXConfiguration rightPivotConfig;
  private TalonFXConfiguration driveMotorConfig;

  private final StatusSignal<AngularVelocity> leftPivotVelocity;
  private final StatusSignal<Voltage> leftPivotVoltage;
  private final StatusSignal<Current> leftPivotCurrent;

  private final StatusSignal<AngularVelocity> rightPivotVelocity;
  private final StatusSignal<Voltage> rightPivotVoltage;
  private final StatusSignal<Current> rightPivotCurrent;

  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveVoltage;
  private final StatusSignal<Current> driveCurrent;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public IntakeIOTalonFX() {
    leftPivotConfig = new TalonFXConfiguration().withSlot0(IntakeConstants.kPivotGains);
    rightPivotConfig = new TalonFXConfiguration().withSlot0(IntakeConstants.kPivotGains);

    leftPivotMotor.setPosition(0);
    rightPivotMotor.setPosition(0);

    leftPivotMotor.getConfigurator().apply(leftPivotConfig);
    rightPivotMotor.getConfigurator().apply(rightPivotConfig);
    driveMotor.getConfigurator().apply(driveMotorConfig);

    rightPivotMotor.setControl(rightPivotFollower);

    leftPivotVelocity = leftPivotMotor.getVelocity();
    leftPivotVoltage = leftPivotMotor.getMotorVoltage();
    leftPivotCurrent = leftPivotMotor.getSupplyCurrent();

    rightPivotVelocity = rightPivotMotor.getVelocity();
    rightPivotVoltage = rightPivotMotor.getMotorVoltage();
    rightPivotCurrent = rightPivotMotor.getSupplyCurrent();

    driveVelocity = driveMotor.getVelocity();
    driveVoltage = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();

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
    leftPivotMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setPivotSpeed(double speed) {
    leftPivotMotor.set(speed);
    rightPivotMotor.setControl(rightPivotFollower);
  }

  @Override
  public void setWheelSpeed(double speed) {
    driveMotor.set(speed);
  }
}
