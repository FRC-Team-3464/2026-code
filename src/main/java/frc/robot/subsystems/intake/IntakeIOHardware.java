package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DeviceIDs;

public class IntakeIOHardware implements IntakeIO {
  private SparkMax pivotMotor = new SparkMax(DeviceIDs.kIntakePivot, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private TalonFX driveMotor = new TalonFX(DeviceIDs.kIntakeDrive);
  private SparkMaxConfig pivotConfig;
  private TalonFXConfiguration wheelMotorConfig;

  public IntakeIOHardware() {
    pivotConfig = new SparkMaxConfig();
    // wheelMotorConfig = new TalonFXConfiguration();
    // driveMotor.getConfigurator().apply(wheelMotorConfig);
    pivotMotor.configure(
        pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPivotPosition(double positionRad) {
    // TODO Auto-generated method stub
    IntakeIO.super.setPivotPosition(positionRad);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setWheelSpeed(double speed) {
    driveMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
    inputs.wheelVelocityRadPerSec =
        Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
    inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition());
    inputs.wheelPositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput();
    inputs.wheelAppliedVolts = driveMotor.getTorqueCurrent().getValueAsDouble();
    inputs.pivotCurrentDrawAmps = pivotMotor.getOutputCurrent();
    inputs.wheelCurrentDrawAmps = driveMotor.getMotorVoltage().getValueAsDouble();
  }
}
