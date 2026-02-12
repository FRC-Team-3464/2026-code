package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
  private SparkMax pivotMotor = new SparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private TalonFX wheelMotor = new TalonFX(IntakeConstants.kRollerMotorID);
  private SparkMaxConfig pivotConfig;
  private TalonFXConfiguration wheelMotorConfig;

  public IntakeIOHardware() {
    pivotConfig = new SparkMaxConfig();
    wheelMotor.getConfigurator().apply(wheelMotorConfig);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, null);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setWheelSpeed(double speed) {
    wheelMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
    inputs.wheelVelocityRadPerSec =
        Units.rotationsToRadians(wheelMotor.getVelocity().getValueAsDouble());
    inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition());
    inputs.wheelPositionRad = Units.rotationsToRadians(wheelMotor.getPosition().getValueAsDouble());
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput();
    inputs.wheelAppliedVolts = wheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.pivotCurrentDrawAmps = pivotMotor.getOutputCurrent();
    inputs.wheelCurrentDrawAmps = wheelMotor.getMotorVoltage().getValueAsDouble();
  }
}
