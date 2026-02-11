package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    SparkMax armMotor = new SparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
    SparkMax wheelMotor = new SparkMax(IntakeConstants.kRollerMotorID, MotorType.kBrushless);
    RelativeEncoder armEncoder = armMotor.getEncoder();
    RelativeEncoder wheelEncoder = wheelMotor.getEncoder();
    SparkMaxConfig armConfig;
    SparkMaxConfig wheelConfig;

    public IntakeIOHardware() {
        armConfig = new SparkMaxConfig();
        wheelConfig = new SparkMaxConfig();
        // armMotor.configure(armConfig, null, null);
        // wheelMotor.configure(armConfig, null, null);
    }

    @Override
    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
    inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity());
    inputs.wheelVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(wheelEncoder.getVelocity());
    inputs.armPositionRad = Units.rotationsToRadians(armEncoder.getPosition());
    inputs.wheelPositionRad = Units.rotationsToRadians(wheelEncoder.getPosition());
    inputs.armAppliedVolts = armMotor.getAppliedOutput();
    inputs.wheelAppliedVolts = wheelMotor.getAppliedOutput();
    inputs.armCurrentDrawAmps = armMotor.getOutputCurrent();
    inputs.wheelCurrentDrawAmps = wheelMotor.getOutputCurrent();
    }

}
