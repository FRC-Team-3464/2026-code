package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOHardware implements IntakeIO {
    SparkMax armMotor = new SparkMax(5, MotorType.kBrushless);
    SparkMax wheelMotor = new SparkMax(6, MotorType.kBrushless);
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
    inputs.armMotorVelocityRPM = armEncoder.getVelocity();
    inputs.wheelMotorVelocityRPM = wheelEncoder.getVelocity();
    inputs.armMotorPositionsRotations = armEncoder.getPosition();
    inputs.wheelMotorPositionRotations = wheelEncoder.getPosition();
    }

}
