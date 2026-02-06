package frc.robot.subsystems.guts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GutsIOSparkMax implements GutsIO {

    private final SparkMax leftGutMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax rightGutMotor = new SparkMax(1, MotorType.kBrushless);
    private final RelativeEncoder leftGutEncoder = leftGutMotor.getEncoder();
    private final RelativeEncoder rightGutEncoder = rightGutMotor.getEncoder();
    private final SparkMaxConfig leftGutMotorConfig;
    private final SparkMaxConfig rightGutMotorConfig;


    public GutsIOSparkMax() {
        leftGutMotorConfig = new SparkMaxConfig();
        rightGutMotorConfig = new SparkMaxConfig();

        rightGutMotorConfig.follow(leftGutMotor, true);

        //fix later to correctly configure motors
        //leftGutMotor.configure(leftGutMotorConfig, null, null);
        //rightGutMotor.configure(rightGutMotorConfig, null, null);
    }

    @Override
    public void setLeftGutMotorSpeed(double speed) {
        leftGutMotor.set(speed);
    }

    @Override
    public void updateInputs(GutsIOInputs inputs) {
        inputs.leftGutMotorPositionRot = leftGutEncoder.getPosition();
        inputs.leftGutMotorVelocityRPM = leftGutEncoder.getVelocity();
        inputs.rightGutMotorPositionRot = rightGutEncoder.getPosition();
        inputs.rightGutMotorVelocityRPM = rightGutEncoder.getVelocity();
    }

}
