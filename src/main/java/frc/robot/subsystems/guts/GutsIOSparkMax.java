package frc.robot.subsystems.guts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * This class contains all of the physical objects: two motors and two
 * corresponding encoders. It also implements the default methods specified in
 * the IO interface to set the speed of the physical motor and update the input
 * values using the encoders.
 * @author Ryan Hefferon
 */
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

        leftGutMotor.configure(leftGutMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightGutMotor.configure(rightGutMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setLeftGutMotorSpeed(double speed) {
        leftGutMotor.set(speed);
    }

    @Override
    public void setRightGutMotorSpeed(double speed) {
        rightGutMotor.set(speed);
    }

    @Override
    public void updateInputs(GutsIOInputs inputs) {
        inputs.leftGutMotorPositionRot = leftGutEncoder.getPosition();
        inputs.leftGutMotorVelocityRPM = leftGutEncoder.getVelocity();
        inputs.rightGutMotorPositionRot = rightGutEncoder.getPosition();
        inputs.rightGutMotorVelocityRPM = rightGutEncoder.getVelocity();
    }

}
