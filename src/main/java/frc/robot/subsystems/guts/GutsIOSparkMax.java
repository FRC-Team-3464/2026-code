package frc.robot.subsystems.guts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * This class contains all of the physical objects: one motor and its
 * corresponding encoder. It also implements the default methods specified in
 * the IO interface to set the speed of the physical motor and update the input
 * values using the encoders.
 * 
 * @author Ryan Hefferon
 */
public class GutsIOSparkMax implements GutsIO {

    private final SparkMax GutMotor = new SparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder GutEncoder = GutMotor.getEncoder();
    private final SparkMaxConfig GutMotorConfig;

    public GutsIOSparkMax() {
        GutMotorConfig = new SparkMaxConfig();

        GutMotor.configure(GutMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setGutMotorSpeed(double speed) {
        GutMotor.set(speed);
    }

    @Override
    public void updateInputs(GutsIOInputs inputs) {
        inputs.GutMotorPositionRot = GutEncoder.getPosition();
        inputs.GutMotorVelocityRPM = GutEncoder.getVelocity();
    }

}
