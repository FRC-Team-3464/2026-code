package frc.robot.subsystems.guts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.GutsConstants;

/**
 * This class contains all of the physical objects: one motor and its
 * corresponding encoder. It also implements the default methods specified in
 * the IO interface to set the speed of the physical motor and update the input
 * values using the encoders.
 * 
 * @author Ryan Hefferon
 */
public class GutsIOSparkMax implements GutsIO {

    private final SparkMax gutMotor = new SparkMax(GutsConstants.kGutMotorID, MotorType.kBrushless);
    private final RelativeEncoder gutEncoder = gutMotor.getEncoder();
    private final SparkMaxConfig gutMotorConfig;

    public GutsIOSparkMax() {
        gutMotorConfig = new SparkMaxConfig();

        gutMotor.configure(gutMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setGutMotorSpeed(double speed) {
        gutMotor.set(speed);
    }

    @Override
    public void updateInputs(GutsIOInputs inputs) {
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(gutEncoder.getVelocity());
        inputs.positionRad = Units.rotationsToRadians(gutEncoder.getPosition());
        inputs.appliedVolts = gutMotor.getAppliedOutput();
        inputs.currentDrawAmps = gutMotor.getOutputCurrent();
    }

}
