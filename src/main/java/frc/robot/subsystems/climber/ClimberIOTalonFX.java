package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimberConstants;

/**
 * This class contains all the physical objects required for the climber
 * subsystem, including one motor and two limit switches. It also implements the
 * default methods specified in the IO interface to set the speed of the motor,
 * get the values of the limit switches, and update the inputs by getting the
 * velocity and position of the built-in encoders and the limit switches.
 * 
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Owen Biamonte
 */
public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID);
    private final DigitalInput topLimitSwitch = new DigitalInput(ClimberConstants.topLimitSwitchID);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ClimberConstants.bottomLimitSwitchID);

    private final TalonFXConfiguration climberMotorConfig;

    public ClimberIOTalonFX() {
        climberMotorConfig = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberMotorConfig);
    }

    @Override
    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public boolean topLimitHit() {
        return topLimitSwitch.get();
    }

    @Override
    public boolean bottomLimitHit() {
        return bottomLimitSwitch.get();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberVelocityRPS = Units.rotationsToRadians(climberMotor.getVelocity().getValueAsDouble());
        inputs.climberPositionRot = Units.rotationsToRadians(climberMotor.getPosition().getValueAsDouble());
        inputs.atTopLimit = topLimitHit();
        inputs.atBottomLimit = bottomLimitHit();
    }

}
