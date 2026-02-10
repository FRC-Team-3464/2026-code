package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * This interface contains the class which initializes all the inputs to be
 * stored as data, as well as the default methods to update the values of these
 * inputs, set the speed of the motors, and get the values of the limit
 * switches.
 * 
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Owen Biamonte
 */
public interface ClimberIO {

    /**
     * Sets the values of the inputs using the physical objects of the built-in
     * encoders and limit switches.
     */
    default void updateInputs(ClimberIOInputs inputs) {
    }

    /**
     * Initializes all the inputs to be stored as data, including the velocity and
     * position of the motor and the states of the limit switches.
     */
    @AutoLog
    public static class ClimberIOInputs {
        public double climberVelocityRPS;
        public double climberPositionRot;
        public boolean atTopLimit = false;
        public boolean atBottomLimit = false;
    }

    /**
     * Gets the value of the top limit switch and returns it as a boolean.
     * 
     * @return whether or not the top limit switch is hit: true or false.
     */
    default boolean topLimitHit() {
        return false;
    }

    /**
     * Gets the value of the bottom limit switch and returns it as a boolean.
     * 
     * @return whether or not the bottom limit switch is hit: true or false.
     */
    default boolean bottomLimitHit() {
        return false;
    }

    /** Sets the speed of the climber motor ranging from -1.0 to 1.0. */
    default void setClimberSpeed(double speed) {
    }

}
