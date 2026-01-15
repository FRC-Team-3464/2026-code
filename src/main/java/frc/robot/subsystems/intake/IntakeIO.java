package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code IntakeIO} class provides methods for interacting with the intake
 * motors and updating the intake inputs.
 * 
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Maxwell Morgan
 * @author Julien Precourt
 */
public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {
    }

    @AutoLog
    public class IntakeIOInputs {

    }

    default void runPivotMotor(double speed) {
    }

    default void runRollerMotor(double speed) {
    }
}
