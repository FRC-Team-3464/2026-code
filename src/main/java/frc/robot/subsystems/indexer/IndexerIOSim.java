package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

/** Captures indexer motor requests in SIM without modeling motor or fuel movement. */
public class IndexerIOSim implements IndexerIO {
  private double throatDutyCycle;
  private double tongueDutyCycle;

  /** Reports the requested motor voltages; no velocity or current is simulated. */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // These voltages show what the commands requested at the simulated battery voltage. No motor
    // or sensor model exists, so velocity/current stay zero and the connection flags stay false.
    double batteryVolts = RobotController.getBatteryVoltage();
    inputs.throatAppliedVolts = throatDutyCycle * batteryVolts;
    inputs.tongueAppliedVolts = tongueDutyCycle * batteryVolts;
  }

  /** Stores a bounded throat duty-cycle request for the next input snapshot. */
  @Override
  public void setThroatOpenLoop(double output) {
    throatDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  /** Stores a bounded tongue duty-cycle request for the next input snapshot. */
  @Override
  public void setTongueOpenLoop(double output) {
    tongueDutyCycle = MathUtil.clamp(output, -1.0, 1.0);
  }

  /** Clears both stored requests when the command ends. */
  @Override
  public void stop() {
    throatDutyCycle = 0.0;
    tongueDutyCycle = 0.0;
  }
}
