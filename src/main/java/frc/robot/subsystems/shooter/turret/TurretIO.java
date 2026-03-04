package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
    public boolean limitTriggered = false;
  }

  public static enum TurretIOOutputMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public class TurretIOOutputs {
    public TurretIOOutputMode mode = TurretIOOutputMode.CLOSED_LOOP;

    public double openLoopOutput = 0.0;
    public Rotation2d closedLoopTarget = Rotation2d.kZero;
  }

  void updateInputs(TurretIOInputs inputs);

  void applyOutputs(TurretIOOutputs outputs);
}
