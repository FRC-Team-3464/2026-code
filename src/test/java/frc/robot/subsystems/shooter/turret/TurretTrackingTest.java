package frc.robot.subsystems.shooter.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import org.junit.jupiter.api.Test;

class TurretTrackingTest {
  @Test
  void endingTrackingStopsOutputWithoutWaitingForADefaultCommand() {
    HAL.initialize(500, 0);
    RecordingTurretIO io = new RecordingTurretIO();
    Turret turret = new Turret(io);
    try {
      // Exercise both normal composition completion and interruption while tracking is active.
      for (boolean interrupted : new boolean[] {false, true}) {
        Command tracking = turret.trackTarget(() -> new Translation2d(3.0, 0.0));
        tracking.initialize();
        tracking.execute();
        turret.periodicAfterScheduler();
        assertEquals(TurretIOOutputMode.CLOSED_LOOP, io.outputMode);

        tracking.end(interrupted);
        turret.periodicAfterScheduler();
        assertEquals(TurretIOOutputMode.OPEN_LOOP, io.outputMode);
        assertEquals(0.0, io.openLoopOutput);
      }
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(turret);
    }
  }

  private static class RecordingTurretIO implements TurretIO {
    TurretIOOutputMode outputMode;
    double openLoopOutput;

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
      outputMode = outputs.mode;
      openLoopOutput = outputs.openLoopOutput;
    }
  }
}
