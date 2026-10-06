package frc.robot.subsystems.shooter.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import org.junit.jupiter.api.Test;

class TurretReadinessTest {
  @Test
  void readinessNeedsStableConnectedFeedbackForCurrentTarget() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    try {
      RecordingTurretIO io = new RecordingTurretIO();
      Turret turret = new Turret(io);
      turret.setPosition(Rotation2d.kZero);

      turret.periodic();
      assertFalse(turret.atGoal());
      SimHooks.stepTiming(0.12);
      turret.periodic();
      assertTrue(turret.atGoal());

      // A new command target must invalidate the earlier ready result immediately.
      turret.setPosition(Rotation2d.fromRadians(0.4));
      assertFalse(turret.atGoal());
      io.positionRad = 0.4;
      turret.periodic();
      assertFalse(turret.atGoal());
      SimHooks.stepTiming(0.12);
      turret.periodic();
      assertTrue(turret.atGoal());

      io.connected = false;
      turret.periodic();
      assertFalse(turret.atGoal());
      io.connected = true;
      turret.setOpenLoop(0.0);
      turret.periodic();
      assertFalse(turret.atGoal());

      Command tracking = turret.trackTarget(() -> new Translation2d(3.0, 0.0));
      tracking.initialize();
      tracking.end(true);
      turret.periodicAfterScheduler();
      assertEquals(TurretIOOutputMode.OPEN_LOOP, io.outputMode);
      assertEquals(0.0, io.openLoopOutput);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  private static class RecordingTurretIO implements TurretIO {
    boolean connected = true;
    double positionRad;
    TurretIOOutputMode outputMode;
    double openLoopOutput;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
      inputs.connected = connected;
      inputs.positionRad = positionRad;
    }

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
      outputMode = outputs.mode;
      openLoopOutput = outputs.openLoopOutput;
    }
  }
}
