package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class HeadingResetTest {
  @ParameterizedTest
  @EnumSource(
      value = AllianceStationID.class,
      names = {"Blue1", "Red1"})
  void resetPreservesTranslationAndTracksSubsequentTurnsWithoutResettingHardware(
      AllianceStationID alliance) {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(alliance);
    DriverStationSim.notifyNewData();
    RecordingGyro gyro = new RecordingGyro();
    double[] wheelTurnRadians = {0.0};
    ModuleIO[] wheels = new ModuleIO[4];
    for (int i = 0; i < wheels.length; i++) {
      var location = DriveConstants.kModuleTranslations[i];
      wheels[i] =
          new ModuleIO() {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              inputs.positionValid = true;
              inputs.driveConnected = true;
              inputs.turnConnected = true;
              inputs.turnEncoderConnected = true;
              inputs.turnPosition = new Rotation2d(-location.getY(), location.getX());
              inputs.drivePositionRad =
                  wheelTurnRadians[0]
                      * location.getNorm()
                      / DriveConstants.TunerConstants.FrontLeft.WheelRadius;
            }
          };
    }
    Drive drive = new Drive(gyro, wheels[0], wheels[1], wheels[2], wheels[3]);
    RobotState state = RobotState.getInstance();
    try {
      tick(drive);
      Rotation2d initialDriverHeading =
          Rotation2d.fromDegrees(45 + (alliance == AllianceStationID.Red1 ? 180 : 0));
      assertEquals(0, drive.getDriverHeading().minus(initialDriverHeading).getRadians(), 1e-8);
      state.setPose(
          new Pose2d(2, 3, Rotation2d.fromDegrees(70)),
          drive.getModulePositions(),
          drive.getRawGyroRotation());
      reset(drive);
      assertHeading(drive, 0);
      tick(drive);
      assertHeading(drive, 0);
      assertEquals(0, gyro.resetCalls);
      assertEquals(2, state.getEstimatedPose().getX(), 1e-9);
      assertEquals(3, state.getEstimatedPose().getY(), 1e-9);

      gyro.degrees = 65;
      tick(drive);
      assertHeading(drive, 20);
      reset(drive);
      tick(drive);
      assertHeading(drive, 0);
      gyro.degrees = 55;
      tick(drive);
      assertHeading(drive, -10);

      gyro.connected = false;
      tick(drive);
      reset(drive);
      tick(drive);
      assertHeading(drive, 0);
      wheelTurnRadians[0] = Math.toRadians(15);
      tick(drive);
      assertHeading(drive, 15);
      gyro.degrees = 120;
      gyro.connected = true;
      tick(drive);
      assertHeading(drive, 15);
      gyro.degrees = 130;
      tick(drive);
      assertHeading(drive, 25);
      assertEquals(0, gyro.resetCalls);
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(drive);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }

  private void tick(Drive drive) {
    SimHooks.stepTiming(.02);
    drive.periodic();
  }

  private void reset(Drive drive) {
    var command = drive.resetHeading();
    command.initialize();
    command.end(false);
  }

  private void assertHeading(Drive drive, double degrees) {
    assertEquals(degrees, drive.getRawGyroRotation().getDegrees(), 1e-8);
    assertEquals(degrees, drive.getDriverHeading().getDegrees(), 1e-8);
    assertEquals(degrees, RobotState.getInstance().getRotation().getDegrees(), 1e-8);
  }

  private static class RecordingGyro implements GyroIO {
    double degrees = 45;
    boolean connected = true;
    int resetCalls;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = connected;
      inputs.yawPosition = Rotation2d.fromDegrees(degrees);
    }

    @Override
    public void setYaw(Rotation2d angle) {
      resetCalls++;
      // Model the existing REAL Pigeon adapter's convention.
      degrees = angle.getDegrees() + 180;
    }
  }
}
