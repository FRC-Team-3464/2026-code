package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class ShooterTargetTest {
  @AfterEach
  void cleanup() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    RobotState.getInstance().setPose(Pose2d.kZero);
  }

  @ParameterizedTest
  @EnumSource(
      value = AllianceStationID.class,
      names = {"Blue1", "Red1"})
  void mirroredPositionsSelectOwnHubOrNearestHomeCorner(AllianceStationID alliance) {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(alliance);
    DriverStationSim.notifyNewData();
    RobotState state = RobotState.getInstance();
    double boundary = FieldConstants.LinesVertical.allianceZone;
    Translation2d hub =
        AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    for (double x : new double[] {2, boundary, boundary + 0.01, FieldConstants.fieldLength - 2}) {
      for (double y : new double[] {1, FieldConstants.fieldWidth - 1}) {
        state.setPose(AllianceFlipUtil.apply(new Pose2d(x, y, Rotation2d.kZero)));
        var target = state.selectTeleopShooterTarget();
        assertEquals(
            hub, state.getAutoShooterTarget(), "Auto hub must not depend on robot position");
        if (x <= boundary) {
          assertEquals(RobotState.ShotType.HUB, target.type());
          assertEquals(hub, target.position());
        } else {
          assertEquals(RobotState.ShotType.PASS, target.type());
          assertEquals(AllianceFlipUtil.apply(new Translation2d(2, y)), target.position());
        }
      }
    }
    state.setPose(new Pose2d(Double.NaN, 1, Rotation2d.kZero));
    assertEquals(RobotState.ShotType.UNAVAILABLE, state.selectTeleopShooterTarget().type());
    state.setPose(Pose2d.kZero);
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
    assertEquals(RobotState.ShotType.UNAVAILABLE, state.selectTeleopShooterTarget().type());
  }
}
