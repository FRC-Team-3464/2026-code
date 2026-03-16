// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double kLoopPeriodSeconds = 0.02;

  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;

  public static boolean kDisableHAL = false;
  public static boolean kTuningMode = true;

  public static void disableHAL() {
    kDisableHAL = true;
  }

  public static RobotConfig kRobotConfig;

  public static final class DeviceIDs {
    public static final int kPigeon2 =
        DriveConstants.TunerConstants.DrivetrainConstants.Pigeon2Id; // 23

    public static final int kFrontLeftModuleDrive =
        DriveConstants.TunerConstants.FrontLeft.DriveMotorId; // 1
    public static final int kFrontLeftModuleAzimuth =
        DriveConstants.TunerConstants.FrontLeft.SteerMotorId; // 2
    public static final int kFrontLeftModuleEncoder =
        DriveConstants.TunerConstants.FrontLeft.EncoderId; // 19

    public static final int kFrontRightModuleDrive =
        DriveConstants.TunerConstants.FrontRight.DriveMotorId; // 3
    public static final int kFrontRightModuleAzimuth =
        DriveConstants.TunerConstants.FrontRight.SteerMotorId; // 4
    public static final int kFrontRightModuleEncoder =
        DriveConstants.TunerConstants.FrontRight.EncoderId; // 20

    public static final int kBackLeftModuleDrive =
        DriveConstants.TunerConstants.BackLeft.DriveMotorId; // 5
    public static final int kBackLeftModuleAzimuth =
        DriveConstants.TunerConstants.BackLeft.SteerMotorId; // 6
    public static final int kBackLeftModuleEncoder =
        DriveConstants.TunerConstants.BackLeft.EncoderId; // 21

    public static final int kBackRightModuleDrive =
        DriveConstants.TunerConstants.BackRight.DriveMotorId; // 7
    public static final int kBackRightModuleAzimuth =
        DriveConstants.TunerConstants.BackRight.SteerMotorId; // 8
    public static final int kBackRightModuleEncoder =
        DriveConstants.TunerConstants.BackRight.EncoderId; // 22

    public static final int kTurretFlywheel = 12;
    public static final int kTurretHood = 15;
    public static final int kTurretAzimuth = 14;

    public static final int kGuts = 16;

    public static final int kIntakeDrive = 17;
    public static final int kIntakePivot = 18;
  }
}
