package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {
  private DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
  private DCMotor turnMotorModel = DCMotor.getKrakenX44(1);

  private DCMotorSim driveMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              driveMotorModel, 0.025, DriveConstants.kDriveGearRatio),
          driveMotorModel);
  private DCMotorSim turnMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.kAngleGearRatio),
          turnMotorModel);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0.1, 0, 0.001);
  private PIDController turnController = new PIDController(15, 0, 0);
  private double driveFFVolts = 0;
  private SimpleMotorFeedforward driveFFModel =
      new SimpleMotorFeedforward(
          DriveConstants.kDriveKS, DriveConstants.kDriveKV, DriveConstants.kDriveKA);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // driveFFModel = new SimpleMotorFeedforward(0.0, 0.05);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveMotorSim.update(Constants.kLoopPeriodSeconds);
    turnMotorSim.update(Constants.kLoopPeriodSeconds);

    inputs.data =
        new ModuleIOData(
            true,
            driveMotorSim.getAngularPositionRad(),
            driveMotorSim.getAngularVelocityRadPerSec(),
            driveAppliedVolts,
            true,
            Rotation2d.fromRadians(MathUtil.angleModulus(turnMotorSim.getAngularPositionRad())),
            turnMotorSim.getAngularVelocityRadPerSec(),
            turnAppliedVolts,
            driveMotorSim.getCurrentDrawAmps(),
            turnMotorSim.getCurrentDrawAmps());
  }

  @Override
  public void runDriveDutyCycle(double percentOutput) {
    driveClosedLoop = false;
    driveAppliedVolts = percentOutput * 12;
  }

  @Override
  public void runTurnDutyCycle(double percentOutput) {
    turnClosedLoop = false;
    turnAppliedVolts = percentOutput * 12;
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = 0.75 * driveFFModel.calculate(velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void runTurnAngle(Rotation2d angle) {
    turnClosedLoop = true;
    turnController.setSetpoint(angle.getRadians());
  }
}
