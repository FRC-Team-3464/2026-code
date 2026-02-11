package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {

private final DCMotor gearbox = DCMotor.getKrakenX44(1);
private final DCMotorSim sim;

//private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

private double appliedVolts = 0.0;

public ClimberIOSim() {
    sim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, ClimberConstants.kClimberMotorGearRatio), 
            gearbox
        );
}

@Override
public void updateInputs(ClimberIOInputs inputs) {

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRotations();
    inputs.velocityRadPerSec = sim.getAngularVelocityRPM();
}

@Override
public void setClimberSpeed(double speed) {
    appliedVolts = 12 * speed;
}

}
