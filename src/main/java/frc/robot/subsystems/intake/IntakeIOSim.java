package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.Constants;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;

  private double getAngleDeg() {
    return Units.radiansToDegrees(sim.getAngleRads());
  }

  @Override
  public void runRollers(double volts) {
    appliedVolts = volts;
  }

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(IntakeSpeedVolts),
          gearing,
          jKgMetersSquared,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          simulateGravity,
          minAngleRads);
  private final PIDController controller;
  private double pivotAppliedVolts = 0.0;

  public IntakeIOSim() {
    controller = new PIDController(kP.get(), 0, kD.get());
    sim.setState(minAngleRads, velocityRadPerSec);
  }

  @Override
  public void updateInput(IntakeIOInputs inputs) {
    inputs.rollerVoltage = appliedVolts;
    sim.update(Constants.loopPeriodSecs);

    inputs.pivotCurrentPositionDeg = getAngleDeg();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps();
  }

  public void setPivotPosition(double angle) {
    double volts = controller.calculate(getAngleDeg(), angle);
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, high);
    sim.setInputVoltage(pivotAppliedVolts);
  }
}
