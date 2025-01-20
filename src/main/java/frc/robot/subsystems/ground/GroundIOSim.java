package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.Constants;

public class GroundIOSim implements GroundIO {

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1,
          (3.6768198 * 0.2533142 * 0.2533142) / 3,
          5.7366408,
          Units.degreesToRadians(18.39),
          Units.degreesToRadians(87.39),
          false,
          Units.degreesToRadians(87.39));

  private final PIDController controller;
  private double pivotAppliedVolts = 0.0;
  private double rollersAppliedVolts = 0.0;

  public GroundIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(GroundIOInputs inputs) {

    sim.update(Constants.loopPeriodSecs);

    inputs.pivotCurrentPosition = Rotations.of(sim.getAngleRads());
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps();

    inputs.rollersAppliedVolts = rollersAppliedVolts;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runGroundRollers(double percentOutput) {
    rollersAppliedVolts = 12 * percentOutput;
  }

  @Override
  public void setPivotPosition(Angle angle) {

    double volts = controller.calculate(sim.getAngleRads(), angle.in(Radians));
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void setPID(double p) {
    controller.setPID(p, 0, 0);
  }

  @Override
  public void stop() {
    pivotAppliedVolts = 0.0;
    sim.setInputVoltage(pivotAppliedVolts);
  }
}
