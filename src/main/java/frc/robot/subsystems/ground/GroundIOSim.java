package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.ground.GroundConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.Constants;

public class GroundIOSim implements GroundIO {

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          50,
          0.5,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          false,
          maxAngleRads);

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

    inputs.pivotCurrentPosition = Radians.of(sim.getAngleRads());
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
