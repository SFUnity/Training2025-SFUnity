package frc.robot.subsystems.ground;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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
          Units.degreesToRadians(87.39),
          null);

  private final PIDController controller;
  private double pivotAppliedVolts = 0.0;
  private double rollersAppliedVolts = 0.0;

  public GroundIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(GroundIOInputs inputs) {}

  @Override
  public void runGroundRollers(double percentOutput) {
    rollersAppliedVolts = 12 * percentOutput;
  }

  @Override
  public void setPivotPosition(Angle angle) {
    /** INCOMPLETE */
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
