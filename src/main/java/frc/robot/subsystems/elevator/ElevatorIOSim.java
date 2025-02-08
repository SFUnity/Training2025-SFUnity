package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constantsGlobal.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private double appliedVolts = 0.0;
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          gearRatio,
          carrageMassKg,
          drumRadiusMeters,
          Units.inchesToMeters(minHeightInches),
          Units.inchesToMeters(maxHeightInches),
          true,
          0);

  public ElevatorIOSim() {
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);
    inputs.position = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSec = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetEncoder(final double position) {
    sim.setState(position, 0.0);
  }
}
