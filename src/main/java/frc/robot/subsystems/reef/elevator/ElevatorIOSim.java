package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

public class ElevatorIOSim implements ElevatorIO {
  private final PIDController pid;

  private double appliedVolts = 0.0;
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          25,
          carrageMassKg,
          drumRadiusMeters,
          Units.inchesToMeters(minHeightInches),
          Units.inchesToMeters(maxHeightInches),
          true,
          0);

  public ElevatorIOSim() {
    pid = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.position = Meters.of(sim.getPositionMeters());
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    runVolts(0);
  }
}
