package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
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
          9,
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
    inputs.position = sim.getPositionMeters();
    inputs.velocityInchesPerSec = 39.3701 * sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {

    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    ;
    sim.setInputVoltage(appliedVolts);
  }
}
