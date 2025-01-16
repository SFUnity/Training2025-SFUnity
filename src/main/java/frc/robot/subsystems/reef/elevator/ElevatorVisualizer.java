package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final String key;

  public ElevatorVisualizer(String key) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
    LoggedMechanismRoot2d root = mechanism.getRoot("elevator", 1.5, 2.0);
    elevator = new LoggedMechanismLigament2d("elevator", 0, 0);

    root.append(elevator);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double heightMeters) {
    // Log Mechanism2d
    elevator.setLength(heightMeters);
    Logger.recordOutput("Reef/Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d elevator =
        new Pose3d(
            ElevatorConstants.elevatorOrigin.getX(),
            heightMeters * 10,
            ElevatorConstants.elevatorOrigin.getY(),
            new Rotation3d(0, 0, 0.0));
    Logger.recordOutput("Reef/Elevator/Mechanism3d/" + key, elevator);
  }
}
