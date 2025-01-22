package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVisualizer {
  private final double ELEVATOR_ANGLE_DEG = 78;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d carriage;
  private final String key;

  public ElevatorVisualizer(String key, Color color, double lineWidth) {
    this.key = key;
    mechanism = new LoggedMechanism2d(.3, .7, new Color8Bit(Color.kBlack));
    root = mechanism.getRoot("Elevator", 0, Units.inchesToMeters(1.1));
    carriage =
        new LoggedMechanismLigament2d(
            "Carriage", 0, ELEVATOR_ANGLE_DEG, lineWidth, new Color8Bit(color));

    root.append(carriage);
  }

  public ElevatorVisualizer(String key, Color color) {
    this(key, color, 5);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double heightMeters) {
    // Log Mechanism2d
    carriage.setLength(heightMeters);
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Rotation2d elevatorAngle = Rotation2d.fromDegrees(ELEVATOR_ANGLE_DEG);
    // The inches to meters stuff here will have to be changed
    Pose3d carriagePose =
        new Pose3d(
            Units.inchesToMeters(4.5) + carriage.getLength() * elevatorAngle.getCos(),
            0.0,
            Units.inchesToMeters(7.0) + carriage.getLength() * elevatorAngle.getSin(),
            new Rotation3d());
    Logger.recordOutput("Elevator/Mechanism3d/" + key, carriagePose);
  }
}
