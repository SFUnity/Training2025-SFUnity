package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.ground.GroundConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class GroundVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d ground;
  private final String key;

  public GroundVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(1, 1, new Color8Bit(Color.kBlack));
    root = mechanism.getRoot("Ground Root", 0, Units.inchesToMeters(9.063));
    ground =
        new LoggedMechanismLigament2d(
            "Ground",
            armLengthMeters,
            Units.radiansToDegrees(maxAngleRads),
            8,
            new Color8Bit(color));

    root.append(ground);
  }

  /** Update intake visualizer with current intake angle */
  public void update(Angle angle) {
    // Log Mechanism2d
    ground.setAngle(angle.in(Degrees));
    Logger.recordOutput("Ground/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    // The inches to meters stuff here will have to be changed
    Pose3d carriagePose =
        new Pose3d(0, 0, Units.inchesToMeters(10), new Rotation3d(0.0, angle.in(Radians), 0.0));
    Logger.recordOutput("Ground/Mechanism3d/" + key, carriagePose);
  }
}
