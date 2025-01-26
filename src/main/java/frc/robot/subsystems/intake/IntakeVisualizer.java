package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d ground;
  private final String key;

  private final LoggedTunableNumber xOffset = new LoggedTunableNumber("Ground/xOffset", -10.2);
  private final LoggedTunableNumber yOffset = new LoggedTunableNumber("Ground/yOffset", -8);
  private final LoggedTunableNumber zOffset = new LoggedTunableNumber("Ground/zOffset", 9);

  public IntakeVisualizer(String key, Color color) {
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
        new Pose3d(
            Units.inchesToMeters(xOffset.get()),
            Units.inchesToMeters(yOffset.get()),
            Units.inchesToMeters(zOffset.get()),
            new Rotation3d(0.0, angle.in(Radians), 0.0));
    Logger.recordOutput("Ground/Mechanism3d/" + key, carriagePose);
  }
}
