package frc.robot.subsystems.ground;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Ground extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Ground/Gains/kP", gains.kP);
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Ground/Angles/lowered", 26);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Ground/Angles/raised", 0);

  // In percent output
  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Ground/Speeds/groundRollers", 1);

  private double positionSetpoint = 0;

  private final GroundIO io;

  public Ground(GroundIO io) {
    this.io = io;
  }
}
