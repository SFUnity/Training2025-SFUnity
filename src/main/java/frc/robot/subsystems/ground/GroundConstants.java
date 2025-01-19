package frc.robot.subsystems.ground;

import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class GroundConstants {
  public static final LoggedTunableNumber kP;
  public static final double pivotPositionFactor = 0;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Ground/Tunables/kP", 2.0);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Ground/SimTunables/kP", 8.0);
        break;
    }
  }
}
