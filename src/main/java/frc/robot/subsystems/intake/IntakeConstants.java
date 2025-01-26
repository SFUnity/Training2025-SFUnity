package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final LoggedTunableNumber kP;
  public static final double pivotPositionFactor = 0;
  public static final double minAngleRads = Units.degreesToRadians(18.39);
  public static final double maxAngleRads = Units.degreesToRadians(87.39);
  public static final double armLengthMeters = Units.inchesToMeters(15);

  public static final LoggedTunableNumber algaeVelocityThreshold;
  public static final LoggedTunableNumber algaeCurrentThreshold;

  static {
    algaeVelocityThreshold = new LoggedTunableNumber("ground/algaeVelocityThreshold");
    algaeCurrentThreshold = new LoggedTunableNumber("ground/algaeCurrentThreshold");
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 2.0);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 4.82);
        break;
    }
  }
}
