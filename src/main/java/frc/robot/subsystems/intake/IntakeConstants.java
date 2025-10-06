package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final double minAngleRads = Units.degreesToRadians(10);
  public static final double maxAngleRads = Units.degreesToRadians(135);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final LoggedTunableNumber kP;


  static {
    updateTunables();

    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 0.1);
        break;
    }
  }


  public static void updateTunables() {
  }
}
