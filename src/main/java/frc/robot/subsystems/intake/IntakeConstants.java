package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final double minAngleRads = Units.degreesToRadians(10);
  public static final double maxAngleRads = Units.degreesToRadians(135);
  public static final double intakeLengthMeters = Units.inchesToMeters(15);
  public static final LoggedTunableNumber kP;
  public static LoggedTunableNumber loweredAngle;
  public static LoggedTunableNumber raisedAngle;
  public static LoggedTunableNumber inVoltage;

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
    loweredAngle = new LoggedTunableNumber("Intake/loweredAngle", 125);
    raisedAngle = new LoggedTunableNumber("Intake/raisedAngle", 0);
    inVoltage = new LoggedTunableNumber("Intake/inVoltage", 6);
  }
}
