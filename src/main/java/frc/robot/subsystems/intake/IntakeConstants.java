package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static boolean groundAlgae = false;

  public static final int rollersID = 18;
  public static final int pivotID = 15;

  public static final LoggedTunableNumber kP;

  public static final boolean pivotInverted = true;
  public static final boolean rollersInverted = false;
  public static final double pivotPositionFactor = 2.9;
  public static final double rollersPositionFactor = .2;

  public static final double minAngleRads = Units.degreesToRadians(18.39);
  public static final double maxAngleRads = Units.degreesToRadians(87.39);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  public static final LoggedTunableNumber algaeVoltageThreshold =
      new LoggedTunableNumber("Intake/algaeVoltageThreshold", .5);

  // In rotations
  public static final LoggedTunableNumber l1Angle = new LoggedTunableNumber("Intake/L1Angle", 0);
  public static final LoggedTunableNumber loweredAngle;
  public static final LoggedTunableNumber raisedAngle;
  // In volts
  public static final LoggedTunableNumber rollersSpeedIn;
  public static final LoggedTunableNumber rollersSpeedOut;

  static {
    if (groundAlgae) {
      loweredAngle = new LoggedTunableNumber("Intake/loweredAngle", 66);
      raisedAngle = new LoggedTunableNumber("Intake/raisedAngle", 0);
      rollersSpeedIn = new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 6);
      rollersSpeedOut = new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", 4);
    } else {
      loweredAngle = new LoggedTunableNumber("Intake/loweredAngle", 116);
      raisedAngle = new LoggedTunableNumber("Intake/raisedAngle", 0);
      rollersSpeedIn = new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 6);
      rollersSpeedOut = new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", 8);
    }

    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 4.82);
        break;
    }
  }
}
