package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final LoggedTunableNumber kP;

  public static final boolean pivotInverted = false;
  public static final boolean rollersInverted = false;
  public static final double pivotPositionFactor = 25 * 360; // 25 rotations of the motor is 360 degrees of the arm
  public static final double rollersPositionFactor = 5; // 5 rotations of the motor is 1 rotation of the rollers

  public static final double minAngleRads = Units.degreesToRadians(18.39);
  public static final double maxAngleRads = Units.degreesToRadians(87.39);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  public static final LoggedTunableNumber algaeVelocityThreshold =
      new LoggedTunableNumber("Intake/algaeVelocityThreshold");
  public static final LoggedTunableNumber algaeCurrentThreshold =
      new LoggedTunableNumber("Intake/algaeCurrentThreshold");

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.0);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 4.82);
        break;
    }
  }
}
