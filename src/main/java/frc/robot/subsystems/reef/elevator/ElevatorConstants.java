package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  public static final int elevatorMotorID = 0;
  public static final double maxElevatorSpeed = 0.5;
  public static final double maxElevatorAcceleration = .5;

  public static final double desiredHeightL1 = 0.5;
  public static final double desiredHeightL2 = 1.5;
  public static final double desiredHeightL3 = 2;
  public static final double desiredHeightHighAlgae = 1;
  public static final double desiredHeightLowAlgae = 1.75;
  public static final double desiredHeightProcessor = 0;
  public static final double desiredHeightSource = 2;
  public static final double desiredHeightBottom = 0;

  public static final double carrageMass = 1;
  public static final double drumRadius = 1;
  public static final double minHeight = 0;
  public static final double maxHeight = 2;

  public static final double gearRatio = 0;
  public static final double wheelRadius = 1;

  public static final double elevatorMinLength = 0;

  public static final Translation2d elevatorOrigin = new Translation2d(0, 0);

  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kI;
  public static final LoggedTunableNumber kD;
  public static final double kS = 0;
  public static final double kG = 2.28;
  public static final double kV = 3.07;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 0.0);
        kI = new LoggedTunableNumber("Reef/Elevator/kI", 0.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 0.0);

        break;
      case SIM:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 0.0);
        kI = new LoggedTunableNumber("Reef/Elevator/kI", 0.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 0.0);

        break;
    }
  }
}
