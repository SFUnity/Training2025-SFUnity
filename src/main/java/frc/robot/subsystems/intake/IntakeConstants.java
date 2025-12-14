package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final int IntakeSpeedVolts = 3;
  public static final int gearing = 50;
  public static final double jKgMetersSquared = 0.5;
  public static final double armLengthMeters = 1;
  public static final double minAngleRads = 1;
  public static final double maxAngleRads = 1;
  public static final boolean simulateGravity = false;
  public static final double velocityRadPerSec = 0.0;
  public static final double high = 12.0;
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0);
  public static final double loweredPositionDegrees = 2;
  public static final double ejectVolts = -6.0;
}
