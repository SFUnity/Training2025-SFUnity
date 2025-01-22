package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  public static final int elevatorMotorID = 0;
  public static final double maxElevatorSpeed = 5;
  public static final double maxElevatorAcceleration = 10;

  public static enum ElevatorHeights {
    L3(desiredHeightL3.get()),
    L2(desiredHeightL2.get()),
    L1(desiredHeightL1.get()),
    AlgaeHigh(desiredHeightHighAlgae.get()),
    AlgaeLow(desiredHeightLowAlgae.get()),
    Processor(desiredHeightProcessor.get()),
    Source(desiredHeightSource.get());

    ElevatorHeights(double height) {
      this.height = height;
    }

    public final double height;
  }

  public static final LoggedTunableNumber desiredHeightL1;
  public static final LoggedTunableNumber desiredHeightL2;
  public static final LoggedTunableNumber desiredHeightL3;
  public static final LoggedTunableNumber desiredHeightHighAlgae;
  public static final LoggedTunableNumber desiredHeightLowAlgae;
  public static final LoggedTunableNumber desiredHeightProcessor;
  public static final LoggedTunableNumber desiredHeightSource;

  public static final double carrageMassKg = Units.lbsToKilograms(15);
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4);
  public static final double minHeightInches = 0;
  public static final double maxHeightInches = 23.0;

  public static final double gearRatio = 0;
  public static final double wheelRadius = 1;

  public static final double elevatorMinLength = 0;

  public static final Translation2d elevatorOrigin = new Translation2d(0, 0);

  public static final int currentLimit = 0;

  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kD;
  public static final LoggedTunableNumber kG;
  public static final LoggedTunableNumber kV;

  public static final double turnPIDMinInput = 0;
  public static final double turnPIDMaxInput = 2 * Math.PI;

  public static final double turnMotorReduction = 150 / 7;
  public static final double encoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double encoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 150.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 17.53);
        kG = new LoggedTunableNumber("Reef/Elevator/kG", 0.11591);
        kV = new LoggedTunableNumber("Reef/Elevator/kV", 11.3);

        desiredHeightL1 = new LoggedTunableNumber("Reef/Elevator/L1");
        desiredHeightL2 = new LoggedTunableNumber("Reef/Elevator/L2");
        desiredHeightL3 = new LoggedTunableNumber("Reef/Elevator/L3");
        desiredHeightLowAlgae = new LoggedTunableNumber("Reef/Elevator/LowAlgae");
        desiredHeightHighAlgae = new LoggedTunableNumber("Reef/Elevator/HighAlgae");
        desiredHeightSource = new LoggedTunableNumber("Reef/Elevator/Source");
        desiredHeightProcessor = new LoggedTunableNumber("Reef/Elevator/Processor");
        break;
      case SIM:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 60.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 0.0);
        kG = new LoggedTunableNumber("Reef/Elevator/kG", 0.06);
        kV = new LoggedTunableNumber("Reef/Elevator/kV", 0.4);
        desiredHeightL1 = new LoggedTunableNumber("Reef/Elevator/L1");
        desiredHeightL2 = new LoggedTunableNumber("Reef/Elevator/L2");
        desiredHeightL3 = new LoggedTunableNumber("Reef/Elevator/L3");
        desiredHeightLowAlgae = new LoggedTunableNumber("Reef/Elevator/LowAlgae");
        desiredHeightHighAlgae = new LoggedTunableNumber("Reef/Elevator/HighAlgae");
        desiredHeightSource = new LoggedTunableNumber("Reef/Elevator/Source");
        desiredHeightProcessor = new LoggedTunableNumber("Reef/Elevator/Processor");
        break;
    }
  }
}
