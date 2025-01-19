package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class GroundIOSparkMax implements GroundIO {
  private static final double GEAR_RATIO = 1;
  private final SparkMax pivot = new SparkMax(0, null);
  private final SparkMax rollers = new SparkMax(0, null);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkClosedLoopController pid = pivot.getClosedLoopController();

  public GroundIOSparkMax() {}

  @Override
  public void stop() {
    pivot.stopMotor();
  }

  @Override
  public void updateInputs(GroundIOInputs inputs) {
    inputs.pivotCurrentPosition = Units.rotationsToDegrees(encoder.getPosition() / GEAR_RATIO);
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();

    inputs.rollersAppliedVolts = rollers.getAppliedOutput() * rollers.getBusVoltage();
    inputs.rollersCurrentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void runGroundRollers(double percentOutput) {
    rollers.set(percentOutput);
  }

  @Override
  public void setPivotPosition(Angle setpointRots) {
    pid.setReference(setpointRots.in(Degrees), ControlType.kPosition);
  }

  @Override
  public void setPID(double kP) {
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.closedLoop.pidf(kP, 0, 0, 0);
    tryUntilOk(
        pivot,
        () ->
            pivot.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
