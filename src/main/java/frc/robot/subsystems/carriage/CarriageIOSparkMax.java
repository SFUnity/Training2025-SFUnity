package frc.robot.subsystems.carriage;

import static frc.robot.subsystems.carriage.CarriageConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class CarriageIOSparkMax implements CarriageIO {
  private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = rollerMotor.getEncoder();
  private final DigitalInput beamBreak = new DigitalInput(beamBreakNumber);

  public CarriageIOSparkMax() {
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 100))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    configureSpark(rollerMotor, config, true);
  }

  @Override
  public void updateInputs(CarrageIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps = rollerMotor.getOutputCurrent();
    inputs.pieceHeld = !beamBreak.get();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setVoltage(volts);
  }

}
