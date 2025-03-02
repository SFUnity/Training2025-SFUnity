package frc.robot.subsystems.carriage;

import static frc.robot.subsystems.carriage.CarriageConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class CarriageIOSparkMax implements CarriageIO {
  private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
  private final RelativeEncoder encoder = rollerMotor.getEncoder();
  private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

  private final SparkMaxConfig config = sparkConfig(inverted, positionFactor);

  public CarriageIOSparkMax() {
    configureSpark(rollerMotor, config, true);
  }

  @Override
  public void updateInputs(CarrageIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps = rollerMotor.getOutputCurrent();
    inputs.beamBreak = !beamBreak.get();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void setBrakeMode(boolean shouldBrake) {
    if (shouldBrake) config.idleMode(IdleMode.kBrake);
    else config.idleMode(IdleMode.kCoast);
    configureSpark(rollerMotor, config, false);
  }
}
