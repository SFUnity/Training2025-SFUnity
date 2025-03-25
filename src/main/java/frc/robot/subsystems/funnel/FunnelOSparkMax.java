package frc.robot.subsystems.funnel;

import static frc.robot.subsystems.funnel.FunnelConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FunnelOSparkMax implements FunnelIO {
    private final SparkMax rollerMotor = new SparkMax(funnelMotorID, MotorType.kBrushless);
    private final SparkMaxConfig config = sparkConfig(inverted, funnelMotorID);
    FunnelOSparkMax(){
        configureSpark(rollerMotor, config, true);
    }
}
