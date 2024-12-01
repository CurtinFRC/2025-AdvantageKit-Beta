package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;

public class ShooterIOSparkMax implements ShooterIO {
  private static final int PORT = 99;

  SparkMax spark = new SparkMax(PORT, MotorType.kBrushless);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVolts = spark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.position = spark.getEncoder().getPosition();
    inputs.velocity = spark.getEncoder().getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
