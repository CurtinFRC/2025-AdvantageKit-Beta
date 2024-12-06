package frc.robot.subsystems.index;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;

public class IndexIOSparkMax implements IndexIO {
  private static final int PORT = 21;

  SparkMax spark = new SparkMax(PORT, MotorType.kBrushless);

  @Override
  public void updateInputs(IndexIOInputs inputs) {
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
