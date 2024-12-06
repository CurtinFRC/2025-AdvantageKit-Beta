package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {
  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs;

  public Index(IndexIO io) {
    this.io = io;
    inputs = new IndexIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }
}
