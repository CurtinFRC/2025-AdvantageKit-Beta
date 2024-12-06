package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }
}
