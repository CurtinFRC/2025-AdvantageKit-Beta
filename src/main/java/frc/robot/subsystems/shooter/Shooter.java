// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final double P = 0.1;
  private static final double D = 0;
  private static final double S = 0;
  private static final double V = 0;
  private static final double A = 0;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final PIDController pid = new PIDController(P, 0, D);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  public Command spinup(double speedRPM) {
    return maintain(speedRPM).until(pid::atSetpoint);
  }

  public Command maintain(double speedRPM) {
    return run(
        () -> {
          Logger.recordOutput("Shooter/Desired Speed", speedRPM);
          var pid_output = pid.calculate(inputs.velocity, speedRPM);
          var ff_output = ff.calculate(RPM.of(speedRPM)).magnitude(); // avoids deprecation warning
          io.setVoltage(pid_output + ff_output);
        });
  }

  public Command maintain() {
    return maintain(pid.getSetpoint());
  }
}
