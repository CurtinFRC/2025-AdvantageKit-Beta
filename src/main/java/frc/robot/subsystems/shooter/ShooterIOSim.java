// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 2, 1.25),
          DCMotor.getNEO(1)); // guessed values

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(0.02);
    inputs.appliedVolts = flywheelSim.getInputVoltage();
    inputs.velocity = flywheelSim.getAngularVelocityRPM();
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.position = 0.0; // we dont care about position so set it to 0 in sim
  }

  @Override
  public void setVoltage(double volts) {
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    flywheelSim.setInputVoltage(0);
    flywheelSim.setAngularVelocity(0);
  }
}
