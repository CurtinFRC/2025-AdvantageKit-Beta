// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure {
  private static final int FRONT_BEAMBREAK_PORT = 5;
  private static final int BACK_BEAMBREAK_PORT = 6;

  // TODO should be logged akit style
  private final DigitalInput frontBeambreak = new DigitalInput(FRONT_BEAMBREAK_PORT);
  private final DigitalInput backBeamBreak = new DigitalInput(BACK_BEAMBREAK_PORT);

  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Index m_index;

  public final Trigger m_hasNote = new Trigger(backBeamBreak::get);
  public final Trigger m_intaking = new Trigger(frontBeambreak::get).negate();

  public Superstructure(Shooter shooter, Intake intake, Index index) {
    m_shooter = shooter;
    m_intake = intake;
    m_index = index;

    m_intaking.onTrue(
        Commands.parallel(
            m_intake.setVoltage(5).until(m_hasNote).andThen(m_intake.stop()),
            m_index
                .setVoltage(-3.5)
                .until(m_hasNote)
                .andThen(new WaitCommand(0.5))
                .andThen(m_index.setVoltage(2))
                .until(m_hasNote)
                .andThen(m_index.stop())));
  }

  public Command intake() {
    return m_intake.setVoltage(10).until(m_intaking);
  }

  public Command shoot() {
    return m_shooter
        .spinup(250)
        .withTimeout(1.5)
        .andThen(Commands.parallel(m_shooter.maintain(), m_index.setVoltage(8)));
  }

  public Command stop() {
    return Commands.parallel(m_shooter.stop(), m_intake.stop(), m_index.stop());
  }
}
