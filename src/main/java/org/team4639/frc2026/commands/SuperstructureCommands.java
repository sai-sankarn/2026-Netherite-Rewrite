package org.team4639.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;

public class SuperstructureCommands {
  public static Command shootCommand(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
        .runShooterCommand()
        .alongWith(
            Commands.sequence(
                Commands.waitSeconds(0.75),
                Commands.parallel(kicker.runKicker(), spindexer.spinCommand())));
  }
}
