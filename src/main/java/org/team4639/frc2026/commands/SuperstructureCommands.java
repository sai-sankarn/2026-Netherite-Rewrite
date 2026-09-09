package org.team4639.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.ShooterAditya;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.lib.util.LoggedTunableNumber;

public class SuperstructureCommands {
  static LoggedTunableNumber setpoint = new LoggedTunableNumber("Shooter Setpoint");

  static {
    setpoint.initDefault(0);
  }

  public static Command shootCommand(ShooterAditya shooter, Kicker kicker, Spindexer spindexer) {
    return Commands.runOnce(() -> shooter.setRPM(setpoint.getAsDouble()), shooter);
    // .alongWith(
    //     Commands.sequence(
    //         Commands.waitSeconds(0.75),
    //         Commands.parallel(kicker.runKicker(), spindexer.spinCommand())));
  }
}
