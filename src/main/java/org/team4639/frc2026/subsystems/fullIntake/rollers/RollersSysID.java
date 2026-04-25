package org.team4639.frc2026.subsystems.fullIntake.rollers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class RollersSysID {
  public static SysIdRoutine routine;

  public RollersSysID(Rollers rollers) {
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Roller SysID Routine", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> rollers.setVoltage(voltage.in(Volts)), null, rollers));
  }

  public Command quasistaticForward() {
    return routine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command quasistaticReverse() {
    return routine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command dynamicForward() {
    return routine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command dynamicReverse() {
    return routine.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
