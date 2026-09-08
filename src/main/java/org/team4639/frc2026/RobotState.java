package org.team4639.frc2026;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;

  @AutoLogOutput @Getter @Setter private boolean extended = false;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}
