package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
  public static enum INTAKE_POSITION {
    GROUND(Degrees.of(0)),
    EJECT(Degrees.of(0)),
    TRANSFER(Degrees.of(0));

    public final Angle position;

    private INTAKE_POSITION(Angle position) {
      this.position = position;
    }
  }

  public static enum ROLLER_STATE {
    INTAKE,
    HOLD,
    EJECT,
    STOP,
  }

  public static Voltage INTAKE_VOLTAGE = Volts.of(6);
  public static Voltage EJECT_VOLTAGE = Volts.of(6);
}
