package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

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
}
