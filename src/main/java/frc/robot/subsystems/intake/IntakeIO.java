package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION;
import frc.robot.subsystems.intake.IntakeConstants.ROLLER_STATE;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeArmMotorConnected = false;
    public Angle intakeArmPosition = Degrees.of(0);
    public AngularVelocity intakeArmVelocity = DegreesPerSecond.of(0);
    public Voltage intakeArmAppliedVoltage = Volts.of(0);
    public Temperature intakeArmMotorTemp = Celsius.of(0);

    public boolean rollerMotorConnected = false;
    public Angle rollerPosition = Degrees.of(0);
    public AngularVelocity rollerVelocity = DegreesPerSecond.of(0);
    public Voltage rollerAppliedVoltage = Volts.of(0);
    public Temperature rollerMotorTemp = Celsius.of(0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakePosition(INTAKE_POSITION newPosition) {}

  public default void setRollerState(ROLLER_STATE newRollerState) {}
}
