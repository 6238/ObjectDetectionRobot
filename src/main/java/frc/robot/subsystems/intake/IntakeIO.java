package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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

  public default void setIntakeArmMotorControl(ControlRequest controlRequest) {}

  public default void setRollerMotorControl(ControlRequest controlRequest) {}
}
