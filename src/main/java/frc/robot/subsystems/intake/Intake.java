package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputs intakeIOInputs;

  private final PositionVoltage intakeArmPositionVoltage = new PositionVoltage(0).withSlot(0);

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    intakeIOInputs = new IntakeIOInputs();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
  }

  public void setIntakeArmPosition(INTAKE_POSITION newPosition) {
    intakeIO.setIntakeArmMotorControl(intakeArmPositionVoltage.withPosition(newPosition.position));
  }
}
