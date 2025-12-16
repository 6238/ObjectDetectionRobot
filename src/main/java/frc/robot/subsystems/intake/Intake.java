package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION;
import frc.robot.subsystems.intake.IntakeConstants.ROLLER_STATE;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputs intakeIOInputs;

  private INTAKE_POSITION intakePosition = INTAKE_POSITION.GROUND;
  private ROLLER_STATE rollerState = ROLLER_STATE.STOP;

  private final PositionVoltage intakeArmPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage rollerPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final VoltageOut rollerVoltageOut = new VoltageOut(0);

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
    intakePosition = newPosition;
  }

  public INTAKE_POSITION getIntakePosition() {
    return intakePosition;
  }

  public void setRollerState(ROLLER_STATE newState) {
    switch (newState) {
      case INTAKE:
        intakeIO.setRollerMotorControl(rollerVoltageOut.withOutput(IntakeConstants.INTAKE_VOLTAGE));
        break;
      case EJECT:
        intakeIO.setRollerMotorControl(rollerVoltageOut.withOutput(IntakeConstants.EJECT_VOLTAGE));
        break;
      case HOLD:
        intakeIO.setRollerMotorControl(rollerPositionVoltage.withPosition(intakeIOInputs.rollerPosition));
        break;
      case STOP:
        intakeIO.setRollerMotorControl(rollerVoltageOut.withOutput(0));
        break;
    }
    rollerState = newState;
  }

  public ROLLER_STATE getRollerState() {
    return rollerState;
  }
}
