package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION;
import frc.robot.subsystems.intake.IntakeConstants.ROLLER_STATE;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputs intakeIOInputs;

  private INTAKE_POSITION intakePosition = INTAKE_POSITION.GROUND;
  private ROLLER_STATE rollerState = ROLLER_STATE.STOP;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    intakeIOInputs = new IntakeIOInputs();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
  }

  public void setIntakeArmPosition(INTAKE_POSITION newPosition) {
    intakeIO.setIntakePosition(newPosition);
    intakePosition = newPosition;
  }

  public Command setIntakeArmPositionCommand(INTAKE_POSITION newPosition) {
    return runOnce(() -> setIntakeArmPosition(newPosition));
  }

  public INTAKE_POSITION getIntakePosition() {
    return intakePosition;
  }

  public void setRollerState(ROLLER_STATE newState) {
    intakeIO.setRollerState(newState);
    rollerState = newState;
  }

  public Command setRollerStateCommand(ROLLER_STATE newState) {
    return runOnce(() -> setRollerState(newState));
  }

  public ROLLER_STATE getRollerState() {
    return rollerState;
  }
}
