package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX rollerMotor;

  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVoltage;
  private final StatusSignal<Temperature> intakeTemp;

  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Temperature> rollerTemp;

  public IntakeIOTalonFX(int intakeId, int rollerId) {
    intakeMotor = new TalonFX(intakeId);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeMotor.getConfigurator().apply(intakeConfig);

    rollerMotor = new TalonFX(rollerId);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerMotor.getConfigurator().apply(rollerConfig);

    intakePosition = intakeMotor.getPosition();
    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVoltage = intakeMotor.getMotorVoltage();
    intakeTemp = intakeMotor.getDeviceTemp();

    rollerPosition = rollerMotor.getPosition();
    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerTemp = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakePosition,
        intakeVelocity,
        intakeAppliedVoltage,
        intakeTemp,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVoltage,
        rollerTemp);

    intakeMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakePosition,
        intakeVelocity,
        intakeAppliedVoltage,
        intakeTemp,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVoltage,
        rollerTemp);

    inputs.intakeArmMotorConnected = intakePosition.getStatus().isOK();
    inputs.intakeArmPosition = intakePosition.getValue();
    inputs.intakeArmVelocity = intakeVelocity.getValue();
    inputs.intakeArmAppliedVoltage = intakeAppliedVoltage.getValue();
    inputs.intakeArmMotorTemp = intakeTemp.getValue();

    inputs.rollerMotorConnected = rollerVelocity.getStatus().isOK();
    inputs.rollerPosition = rollerPosition.getValue();
    inputs.rollerVelocity = rollerVelocity.getValue();
    inputs.rollerAppliedVoltage = rollerAppliedVoltage.getValue();
    inputs.rollerMotorTemp = rollerTemp.getValue();
  }

  @Override
  public void setIntakeArmMotorControl(ControlRequest controlRequest) {
    intakeMotor.setControl(controlRequest);
  }

  @Override
  public void setRollerMotorControl(ControlRequest controlRequest) {
    rollerMotor.setControl(controlRequest);
  }
}
