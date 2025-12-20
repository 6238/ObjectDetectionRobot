package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION;
import frc.robot.subsystems.intake.IntakeConstants.ROLLER_STATE;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

public class IntakeIOSim implements IntakeIO {
  // ---------------------------------------------------
  // 1. Hardware Objects (Simulated)
  // ---------------------------------------------------
  private final TalonFX intakeArmMotor;
  private final TalonFX rollerMotor;

  // ---------------------------------------------------
  // 2. Physics Simulation Models
  // ---------------------------------------------------
  // Simulates the physical pivot arm (Gravity, Mass, Inertia)
  private final SingleJointedArmSim armPhysicsSim;
  // Simulates the roller flywheel (Current draw, Momentum)
  private final FlywheelSim rollerPhysicsSim;
  // Simulates the Game Piece interaction (IronMaple)
  private final IntakeSimulation mapleIntakeSim;

  // ---------------------------------------------------
  // 3. Signals & Controls
  // ---------------------------------------------------
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVoltage;
  private final StatusSignal<Temperature> intakeTemp;

  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Temperature> rollerTemp;

  private final PositionVoltage intakeArmPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage rollerPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final VoltageOut rollerVoltageOut = new VoltageOut(0);

  // Constants for Simulation Physics
  private static final double ARM_GEAR_RATIO = 25.0; // Example: 25:1 reduction
  private static final double ROLLER_GEAR_RATIO = 1.0;

  public IntakeIOSim(SwerveDriveSimulation driveSimulation) {
    // ------------------------------------------------
    // Initialize Hardware (TalonFXs)
    // ------------------------------------------------
    intakeArmMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // SIMULATION PID: These must be tuned for the simulated mass!
    armConfig.Slot0.kP = 20.0;
    armConfig.Slot0.kD = 1.0;
    intakeArmMotor.getConfigurator().apply(armConfig);

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // ------------------------------------------------
    // Initialize Physics Sims
    // ------------------------------------------------
    // 1. Arm Physics (WPILib)
    armPhysicsSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), // Motor type
            ARM_GEAR_RATIO, // Gearing
            SingleJointedArmSim.estimateMOI(0.4, 3.0), // MOI (Length, Mass)
            0.4, // Arm Length (Meters)
            0.0, // Min Angle (Rads) - Stowed
            Math.PI / 2, // Max Angle (Rads) - Deployed
            true, // Simulate Gravity
            0.0 // Starting Angle
            );

    // 2. Roller Physics (WPILib)
    rollerPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(0),
                KilogramSquareMeters.of(0.000190215775).baseUnitMagnitude(),
                ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60(1).withReduction(ROLLER_GEAR_RATIO),
            0.01,
            0.0);

    // 3. Game Piece Logic (IronMaple)
    // Using the factory method as seen in your code snippet
    mapleIntakeSim =
        IntakeSimulation.OverTheBumperIntake(
            CrescendoNoteOnField.CRESCENDO_NOTE_INFO.type(),
            driveSimulation,
            IntakeConstants.INTAKE_WIDTH,
            IntakeConstants.INTAKE_LENGTH,
            IntakeSide.BACK,
            1);

    // ------------------------------------------------
    // Initialize Signals
    // ------------------------------------------------
    intakePosition = intakeArmMotor.getPosition();
    intakeVelocity = intakeArmMotor.getVelocity();
    intakeAppliedVoltage = intakeArmMotor.getMotorVoltage();
    intakeTemp = intakeArmMotor.getDeviceTemp();

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

    intakeArmMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // ------------------------------------------------
    // 1. Step Physics World (WPILib)
    // ------------------------------------------------
    TalonFXSimState armState = intakeArmMotor.getSimState();
    TalonFXSimState rollerState = rollerMotor.getSimState();

    armState.setSupplyVoltage(12.0);
    rollerState.setSupplyVoltage(12.0);

    // Apply voltage from Talon PID to Physics Sim
    armPhysicsSim.setInput(armState.getMotorVoltage());
    armPhysicsSim.update(0.020); // 20ms Loop

    rollerPhysicsSim.setInput(rollerState.getMotorVoltage());
    rollerPhysicsSim.update(0.020);

    // ------------------------------------------------
    // 2. Bridge Physics -> IronMaple Logic
    // ------------------------------------------------
    // We treat the intake as "Active" in IronMaple (collision box extended)
    // when the physical arm is deployed beyond a threshold (e.g. 45 degrees).
    double armAngleDeg = Units.radiansToDegrees(armPhysicsSim.getAngleRads());

    if (armAngleDeg > 45.0) {
      mapleIntakeSim.startIntake();
    } else {
      mapleIntakeSim.stopIntake();
    }

    // ------------------------------------------------
    // 3. Bridge Physics -> Talon Sensors
    // ------------------------------------------------
    // Sync Arm
    double armRotations = Units.radiansToRotations(armPhysicsSim.getAngleRads());
    double armVelocityRotPerSec = Units.radiansToRotations(armPhysicsSim.getVelocityRadPerSec());

    // Convert mechanism rotations back to motor rotations (before gear)
    armState.setRawRotorPosition(armRotations * ARM_GEAR_RATIO);
    armState.setRotorVelocity(armVelocityRotPerSec * ARM_GEAR_RATIO);

    // Sync Roller
    double rollerRotPerSec =
        Units.radiansToRotations(rollerPhysicsSim.getAngularVelocityRadPerSec());
    rollerState.setRotorVelocity(rollerRotPerSec * ROLLER_GEAR_RATIO);
    rollerState.addRotorPosition(rollerRotPerSec * ROLLER_GEAR_RATIO * 0.020);

    // ------------------------------------------------
    // 4. Update IO Inputs
    // ------------------------------------------------
    BaseStatusSignal.refreshAll(
        intakePosition,
        intakeVelocity,
        intakeAppliedVoltage,
        intakeTemp,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVoltage,
        rollerTemp);

    inputs.intakeArmMotorConnected = true;
    inputs.intakeArmPosition = intakePosition.getValue();
    inputs.intakeArmVelocity = intakeVelocity.getValue();
    inputs.intakeArmAppliedVoltage = intakeAppliedVoltage.getValue();
    inputs.intakeArmMotorTemp = intakeTemp.getValue();

    inputs.rollerMotorConnected = true;
    inputs.rollerPosition = rollerPosition.getValue();
    inputs.rollerVelocity = rollerVelocity.getValue();
    inputs.rollerAppliedVoltage = rollerAppliedVoltage.getValue();
    inputs.rollerMotorTemp = rollerTemp.getValue();
  }

  @Override
  public void setIntakePosition(INTAKE_POSITION newPosition) {
    // The real Talon PID runs inside the SimState logic we set up above
    intakeArmMotor.setControl(intakeArmPositionVoltage.withPosition(newPosition.position));
  }

  @Override
  public void setRollerState(ROLLER_STATE newState) {
    switch (newState) {
      case INTAKE:
        rollerMotor.setControl(rollerVoltageOut.withOutput(IntakeConstants.INTAKE_VOLTAGE));
        break;
      case EJECT:
        rollerMotor.setControl(rollerVoltageOut.withOutput(IntakeConstants.EJECT_VOLTAGE));
        break;
      case HOLD:
        rollerMotor.setControl(rollerPositionVoltage.withPosition(rollerPosition.getValue()));
        break;
      case STOP:
        rollerMotor.setControl(rollerVoltageOut.withOutput(0));
        break;
    }
  }
}
