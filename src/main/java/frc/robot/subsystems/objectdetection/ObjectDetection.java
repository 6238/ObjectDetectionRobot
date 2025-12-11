package frc.robot.subsystems.objectdetection;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO.TargetObservation;

public class ObjectDetection extends SubsystemBase {
  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs;
  private final Drive driveSubsystem;

  private ArrayList<Pose2d> objectPoses;

  public ObjectDetection(ObjectDetectionIO io, Drive driveSubsystem) {
    this.io = io;
    inputs = new ObjectDetectionIOInputsAutoLogged();
    this.driveSubsystem = driveSubsystem;

    objectPoses = new ArrayList<>();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    objectPoses.clear();
    for (TargetObservation target : inputs.targetObservations) {
      Optional<Pose2d> targetPose = getObjectWorldPose(target);
      if (targetPose.isEmpty()) {
        continue;
      }

      objectPoses.add(targetPose.get());
    }
  }

  public Optional<Pose2d> getObjectWorldPose(TargetObservation observation) {
    Optional<Pose2d> robotPose2d = driveSubsystem.getTimestampPose(observation.timestamp());
    if (robotPose2d.isEmpty()) {
      return Optional.empty();
    }

    Pose2d transformedPose = robotPose2d.get()
      .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
      .transformBy(new Transform2d(
        new Translation2d(observation.dx(), observation.dy()),
        new Rotation2d(0.0)
      ));

    return Optional.of(transformedPose);
  }
}
