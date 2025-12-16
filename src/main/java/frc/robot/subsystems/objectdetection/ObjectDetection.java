package frc.robot.subsystems.objectdetection;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO.TargetObservation;
import frc.robot.util.MovingAveragePoseFilter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleFunction;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs;
  private final DoubleFunction<Optional<Pose2d>> timestampPoseFunction;

  private ArrayList<TrackedObject> trackedObjects;

  public ObjectDetection(
      ObjectDetectionIO io, DoubleFunction<Optional<Pose2d>> timestampPoseFunction) {
    this.io = io;
    inputs = new ObjectDetectionIOInputsAutoLogged();

    trackedObjects = new ArrayList<TrackedObject>();
    this.timestampPoseFunction = timestampPoseFunction;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    Arrays.stream(inputs.targetObservations)
        .parallel()
        .forEach(
            target ->
                getObjectWorldPose(target)
                    .ifPresent(
                        pose -> {
                          closestTrackedObject(pose)
                              .ifPresentOrElse(
                                  closestObject -> {
                                    if (closestObject
                                            .getPose()
                                            .getTranslation()
                                            .getDistance(pose.getTranslation())
                                        < ObjectDetectionConstants.OBJECT_ASSOCIATION_DISTANCE.abs(
                                            Meters)) {
                                      closestObject.update(pose, target.timestamp());
                                    } else {
                                      trackedObjects.add(
                                          new TrackedObject(pose, target.timestamp()));
                                    }
                                  },
                                  () -> {
                                    trackedObjects.add(new TrackedObject(pose, target.timestamp()));
                                  });
                        }));

    double currentTime = Timer.getTimestamp();
    trackedObjects.removeIf(
        trackedObject ->
            (currentTime - trackedObject.getLastSeenTimestamp())
                > ObjectDetectionConstants.OBJECT_HISTORY_TIMEOUT.abs(Seconds));

    Pose2d[] trackedObjectPoses =
        trackedObjects.stream().map(TrackedObject::getPose).toArray(Pose2d[]::new);

    Logger.recordOutput("ObjectDetection/ObjectPoses", trackedObjectPoses);
  }

  public Optional<Pose2d> closestTrackedObjectPose(Pose2d currentPose) {
    if (trackedObjects.isEmpty()) {
      return Optional.empty();
    }

    Pose2d closestPose = null;
    double minDistance = Double.MAX_VALUE;
    for (TrackedObject trackedObject : trackedObjects) {
      Pose2d objectPose = trackedObject.filteredPose;
      double distance = currentPose.getTranslation().getDistance(objectPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestPose = trackedObject.filteredPose;
      }
    }

    return Optional.of(closestPose);
  }

  public Optional<TrackedObject> closestTrackedObject(Pose2d pose) {
    if (trackedObjects.isEmpty()) {
      return Optional.empty();
    }

    TrackedObject closestObject = null;
    double minDistance = Double.MAX_VALUE;
    for (TrackedObject trackedObject : trackedObjects) {
      Pose2d objectPose = trackedObject.filteredPose;
      double distance = pose.getTranslation().getDistance(objectPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestObject = trackedObject;
      }
    }

    if (closestObject == null) {
      return Optional.empty();
    }

    return Optional.of(closestObject);
  }

  public TrackedObject[] getTrackedObjects() {
    return trackedObjects.toArray(TrackedObject[]::new);
  }

  public Pose2d[] getTrackedObjectPoses() {
    return trackedObjects.stream().map(TrackedObject::getPose).toArray(Pose2d[]::new);
  }

  public Optional<Pose2d> getObjectWorldPose(TargetObservation observation) {
    Optional<Pose2d> robotPose2d = timestampPoseFunction.apply(observation.timestamp());
    if (robotPose2d.isEmpty()) {
      return Optional.empty();
    }

    Pose2d transformedPose =
        robotPose2d
            .get()
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
            .transformBy(
                new Transform2d(
                    new Translation2d(observation.dx(), observation.dy()), new Rotation2d(0.0)));

    return Optional.of(transformedPose);
  }

  public class TrackedObject {
    private Pose2d filteredPose;
    private double lastSeenTimestamp;

    private MovingAveragePoseFilter poseFilter =
        new MovingAveragePoseFilter(ObjectDetectionConstants.POSE_FILTER_TAPS);

    public TrackedObject(Pose2d pose, double timestamp) {
      lastSeenTimestamp = timestamp;

      filteredPose = poseFilter.calculate(pose);
    }

    public void update(Pose2d newPose, double newTimestamp) {
      lastSeenTimestamp = newTimestamp;

      filteredPose = poseFilter.calculate(newPose);
    }

    public Pose2d getPose() {
      return filteredPose;
    }

    public double getLastSeenTimestamp() {
      return lastSeenTimestamp;
    }
  }
}
