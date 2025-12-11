package frc.robot.subsystems.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public TargetObservation[] targetObservations = new TargetObservation[0];
  }

  public static record TargetObservation(
      double dx, double dy, double confidence, double targetArea, double timestamp) {}

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
