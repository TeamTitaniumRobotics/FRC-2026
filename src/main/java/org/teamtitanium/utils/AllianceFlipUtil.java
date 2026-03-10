// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.teamtitanium.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
  public static double applyX(double x, boolean force) {
    return shouldFlip() || force ? FieldConstants.fieldLength - x : x;
  }

  public static double applyX(double x) {
    return applyX(x, false);
  }

  public static double applyY(double y, boolean force) {
    return shouldFlip() || force ? FieldConstants.fieldWidth - y : y;
  }

  public static double applyY(double y) {
    return applyY(y, false);
  }

  public static Translation2d apply(Translation2d translation, boolean force) {
    return new Translation2d(applyX(translation.getX(), force), applyY(translation.getY(), force));
  }

  public static Translation2d apply(Translation2d translation) {
    return apply(translation, false);
  }

  public static Rotation2d apply(Rotation2d rotation, boolean force) {
    return shouldFlip() || force ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return apply(rotation, false);
  }

  public static Pose2d apply(Pose2d pose, boolean force) {
    return shouldFlip() || force
        ? new Pose2d(apply(pose.getTranslation(), force), apply(pose.getRotation(), force))
        : pose;
  }

  public static Pose2d apply(Pose2d pose) {
    return apply(pose, false);
  }

  public static Rectangle2d apply(Rectangle2d rectangle, boolean force) {
    return new Rectangle2d(
        apply(rectangle.getCenter(), force), rectangle.getXWidth(), rectangle.getYWidth());
  }

  public static Rectangle2d apply(Rectangle2d rectangle) {
    return apply(rectangle, false);
  }

  public static Translation3d apply(Translation3d translation, boolean force) {
    return new Translation3d(
        applyX(translation.getX(), force), applyY(translation.getY(), force), translation.getZ());
  }

  public static Translation3d apply(Translation3d translation) {
    return apply(translation, false);
  }

  public static Rotation3d apply(Rotation3d rotation, boolean force) {
    return shouldFlip() || force ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return apply(rotation, false);
  }

  public static Pose3d apply(Pose3d pose, boolean force) {
    return new Pose3d(apply(pose.getTranslation(), force), apply(pose.getRotation(), force));
  }

  public static Pose3d apply(Pose3d pose) {
    return apply(pose, false);
  }

  public static boolean shouldFlip() {
    return !Constants.disableHAL
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
