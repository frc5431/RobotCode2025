package frc.team5431.titan.core.misc;

import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;

public class GameField {

  public @Getter static final Distance fieldLength = Units.Inches.of(690.876);
  private @Getter static final Distance halfLength = fieldLength.div(2);
  public static final Distance fieldWidth = Units.Inches.of(317);
  private @Getter static final Distance halfWidth = fieldWidth.div(2);
  private @Getter static final Distance aprilTagWidth = Units.Inches.of(6.50);
  private @Getter @Setter AprilTagFieldLayout aprilTagFieldLayout;

  public static final Distance startingLineX = Units.Inch.of(299.438); // Measured from the inside of starting line

  public @Getter static final Trigger red = new Trigger(() -> isRed());
  public @Getter static final Trigger blue = new Trigger(() -> isBlue());

  public @Getter @Setter Pose2d[] autonFieldElements;
  public @Getter @Setter Pose2d[] autonRedFieldElements;
  public @Getter @Setter Pose2d[] autonBlueFieldElements;

  public @Getter @Setter Pose2d[] primaryFieldElements;
  public @Getter @Setter Pose2d[] primaryRedFieldElements;
  public @Getter @Setter Pose2d[] primaryBlueFieldElements;
  
  public @Getter @Setter Pose2d[] secondaryFieldElements;
  public @Getter @Setter Pose2d[] secondaryRedFieldElements;
  public @Getter @Setter Pose2d[] secondaryBlueFieldElements;

  public @Getter @Setter Pose2d[] endgameFieldElements;
  public @Getter @Setter Pose2d[] endgameRedFieldElements;
  public @Getter @Setter Pose2d[] endgameBlueFieldElements;

  /** Returns {@code true} if the robot is on the blue alliance. */
  public static boolean isBlue() {
    return !isRed();
  }

  /** Returns {@code true} if the robot is on the red alliance. */
  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;

  }

  /**
   * Flip the angle if we are blue, as we are setting things for a red driver
   * station angle
   * This flips the left and right side for aiming purposes
   */
  public static double flipAimAngleIfBlue(double redAngleDegs) {
    if (GameField.isBlue()) {
      return 180 - redAngleDegs;
    }
    return redAngleDegs;
  }

  // This flips the true angle of the robot if we are blue
  public static double flipTrueAngleIfBlue(double redAngleDegs) {
    if (GameField.isBlue()) {
      return (180 + redAngleDegs) % 360;
    }
    return redAngleDegs;
  }

  public static double flipTrueAngleIfRed(double blueAngleDegs) {
    if (GameField.isRed()) {
      return (180 + blueAngleDegs) % 360;
    }
    return blueAngleDegs;
  }

  public static Rotation2d flipAngleIfRed(Rotation2d blue) {
    if (GameField.isRed()) {
      return new Rotation2d(-blue.getCos(), blue.getSin());
    } else {
      return blue;
    }
  }

  public static Pose2d flipXifRed(Pose2d blue) {
    return flipXifRed(new Pose2d(blue.getX(), blue.getTranslation().getY(), blue.getRotation()));
  }

  public static Translation2d flipXifRed(Translation2d blue) {
    return flipXifRed(new Translation2d(blue.getX(), blue.getY()));
  }

  public static Translation3d flipXifRed(Translation3d blue) {
    return flipXifRed(new Translation3d(blue.getX(), blue.getY(), blue.getZ()));
  }

  // If we are red flip the x pose to the other side of the field
  public static Distance flipXifRed(Distance xCoordinate) {
    if (GameField.isRed()) {
      return GameField.fieldLength.minus(xCoordinate);
    }
    return xCoordinate;
  }

  // If we are red flip the y pose to the other side of the field
  public static double flipYifRed(Distance yCoordinate) {
    if (GameField.isRed()) {
      return GameField.fieldWidth.in(Inches) - yCoordinate.in(Inches);
    }
    return yCoordinate.in(Inches);
  }

  public static boolean poseOutOfField(Pose2d pose2D) {
    double x = pose2D.getX();
    double y = pose2D.getY();
    return (x <= 0 || x >= fieldLength.in(Inches)) || (y <= 0 || y >= fieldWidth.in(Inches));
  }

  public static boolean poseOutOfField(Pose3d pose3D) {
    return poseOutOfField(pose3D.toPose2d());
  }

}
