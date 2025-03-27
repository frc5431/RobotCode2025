package frc.robot.Util;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Systems;
import lombok.Getter;

public class Field {
	public @Getter static final Distance fieldLength = Units.Inches.of(690.876);
	private @Getter static final Distance halfLength = fieldLength.div(2);
	public static final Distance fieldWidth = Units.Inches.of(317);
	private @Getter static final Distance halfWidth = fieldWidth.div(2);

	public static final Distance startingLineX = Units.Inch.of(299.438); // Measured from the inside of starting
																			// line

	public static class Processor {
		public static final Pose2d centerFace = new Pose2d(Units.Inches.of(235.726), Units.Inches.of(0),
				Rotation2d.fromDegrees(90));
	}

	public static class Barge {
		public static final Translation2d farCage = new Translation2d(Units.Inches.of(345.428),
				Units.Inches.of(286.779));
		public static final Translation2d middleCage = new Translation2d(Units.Inches.of(345.428),
				Units.Inches.of(242.855));
		public static final Translation2d closeCage = new Translation2d(Units.Inches.of(345.428),
				Units.Inches.of(199.947));

		// Measured from floor to bottom of cage
		public static final Distance deepHeight = Units.Inches.of(3.125);
		public static final Distance shallowHeight = Units.Inches.of(30.125);
	}

	public static class CoralStation {
		public static final Pose2d leftCenterFace = new Pose2d(
				Units.Inches.of(33.526),
				Units.Inches.of(291.176),
				Rotation2d.fromDegrees(90 - 144.011));
		public static final Pose2d rightCenterFace = new Pose2d(
				Units.Inches.of(33.526),
				Units.Inches.of(25.824),
				Rotation2d.fromDegrees(144.011 - 90));
	}

	public static final int[] reefAprilTags = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
	public static final int[] redReefAprilTags = { 6, 7, 8, 9, 10, 11 };
	public static final int[] blueReefAprilTags = { 17, 18, 19, 20, 21, 22 };

	public enum FieldBranch {
		A(BranchSide.LEFT, ReefSide.ONE), B(BranchSide.RIGHT, ReefSide.ONE), C(BranchSide.LEFT, ReefSide.TWO), D(
				BranchSide.RIGHT, ReefSide.TWO), E(BranchSide.LEFT, ReefSide.THREE), F(BranchSide.RIGHT,
						ReefSide.THREE), G(BranchSide.LEFT, ReefSide.FOUR), H(BranchSide.RIGHT, ReefSide.FOUR), I(
								BranchSide.LEFT, ReefSide.FIVE), J(BranchSide.RIGHT, ReefSide.FIVE), K(BranchSide.LEFT,
										ReefSide.SIX), L(BranchSide.RIGHT, ReefSide.SIX);

		public SimpleBranch simpleBranchInfo;

		private FieldBranch(BranchSide branchSide, ReefSide reefSide) {
			this.simpleBranchInfo = new SimpleBranch(branchSide, reefSide);
		}
	}

	public record SimpleBranch(BranchSide branchSide, ReefSide reefSide) {
		public SimpleBranch mirror() {
			// TODO check if mirroring the branchside does work here
			return new SimpleBranch(branchSide.mirror(), reefSide.mirror());
		}
	}

	public enum BranchSide {
		LEFT(new Translation2d(Inches.of(0), Inches.of(0))), 
		RIGHT(new Translation2d(Inches.of(0), Inches.of(0))),
		MIDDLE(new Translation2d(0, 0));

		public Translation2d tagOffset;

		private BranchSide(Translation2d offsets) {
			tagOffset = offsets;
		}

		public BranchSide mirror() {
			switch (this) {
				case LEFT:
					return RIGHT;
				default:
					return LEFT;
			}
		}
	}

	public enum ReefSide {
		ONE(18, 7), SIX(19, 6), FIVE(20, 11), FOUR(21, 10), THREE(22, 9), TWO(17, 8);

		public final Pose2d redTagPose;
		public final Pose2d blueTagPose;

		public Pose2d getCurrent() {
			return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blueTagPose : redTagPose;
		}

		public ReefSide mirror() {
			switch (this) {
				case ONE:
					return ONE;
				case TWO:
					return SIX;
				case THREE:
					return FIVE;
				case FOUR:
					return FOUR;
				case FIVE:
					return THREE;
				default:
					return TWO; // SIX case
			}
		}

		private ReefSide(int blue, int red) {
			var layout = Systems.getLayout();

			redTagPose = layout.getTagPose(red).get().toPose2d();
			blueTagPose = layout.getTagPose(blue).get().toPose2d();
		}
	}

	public static class Reef {
		public static final Translation2d center = new Translation2d(Units.Inches.of(176.746),
				Units.Inches.of(158.501));
		public static final Distance faceToZoneLine = Units.Inches.of(12); // Side of the reef to the inside of
																			// the reef zone line

		public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in
																	// clockwise order
		public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(); // Starting at
																								// the right
																								// branch facing
																								// the driver
																								// station in
		// clockwise

		static {
			// Initialize faces
			centerFaces[0] = new Pose2d(
					Units.Inches.of(144.003),
					Units.Inches.of(158.500),
					Rotation2d.fromDegrees(180));
			centerFaces[1] = new Pose2d(
					Units.Inches.of(160.373),
					Units.Inches.of(186.857),
					Rotation2d.fromDegrees(120));
			centerFaces[2] = new Pose2d(
					Units.Inches.of(193.116),
					Units.Inches.of(186.858),
					Rotation2d.fromDegrees(60));
			centerFaces[3] = new Pose2d(
					Units.Inches.of(209.489),
					Units.Inches.of(158.502),
					Rotation2d.fromDegrees(0));
			centerFaces[4] = new Pose2d(
					Units.Inches.of(193.118),
					Units.Inches.of(130.145),
					Rotation2d.fromDegrees(-60));
			centerFaces[5] = new Pose2d(
					Units.Inches.of(160.375),
					Units.Inches.of(130.144),
					Rotation2d.fromDegrees(-120));

			// Initialize branch positions
			for (int face = 0; face < 6; face++) {
				Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
				Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
				for (var level : ReefHeight.values()) {
					Pose2d poseDirection = new Pose2d(center,
							Rotation2d.fromDegrees(180 - (60 * face)));
					Distance adjustX = Units.Inches.of(30.738);
					Distance adjustY = Units.Inches.of(6.469);

					fillRight.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(
															new Transform2d(
																	adjustX.in(Units.Meters),
																	adjustY.in(Units.Meters),
																	new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(
															new Transform2d(
																	adjustX.in(Units.Meters),
																	adjustY.in(Units.Meters),
																	new Rotation2d()))
													.getY(),
											level.height.in(Inches)),
									new Rotation3d(
											0,
											level.pitch.in(Units.Radians),
											poseDirection.getRotation()
													.getRadians())));

					fillLeft.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(
															new Transform2d(
																	adjustX,
																	adjustY.div(-1),
																	new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(
															new Transform2d(
																	adjustX,
																	adjustY.div(-1),
																	new Rotation2d()))
													.getY(),
											level.height.in(Inches)),
									new Rotation3d(
											0,
											level.pitch.in(Units.Radians),
											poseDirection.getRotation()
													.getRadians())));
				}
				branchPositions.add((face * 2) + 1, fillRight);
				branchPositions.add((face * 2) + 2, fillLeft);
			}
		}
	}

	public static class StagingPositions {
		// Measured from the center of the ice cream
		public static final Pose2d leftIceCream = new Pose2d(Units.Inches.of(48), Units.Inches.of(230.5),
				new Rotation2d());
		public static final Pose2d middleIceCream = new Pose2d(Units.Inches.of(48), Units.Inches.of(158.5),
				new Rotation2d());
		public static final Pose2d rightIceCream = new Pose2d(Units.Inches.of(48), Units.Inches.of(86.5),
				new Rotation2d());
	}

	public enum ReefHeight {
		L4(Units.Inches.of(72), Units.Degrees.of(-90)), L3(Units.Inches.of(47.625), Units.Degrees.of(-35)), L2(
				Units.Inches.of(31.875),
				Units.Degrees.of(-35)), L1(Units.Inches.of(18), Units.Degrees.of(0));

		ReefHeight(Distance height, Angle pitch) {
			this.height = height;
			this.pitch = pitch; // in degrees
		}

		public final Distance height;
		public final Angle pitch;
	}

	@Getter
	private static final Distance aprilTagWidth = Units.Inches.of(6.50);

	public static boolean isReef(double id) {
		for (double x : reefAprilTags) {
			if (x == id) {
				return true;
			}
		}
		return false;
	}

	public static boolean isRedTag(double id) {
		return id < 12;
	}

	/** Returns {@code true} if the robot is on the blue alliance. */
	public static boolean isBlue() {
		//TODO ADD NULL CEHCIGN
		return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
	}

	/** Returns {@code true} if the robot is on the red alliance. */
	public static boolean isRed() {
		return !isBlue();
	}

	public @Getter static final Trigger red = new Trigger(() -> isRed());
	public @Getter static final Trigger blue = new Trigger(() -> isBlue());

	/**
	 * Flip the angle if we are blue, as we are setting things for a red driver
	 * station angle
	 * This flips the left and right side for aiming purposes
	 */
	public static double flipAimAngleIfBlue(double redAngleDegs) {
		if (Field.isBlue()) {
			return 180 - redAngleDegs;
		}
		return redAngleDegs;
	}

	// This flips the true angle of the robot if we are blue
	public static double flipTrueAngleIfBlue(double redAngleDegs) {
		if (Field.isBlue()) {
			return (180 + redAngleDegs) % 360;
		}
		return redAngleDegs;
	}

	public static double flipTrueAngleIfRed(double blueAngleDegs) {
		if (Field.isRed()) {
			return (180 + blueAngleDegs) % 360;
		}
		return blueAngleDegs;
	}

	public static Rotation2d flipAngleIfRed(Rotation2d blue) {
		if (Field.isRed()) {
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
		if (Field.isRed()) {
			return Field.fieldLength.minus(xCoordinate);
		}
		return xCoordinate;
	}

	// If we are red flip the y pose to the other side of the field
	public static double flipYifRed(Distance yCoordinate) {
		if (Field.isRed()) {
			return Field.fieldWidth.in(Inches) - yCoordinate.in(Inches);
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

	private AprilTagFieldLayout aprilTagFieldLayout;

	public Pose3d getAprilTagPose3d(int aprilTagID) {
		for (double x : reefAprilTags) {
			if (x == aprilTagID) {
				return aprilTagFieldLayout.getTagPose(aprilTagID).orElse(null);
			}
		}
		return null;
	}
}