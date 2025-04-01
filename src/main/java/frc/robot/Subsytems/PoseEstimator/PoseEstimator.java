package frc.robot.Subsytems.PoseEstimator;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systems;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Limelight.Vision;
import frc.robot.Util.Field;
import frc.robot.Util.Constants.VisionConstants;
import lombok.Getter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimator extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
  //(1.5, 1.5, 1.5);

  private Supplier<Rotation2d> rotationSupplier;
  private Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private @Getter SwerveDrivePoseEstimator poseEstimator;
  private final Drivebase drivebase = Systems.getDrivebase();
  private final Vision vision = Systems.getVision();

  StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision Pose", Pose2d.struct).publish();
  // private final PhotonRunnable photonEstimator = new PhotonRunnable();
  // private final Notifier photonNotifier = new Notifier(photonEstimator);

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  public PoseEstimator(
      Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier) {

    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;
    poseEstimator = new SwerveDrivePoseEstimator(
        drivebase.getKinematics(),
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        drivebase.getRobotPose(),
        stateStdDevs,
        visionMeasurementStdDevs);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * 
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system,
      // the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getCurrentPose());
      poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());
    visionPosePublisher.set(poseEstimator.getEstimatedPosition());

    //TODO: this is where we left off last night, ading isreef
    if (vision.getBestLimelight().targetInView() && Field.isReef(vision.getBestLimelight().getClosestTagID())) { // Check to see if new tag was seen
      // New pose from vision
      sawTag = true;
      // var pose2d = visionPose.estimatedPose.toPose2d();
      var pose2d = vision.getBestLimelight().getRawPose3d().toPose2d();
      // if (originPosition != kBlueAllianceWallRightSide) {
      //   pose2d = flipAlliance(pose2d);
      // }

      // TODO: Need to filter out the poses
      // TODO: Kill Theta STDs
      // May want to use vision.addFilteredVisionInput
      poseEstimator.setVisionMeasurementStdDevs(Systems.getVision().visionStdMatrix);
      pose2d = Systems.getDrivebase().keepPoseOnField(pose2d);
      pose2d = new Pose2d(pose2d.getTranslation(), Systems.getDrivebase().getRobotHeading());
      poseEstimator.addVisionMeasurement(pose2d, Timer.getFPGATimestamp());
    }
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetRotablion() {
    poseEstimator.resetRotation(Rotation2d.kZero);
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()));
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
   * always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates
   * for each alliance.
   * 
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
  }

  public Command testcommand() {
    return new InstantCommand(() -> this.commandprint());
  }

  public void commandprint() {
    System.out.println("Field reset");
    // this.resetFieldPosition();
    // drivebase.resetRotation(new Rotation2d(0.0));
    // drivebase.resetTranslation(new Translation2d(0.0, new Rotation2d(0.0)));
    drivebase.resetPose(new Pose2d(0.0, .0, new Rotation2d(0.0)));
  }

}