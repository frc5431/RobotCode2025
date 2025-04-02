package frc.robot.Commands.Chained;

import static frc.robot.Util.Constants.AutonConstants.THETA_kD;
import static frc.robot.Util.Constants.AutonConstants.THETA_kI;
import static frc.robot.Util.Constants.AutonConstants.THETA_kP;
import static frc.robot.Util.Constants.AutonConstants.D_kD;
import static frc.robot.Util.Constants.AutonConstants.D_kI;
import static frc.robot.Util.Constants.AutonConstants.D_kP;
import static frc.robot.Util.Constants.AutonConstants.TRANSLATION_TOLERANCE;
import static frc.robot.Util.Constants.AutonConstants.THETA_TOLERANCE;
import static frc.robot.Util.Constants.DrivebaseConstants.AutonMaxAngularRate;
import static frc.robot.Util.Constants.DrivebaseConstants.AutonMaxVelocity;
import static frc.robot.Util.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Systems;
import frc.robot.Subsytems.Drivebase.Drivebase;

/**
 * Command to drive to a pose.
 */

public class DriveToPoseCommand extends Command {

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    25, // TODO: Convert this to double - AutonMaxVelocity * 0.5,
    5.0); // Convert this to double as well - nAutonMaxVelocity);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    25, // TODO: Convert these to double - AutonMaxAngularRate * 0.4,
    5.0); //AutonMaxAngularRate);

  // private final ProfiledPIDController dController;
  private final ProfiledPIDController thetaController;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;

  private Drivebase drivebase = Systems.getDrivebase();
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final int turnFirst;

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        // LEDSubsystem ledSubsystem,
        int turnFirst) {
    this(drivebase, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, turnFirst);
  }

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        // LEDSubsystem ledSubsystem,
        int turnFirst) {
    this.drivebase = drivebase;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.turnFirst = turnFirst;

    // dController = new ProfiledPIDController(D_kP, D_kI, D_kD, xyConstraints);
    // dController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    xController = new ProfiledPIDController(D_kP, D_kI, D_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController = new ProfiledPIDController(D_kP, D_kI, D_kD, xyConstraints);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    addRequirements(drivebase);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;

    thetaController.setGoal(pose.getRotation().getRadians());
    // dController.setGoal(0.0);

    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  
  }

  public boolean atGoal() {
    // return dController.atGoal() && thetaController.atGoal();
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    // dController.reset(Math.sqrt(Math.pow(goalPose.getX() - robotPose.getX(), 2) + Math.pow(goalPose.getY() - robotPose.getY(), 2)));

    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // @Override
  // public void execute() {

  //   var robotPose = poseProvider.get();// Drive to the goal

  //   double distance = Math.sqrt(Math.pow(goalPose.getX() - robotPose.getX(), 2) + Math.pow(goalPose.getY() - robotPose.getY(), 2));
  //   double x_diff = (goalPose.getX() - robotPose.getX()) / distance;
  //   double y_diff = (goalPose.getY() - robotPose.getY()) / distance;

  //   var distancePower = dController.calculate(distance);
  //   if (dController.atGoal()) {
  //     distancePower = 0;
  //   }

  //   var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
  //   if (thetaController.atGoal()) {
  //     omegaSpeed = 0;
  //   }

  //   if(turnFirst == 1){
  //     if(!thetaController.atGoal()){
  //       distancePower = 0;
  //     }
  //   }else if(turnFirst==2){
  //     if(!dController.atGoal()){
  //       omegaSpeed = 0;
  //     }
  //   }

  //   // Translate the power into the unit direction for x and y
  //   double xSpeed = distancePower * x_diff;
  //   double ySpeed = distancePower * y_diff;


  //   drive(xSpeed, ySpeed, omegaSpeed);

  //   SmartDashboard.putNumber("ZZ_X_diff", x_diff);
  //   SmartDashboard.putNumber("ZZ_y_diff", y_diff);
  //   SmartDashboard.putNumber("ZZ_diff", distance);
  //   SmartDashboard.putNumber("ZZ_theta_speed", omegaSpeed);
  //   SmartDashboard.putString("ZZ_PathFinised", "Still Running");
  // }

  @Override 
  public void execute(){
    var robotPose = poseProvider.get();// Drive to the goal

    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }
  

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    if(turnFirst == 1){
      if(!thetaController.atGoal()){
        xSpeed = 0;
        ySpeed = 0;
      }
    }else if(turnFirst==2){
      if(!xController.atGoal() & !yController.atGoal()){
        omegaSpeed = 0;
      }
    }

    double max_speed = 0.5;
    drive(Math.min(Math.max(-xSpeed, -max_speed), max_speed), Math.min(Math.max(-ySpeed, -max_speed), max_speed), omegaSpeed);
    
    SmartDashboard.putNumber("ZZ_X_diff", xSpeed);
    SmartDashboard.putNumber("ZZ_y_diff", ySpeed);
    SmartDashboard.putNumber("ZZ_theta_speed", omegaSpeed);
    SmartDashboard.putString("ZZ_PathFinised", "Still Running");
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive(0, 0, 0);
    SmartDashboard.putString("ZZ_PathFinised", "All Done");
  }

  public void drive(double xSpeed, double ySpeed, double omegaSpeed){
    // drivebase.setControl(drivebase.getDriveFieldCentric()
    drivebase.setControl(
      // drivebase.getDriveFieldCentric()
      drivebase.getDriverFOControl()
      .withVelocityX(xSpeed)  // Set X velocity (forward/backward speed in m/s)
      .withVelocityY(ySpeed)  // Set Y velocity (sideways speed in m/s)
      .withRotationalRate(omegaSpeed)
      );
  }

}