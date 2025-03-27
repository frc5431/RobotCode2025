package frc.robot.Subsytems.Drivebase;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TitanFieldCentricFacingAngle implements SwerveRequest {

    public double velocityX;
    public double velocityY;
    public Angle targetHeading = Radians.of(0);
    public PIDController pid;
    public Pigeon2 gyro;
    // the classic WPI_GAIMFBERLMLMNFTCCZZEController

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        double rotationRate = pid.calculate(gyro.getYaw().getValue().in(Radians), targetHeading.in(Radians));

        SmartDashboard.putNumber("gyroRads", gyro.getYaw().getValue().in(Radians));

        double toApplyOmega = rotationRate;

        ChassisSpeeds speeds = ChassisSpeeds
                .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

        SwerveModuleState[] states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(new ModuleRequest()
                    .withDriveRequest(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequest(SteerRequestType.MotionMagicExpo)
                    .withState(states[i])
                    .withEnableFOC(true)
            );
        }

        return StatusCode.OK;
    }

    public TitanFieldCentricFacingAngle withHeading(Angle targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    public TitanFieldCentricFacingAngle withVelocityX(double velocityX) {
        this.velocityX = velocityX;
        return this;
    }

    public TitanFieldCentricFacingAngle withVelocityY(double velocityY) {
        this.velocityY = velocityY;
        return this;
    }

    public TitanFieldCentricFacingAngle withPID(PIDController pid) {
        this.pid = pid;
        this.pid.enableContinuousInput(0, 2 * Math.PI);
        return this;
    }

}