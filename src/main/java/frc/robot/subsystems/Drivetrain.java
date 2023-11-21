package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public Field2d field = new Field2d();
    public Command autoCommandOne;

    public Drivetrain() 
    {
      gyro = new PigeonIMU(Constants.SwerveConstants.pigeonID);
      gyro.configFactoryDefault();
      zeroGyro();

      mSwerveMods = new SwerveModule[] 
      {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
      };

      /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        */
      Timer.delay(1.0);
      resetModulesToAbsolute();

      //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

      SmartDashboard.putData("Field", field);

      swerveOdometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

      AutoBuilder.configureHolonomic
      (
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0),
          4.5,
          Units.inchesToMeters(14.3896),
          new ReplanningConfig()
        ),
        this
      );
      this.resetOdometry(new Pose2d(2.45, 4.39, getYaw()));
    }

    public void driveRobotRelative(ChassisSpeeds speeds)
    {
      SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

      for(SwerveModule mod : mSwerveMods)
      {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Speed Setpoint: ", swerveModuleStates[mod.moduleNumber].speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Angle Setpoint: ", swerveModuleStates[mod.moduleNumber].angle.getDegrees());
      }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
      SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

      for(SwerveModule mod : mSwerveMods)
      {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Speed Setpoint: ", swerveModuleStates[mod.moduleNumber].speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Angle Setpoint: ", swerveModuleStates[mod.moduleNumber].angle.getDegrees());
      }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
      
      for(SwerveModule mod : mSwerveMods)
      {
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      }
    }    

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) 
    {
      setModuleStates(Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Pose2d getPose() 
    {
      return new Pose2d(swerveOdometry.getEstimatedPosition().getTranslation(), getYaw());
    }

    public ChassisSpeeds getFieldVelocity()
    {
      // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
      // but not the reverse.  However, because this transform is a simple rotation, negating the angle
      // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
      return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()), getYaw().unaryMinus());
    }

    public ChassisSpeeds getRobotVelocity()
    {
      return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()), getYaw().unaryMinus());
    }

    public void resetOdometry(Pose2d pose)
    {
      swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates()
    {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(SwerveModule mod : mSwerveMods){
        states[mod.moduleNumber] = mod.getState();
      }
      return states;
    }

    public SwerveModulePosition[] getModulePositions()
    {
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(SwerveModule mod : mSwerveMods)
      {
        positions[mod.moduleNumber] = mod.getPosition();
      }
      return positions;
    }

    public void zeroGyro()
    {
        gyro.setYaw(0);
    }

    public void setGyro(Rotation2d angle) 
    {
        gyro.setYaw(angle.getDegrees());
        resetOdometry(new Pose2d(getPose().getTranslation(), angle));
      }

    public Rotation2d getYaw() 
    {
      double[] ypr = new double[3];
      gyro.getYawPitchRoll(ypr);
      return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public void resetModulesToAbsolute(){
      for(SwerveModule mod : mSwerveMods)
      {
        mod.resetToAbsolute();
      }
    }

    public void sideDrive(double way) 
    {
      drive(new Translation2d(0, 0.3 * way), 0, false, true);
    }

    @Override
    public void periodic()
    {
      // Seed odometry if this has not been done
      swerveOdometry.update(getYaw(), getModulePositions()); 
      
      field.setRobotPose(swerveOdometry.getEstimatedPosition());

      SmartDashboard.putString("Odometry", swerveOdometry.getEstimatedPosition().toString());
      SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("gyro", getYaw().getDegrees());
      SmartDashboard.putNumber("roll", gyro.getRoll());
      for(SwerveModule mod : mSwerveMods)
      {
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
      }
    }
}
