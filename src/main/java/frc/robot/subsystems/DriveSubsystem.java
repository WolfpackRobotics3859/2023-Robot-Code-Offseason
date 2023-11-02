package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.ejml.data.ZMatrix;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public Field2d field = new Field2d();
    private NetworkTable visionData;
    private boolean wasOdometrySeeded;
    private Alliance alliance;

    public DriveSubsystem() {
        gyro = new PigeonIMU(Constants.SwerveConstants.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
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
        visionData = NetworkTableInstance.getDefault().getTable("limelight");

        SmartDashboard.putData("Field", field);

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        wasOdometrySeeded = false;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
    
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            SmartDashboard.putNumber("Module " + mod.moduleNumber + " Speed Setpoint: ", swerveModuleStates[mod.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Angle Setpoint: ", swerveModuleStates[mod.moduleNumber].angle.getDegrees());
        }
    }    

    public void driveCircle(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
       //TODO
  }   

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(
          Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds));
      }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
        //return new Pose2d(swerveOdometry.getEstimatedPosition().getTranslation(), getYaw());
    }

    public ChassisSpeeds getFieldVelocity() {
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the angle
        // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
        return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()), getYaw().unaryMinus());
      }

      public ChassisSpeeds getRobotVelocity() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()), getYaw().unaryMinus());
      }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void setGyro(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
        resetOdometry(new Pose2d(getPose().getTranslation(), angle));
      }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void sideDrive(double way) {
      drive(new Translation2d(0, 0.3 * way), 0, false, true);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        wasOdometrySeeded = false;
      }

      public Pose3d getVisionPose(NetworkTable visionData) {
        if (visionData.getEntry("tv").getDouble(0) == 0) {
          return null;
        }
        Pose3d robotPose;
        double[] poseComponents;
        if (alliance== Alliance.Blue) {
          poseComponents = visionData.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
          robotPose = new Pose3d(
            poseComponents[0],
            poseComponents[1],
            poseComponents[2],
            new Rotation3d(
              poseComponents[3],
              poseComponents[4],
              poseComponents[5]));
        } else if (alliance == Alliance.Red) {
          if(DriverStation.getAlliance() == Alliance.Blue) {
            poseComponents = visionData.getEntry("botpose_wpired").getDoubleArray(new double[6]);
          robotPose = new Pose3d(
            poseComponents[0],
            poseComponents[1],
            poseComponents[2],
            new Rotation3d(
              Math.toRadians(poseComponents[3]),
              Math.toRadians(poseComponents[4]),
              Math.toRadians(poseComponents[5])));
          } else {
            poseComponents = visionData.getEntry("botpose_wpired").getDoubleArray(new double[6]);
          robotPose = new Pose3d(
            poseComponents[0],
            poseComponents[1],
            poseComponents[2],
            new Rotation3d(
              Math.toRadians(poseComponents[3]),
              Math.toRadians(poseComponents[4]),
              Math.toRadians(poseComponents[5])));
          }
          
        } else {
          return null;
        }
        return robotPose;
      }
    

    @Override
    public void periodic(){
    
        // Seed odometry if this has not been done

        swerveOdometry.update(getYaw(), getModulePositions()); 
        

        field.setRobotPose(swerveOdometry.getEstimatedPosition());


        SmartDashboard.putString("Odometry", swerveOdometry.getEstimatedPosition().toString());
        SmartDashboard.putNumber("tv", visionData.getEntry("tv").getDouble(0));
        SmartDashboard.putNumberArray("botpose_wpiredd", visionData.getEntry("botpose_wpired").getDoubleArray(new double[6]));
        SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro", getYaw().getDegrees());
        //SmartDashboard.putNumber("ppose anle", get);


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}
