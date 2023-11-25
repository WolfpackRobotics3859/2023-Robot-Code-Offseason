package frc.robot.commands.drive;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class Drive extends Command 
{    
    private Drivetrain s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public Drive(Drivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.03);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.03);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.03);
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }

    @Override
    public void end(boolean interupted) {
        s_Swerve.drive(new Translation2d(), 0, false, true);
    }
}
