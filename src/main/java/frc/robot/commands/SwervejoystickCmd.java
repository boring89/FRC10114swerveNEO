package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwervejoystickCmd extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;


    public SwervejoystickCmd(SwerveSubsystem swerveSubsystem, 
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
            Supplier<Boolean> fieldOrientedfunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedfunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetEncoder();
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        //Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            //Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            //Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
