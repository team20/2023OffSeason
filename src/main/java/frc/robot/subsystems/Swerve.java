package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;


/**
 * Drive subsystem composed of all 4 Swerve MK4 modules.
 */
public class Swerve extends SubsystemBase {

    private static final boolean SAMPLE_MODE = false;
    private static final Logger LOGGER = Logger.getLogger(Swerve.class.getName());

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    //private AtomicInteger count = new AtomicInteger();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), 
            getModulePositions());
    }  

    /**
     * Engage the drive motors.
     * 
     * @param translation   
     *   X and Y movement.  The y movement seems to be "strafe"?
     * @param rotation
     *   Rotation of the motor?
     * @param fieldRelative 
     *   If true, plot relative to the competition field using an inertial gyro (?).
     * @param isOpenLoop
     *   If true, assume there is no "closed loop" feedback from the motor sensors and that all inputs are manually drive (and not really a loop at all).
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        LOGGER.info("drive(): t=" + translation + ", r=" + rotation + ", fieldRelative=" + fieldRelative + ", isOpenLoop=" + isOpenLoop);

        // Ignore input and just run a sample command.
        if (SAMPLE_MODE) {
            sampleDriveOpenLoop(false);
        } else {
            driveAllModules(translation, rotation, fieldRelative, isOpenLoop);
        }
    }

    /**
     * Sample drive program to debug the modules and make sure they are behaving.
     */
    private void sampleDriveOpenLoop(boolean fieldRelative) {

        double vxMetersPerSecond = 0.0;
        double vyMetersPerSecond = 0.0;
        double omegaRadiansPerSecond = 0.0;
    
        // TODO what does inverse kinematics mean in the context of the wpi library
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

        // Use the preset kinematics for the robot to figure out how to move the wheels.
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        System.out.println(swerveModuleStates[0].angle);

        // for(SwerveModule mod : mSwerveMods){
        //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        // }
        SwerveModule mod = mSwerveMods[0];
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }

    private void driveAllModules(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        // System.out.println("X: " + translation.getX());
        // System.out.println("Y: " + translation.getY());
        // System.out.println("Rotation: " + rotation);
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
        } else {
            chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        // Use the preset kinematics for the robot to figure out how to move the wheels.
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        System.out.println(swerveModuleStates[0].angle);

        // System.out.println("Swerve module state: " + swerveModuleStates[0]);
        // for(SwerveModule mod : mSwerveMods){
        //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        // }

        SwerveModule mod = mSwerveMods[0];
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
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

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}