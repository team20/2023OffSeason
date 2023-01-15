package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

/**
 * A single Swerve module.
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset; // TODO not used?
    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private SparkMaxAbsoluteEncoder mDriveEncoder;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        /* Drive Encoder Config */
        mDriveEncoder = mDriveMotor.getAbsoluteEncoder(Type.kDutyCycle);
        //lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        
        // Optimize heading angle and speed to minimize wheel movement.  Default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        /*
        else {
            double velocity = Conversions.MPSToSpark(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        */
    }

    private void setAngle(SwerveModuleState desiredState){
        
        //System.out.println("Speed: " + desiredState.speedMetersPerSecond);
        Rotation2d desiredAngle = desiredState.angle;
        
        // Dont turn the module if speed is less then 1%. Prevents Jittering.
        boolean isSpeedLessThanOnePercent = Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01);
        if (isSpeedLessThanOnePercent) {
            desiredAngle = getState().angle;
        }
        
        double angleDeg = desiredAngle.getDegrees();
        mAngleMotor.set(angleDeg); // TODO documentation says this should be in the range [0.0, 1.0].
        //System.out.println("Angle: " + angleDeg);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.sparkToDegrees(angleEncoder.getPosition(), Constants.Swerve.angleGearRatio));
    }

    /**
     * Returns the angle encoder posistion.
     * 
     * @return
     */
    public Rotation2d getCanCoder(){    
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    /*
    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToSpark(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }
    */

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.enableVoltageCompensation(12);
        mAngleMotor.setSmartCurrentLimit(10);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        //resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.enableVoltageCompensation(12);
        mDriveMotor.setSmartCurrentLimit(10);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.sparkToMPS(mDriveEncoder.getVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.sparkToMeters(mDriveEncoder.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}