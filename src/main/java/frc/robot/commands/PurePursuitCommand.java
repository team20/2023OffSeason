package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PurePursuitCommand extends CommandBase {
  /** Creates a new PurePursuitFollowerCommand. */

  private Trajectory m_splinePath;
  private double lookahead_distance = 0.5;
  private double m_lastGuess = 0;
  private Pose2d m_finalPoint;
  private boolean m_reversed;
  private double speed = 0.3;
  private int m_numLoops = 0;
  private Pose2d m_goalPoint;
  private double m_driveSpeed;
  public PurePursuitCommand(boolean reversed, Trajectory path) {
    m_reversed = reversed;
    m_splinePath = path;
    m_finalPoint = m_splinePath.sample(m_splinePath.getTotalTimeSeconds()).poseMeters;
    addRequirements(DriveSubsystem.get());
  }

  public PurePursuitCommand(Trajectory path) {
 
    m_splinePath = path;
    m_finalPoint = m_splinePath.sample(m_splinePath.getTotalTimeSeconds()).poseMeters;
    addRequirements(DriveSubsystem.get());
  }

  public PurePursuitCommand(double speed, Trajectory path) {
    this.speed = speed;
    m_splinePath = path;
    m_finalPoint = m_splinePath.sample(m_splinePath.getTotalTimeSeconds()).poseMeters;
    addRequirements(DriveSubsystem.get());
  }

  public PurePursuitCommand(double speed, boolean reversed, Trajectory path) {
    this.speed = speed;
    m_reversed = reversed;
    m_splinePath = path;
    m_finalPoint = m_splinePath.sample(m_splinePath.getTotalTimeSeconds()).poseMeters;
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_numLoops%5 == 0){
      m_goalPoint = getGoalPoint(DriveSubsystem.get().getPose(), m_lastGuess, m_splinePath);
    }
    double turn = getTurn(m_goalPoint, DriveSubsystem.get().getPose());
    DriveSubsystem.get().arcadeDrive(m_driveSpeed,turn);
    m_numLoops++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_finalPoint.getX() - DriveSubsystem.get().getPose().getX()) < 0.1
        && Math.abs(m_finalPoint.getY() - DriveSubsystem.get().getPose().getY()) < 0.1) {
      return true;
    }
    return false;
  }

  private Pose2d getGoalPoint(Pose2d robotLocation, double lastGuess, Trajectory spline) {

    //robot location
    double robotX = robotLocation.getX();
    double robotY = robotLocation.getY();
    double lowerBound, upperBound, checkT, xDisplacement, yDisplacement, ft;
    Pose2d checkVal;
    
    //upper bound of search starts at the end of the spline
    //lower bound starts at the time value of the last guess, at start we set lastguess to 0
    upperBound = spline.getTotalTimeSeconds();
    lowerBound = lastGuess;


    int count = 1;
    
    //perform a binary search to find a point on the spline at the lookahead distance from the robot
    // making a few assumptions here:
    //1. We start within lookahead distance of the spline
    //2. The next point we're getting is ahead of the last point we got
    do {
      //time value we're checking is the middle between the upper and lower time bounds of our search, in line with a binary search
      checkT = (upperBound + lowerBound) / 2;

      //get the spline point at the time we're checking
      checkVal = spline.sample(checkT).poseMeters;

      //compute x and y displacement of the spline point from our 
      xDisplacement = checkVal.getX() - robotX;
      yDisplacement = checkVal.getY() - robotY;


      //distance from the current location of the robot to the point we're checking on the spline is given by sqrt((pointx-robotx)^2 + (pointx-robotx)^2)
      //by subtracting the square of the lookahead distance from the square of the distance between the robot and the point, we can say whether our current point is inside or outside the lookahead distance
      ft = xDisplacement * xDisplacement + yDisplacement * yDisplacement - lookahead_distance * lookahead_distance;

      if (ft < 0) {
        //ft<0 = point is inside circle, as lookahead distance > than distance to point, so we use the time value of our current guess as the new lower bound of our search
        lowerBound = checkT;
      } else {  
        //ft>0 = point is inside circle, as lookahead distance > than distance to point, so we use the time value of our current guess as the new lower bound of our search
        upperBound = checkT;
      }

      //if it's been more than 10 iterations, just use the last point we got as an answer, since it'll probably be close enough
      if (count > 10) {
        checkVal = spline.sample(lastGuess).poseMeters;
        checkT = lastGuess;
        System.out.println("failed to get goal point, ft="+ft);
        break;
      }
      ++count;

    //stop if we've found an acceptable point or after 10 iterations
    } while (Math.abs(ft) > 0.05);

    //save the time value of our last guess for the next time we use this function
    m_lastGuess = checkT;
    
    //return our point!
    return checkVal;
  }

  private double getTurn(Pose2d lookaheadPoint, Pose2d currPoint) {

    // just the distance formula
    double vectorDistance = lookahead_distance;
    double vectorAngle = Math.atan2((lookaheadPoint.getY() - currPoint.getY()),
        (lookaheadPoint.getX() - currPoint.getX()));
    double robotAngle = DriveSubsystem.get().getHeading();
    double angleChangeRadians = robotAngle;
    double vectorAngleAdjusted = vectorAngle - angleChangeRadians;
    double x = vectorDistance * Math.cos(vectorAngleAdjusted);

    if(x>0){
      m_driveSpeed = speed;
      return Math.sin(vectorAngleAdjusted);
    }else{
      m_driveSpeed = 0.1;
      return Math.signum(Math.sin(vectorAngleAdjusted))*1;
    }

  }
}