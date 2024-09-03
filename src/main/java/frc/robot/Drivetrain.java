package frc.robot;

import java.util.ArrayList;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Drivetrain {
  public static final double fieldWidth = 8.0137; // The width of the field in meters. Used to translate between Blue and Red coordinate systems.
  public static final double maxVelTeleop = 5.27; // User defined maximum speed of the robot. Enforced during teleop. Unit: meters per second Robot maximum is 4 m/s.
  public static final double maxAngularVelTeleop = 5.27*Math.PI; // User defined maximum rotational speed of the robot. Enforced during teleop. Unit: raidans per second Robot maximum is 4pi rad/s.
  public static final double maxAccTeleop = 10.0; // User defined maximum acceleration of the robot. Enforced during teleop. Unit: meters per second^2 Robot maximum is 5 m/s2.
  public static final double maxAngularAccTeleop = 10.0*Math.PI; // User defined maximum rotational acceleration of the robot. Enforced during teleop. Unit: raidans per second^2 Robot maximum is 5pi rad/s2.
  public static final double maxVelAuto = 5.27; // User defined maximum speed of the robot. Enforced during auto. Unit: meters per second
  public static final double maxAngularVelAuto = 5.27*Math.PI; // User defined maximum rotational speed of the robot. Enforced during auto. Unit: raidans per second
  public static final double maxAccAuto = 7.0; // User defined maximum acceleration of the robot. Enforced during auto. Unit: meters per second^2
  public static final double maxAngularAccAuto = 7.0*Math.PI; // User defined maximum rotational acceleration of the robot. Enforced during auto. Unit: raidans per second^2

  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left. Units: meters.
  private static final Translation2d frontLeftModulePos = new Translation2d(0.30162, 0.22542);
  private static final Translation2d frontRightModulePos = new Translation2d(0.30162, -0.22542); 
  private static final Translation2d backRightModulePos = new Translation2d(-0.30162, -0.22542);
  private static final Translation2d backLeftModulePos = new Translation2d(-0.30162, 0.22542);
  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftModulePos, frontRightModulePos, backRightModulePos, backLeftModulePos);

  // Initializes each swerve module.
  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 1, false, 0.65, "canivore"); 
  private final SwerveModule frontRightModule = new SwerveModule(3, 4, 2, true, 0.73, "canivore");
  private final SwerveModule backRightModule = new SwerveModule(5, 6, 3, true, 0.77, "canivore");
  private final SwerveModule backLeftModule = new SwerveModule(7, 8, 4, false, 0.47, "canivore");
  private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backRightModule, backLeftModule};

  private final Pigeon2 pigeon = new Pigeon2(0, "canivore"); // Pigeon 2.0 CAN Gyroscope

  // Limelight (LL) Variables
  private final int maxCalibrationFrames = 50; // The number of LL frames that will be averaged to determine the position of the robot when it is disabled() or being calibrated.
  private final int minCalibrationFrames = 3; // The minimum amount of LL frames that must be processed to accept a calibration.
  private double[][] calibrationArray = new double[3][maxCalibrationFrames]; // An array that stores the LL botpose for the most recent frames, up to the number of frames specified by maxCalibrationFrames
  private int calibrationIndex = 0; // The index of the most recent entry into the calibrationPosition array. The index begins at 0 and goes up to calibrationFrames-1, after which it returns to 0 and repeats.
  private int calibrationFrames = 0; // The current number of frames stored in the calibrationPosition array. 
  private long lastFrame = 0; // The Limelight frame number of the last frame stored in the calibrationPosition array. Used to detect whether a new frame was recieved.

  // Path Following and Targeting Variables
  private ArrayList<PathPlannerTrajectory> paths = new ArrayList<PathPlannerTrajectory>(); // Stores the trajectories generated by Path Planner.
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getSMPs(), new Pose2d(), VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(0.5)), VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0))); // Uses the limelight, motor encoders, and gyroscope to track the position of the robot on the field.
  private final Timer pathTimer = new Timer(); // Keeps track of how long the robot has been following a path. Used to sample Path Planner trajectories.
  private final ProfiledPIDController xController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxVelAuto, maxAccAuto)); // Controls the x-position of the robot.
  private final ProfiledPIDController yController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxVelAuto, maxAccAuto)); // Controls the y-position of the robot.
  private final ProfiledPIDController angleController = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxAngularVelAuto, maxAngularAccAuto)); // Controls the angle of the robot.
  private boolean atDriveGoal = false; // Whether the robot is at the target within the tolerance specified by posTol and angTol when controlled by aimDrive() or moveToTarget()
  private double posTol = 0.07; // The allowable error in the x and y position of the robot in meters.
  private double angTol = 2.5; // The allowable error in the angle of the robot in degrees.
  
  // These variables are updated each period so they can be passed along to the user or the dashboard.
  private double xVel = 0.0; // Unit: meters per second
  private double yVel = 0.0; // Unit: meters per second
  private double angVel = 0.0; // Unit: degrees per second
  private double pathXPos = 0.0; // Unit: meters
  private double pathYPos = 0.0; // Unit: meters
  private double pathAngPos = 0.0; // Unit degrees

  public Drivetrain() {
    xController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    yController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    angleController.setIntegratorRange(-maxAngularVelAuto*0.8, maxAngularVelAuto*0.8);
    resetGyro(); // Sets the gyro angle to 0 based on the current heading of the robot.
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel. 
  // fieldRelative determines field-oriented control vs. robot-oriented control. field-relative control is automatically disabled in the case of a gyro failure.
  // Center of Rotation variables define where the robot will rotate from. 0,0 corresponds to rotations about the center of the robot. +x is towards the front. +y is to the left side.
  public void drive(double _xVel, double _yVel, double _angVel, boolean fieldRelative, double centerOfRotationX, double centerOfRotationY) {
    xVel = _xVel;
    yVel = _yVel;
    angVel = _angVel*180.0/Math.PI;
    SwerveModuleState[] moduleStates = fieldRelative
      ? kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(_xVel, _yVel, _angVel, Rotation2d.fromDegrees(getFusedAng())), new Translation2d(centerOfRotationX, centerOfRotationY))
      : kinematics.toSwerveModuleStates(new ChassisSpeeds(_xVel, _yVel, _angVel), new Translation2d(centerOfRotationX, centerOfRotationY));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVelTeleop); // Makes sure the calculated velocities are attainable. If they are not, all modules velocities are scaled back.
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modules[moduleIndex].setSMS(moduleStates[moduleIndex]); // Sets the module angles and velocities.
    }
  }

  // Should be called immediately prior to aimDrive() or driveTo(). Resets the PID controllers. Target angle specifies the first angle that will be demanded.
  public void resetDriveController(double targetAngle) {
    xController.reset(getXPos(), 0.0);
    yController.reset(getYPos(), 0.0);
    angleController.reset(getAngleDistance(getFusedAng(), targetAngle)*Math.PI/180.0, 0.0);
    xController.setPID(3.0, 0.0, 0.0);
    yController.setPID(3.0, 0.0, 0.0);
    angleController.setPID(4.0, 0.0, 0.0);
    atDriveGoal = false;
  }

  // Should be called periodically to rotate the robot to the demanded angle in degrees while translating the robot at the specified speed in meter per second.
  public void aimDrive(double _xVel, double _yVel, double targetAngle, boolean fieldRelative) {
    double angleDistance = getAngleDistance(getFusedAng(), targetAngle);
    atDriveGoal = Math.abs(angleDistance) < angTol;
    double _angVel = angleController.calculate(angleDistance*Math.PI/180.0, 0.0);
    if (atDriveGoal) {
      _angVel = 0.0;
    }
    if (Math.abs(_angVel) > Drivetrain.maxAngularVelAuto) {
      _angVel = _angVel > 0.0 ? Drivetrain.maxAngularVelAuto : -Drivetrain.maxAngularVelAuto;
    }
    drive(_xVel, _yVel, _angVel, fieldRelative, 0.0, 0.0);
  }

  // Should be called periodically to move the robot to a specified position and angle. Units are meters and degrees.
  public void driveTo(double targetX, double targetY, double targetAngle) {
    double xVelSetpoint = xController.calculate(getXPos(), targetX);
    double yVelSetpoint = yController.calculate(getYPos(), targetY);
    boolean atXTarget = Math.abs(getXPos() - targetX) < posTol;
    boolean atYTarget = Math.abs(getYPos() - targetY) < posTol;
    double angleDistance = getAngleDistance(getFusedAng(), targetAngle);
    double angVelSetpoint = angleController.calculate(angleDistance*Math.PI/180.0, 0.0);
    boolean atAngTarget = Math.abs(angleDistance) < angTol;

    // Checks to see if all 3 targets have been achieved. Sets velocities to 0 to prevent twitchy robot motions at near 0 velocities.
    atDriveGoal = atXTarget && atYTarget && atAngTarget;
    if (atXTarget) {
      xVelSetpoint = 0.0;
    } 
    if (atYTarget) {
      yVelSetpoint = 0.0;
    } 
    if (atAngTarget) {
      angVelSetpoint = 0.0;
    }

    // Caps the velocities if the PID controllers return values above the specified maximums.
    if (Math.abs(xVelSetpoint) > maxVelAuto) {
      xVelSetpoint = xVelSetpoint > 0.0 ?  maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(yVelSetpoint) > maxVelAuto) {
      yVelSetpoint = yVelSetpoint > 0.0 ? maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(angVelSetpoint) > maxAngularVelAuto) {
      angVelSetpoint = angVelSetpoint > 0.0 ? maxAngularVelAuto : -maxAngularVelAuto;
    }

    drive(xVelSetpoint, yVelSetpoint, angVelSetpoint, true, 0.0, 0.0);
  }

  // Whether the robot has reached the angle specified in the last call to aimDrive() or driveTo(). Should be called after aimDrive() or driveTo() is called within a period.
  public boolean atDriveGoal() {
    return atDriveGoal;
  }
  
  // Sets the allowable tolerance for the driveTo(), aimDrive(), and followPath() functions. Units are degrees and meters.
  public void setTolerance(double _posTargetTol, double _angTargetTol) {
    posTol = _posTargetTol;
    angTol = _angTargetTol;
  }

  // Calculates the shortest distance between two points on a 360 degree circle. CW is + and CCW is -
  public double getAngleDistance(double currAngle, double targetAngle) {
    double directDistance = Math.abs(currAngle - targetAngle);
    double wraparoundDistance = 360.0 - directDistance;
    double minimumDistance = Math.min(directDistance, wraparoundDistance);
    boolean isCW = (currAngle > targetAngle && wraparoundDistance > directDistance) || (currAngle < targetAngle && wraparoundDistance < directDistance);
    if (!isCW) {
      minimumDistance = -minimumDistance;
    }
    return minimumDistance;
  }

  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive. Each path is stored and refered to by the provided index.
  // pathName: The name of the path in Path Planner
  // initialVel: Robot velocity at the begining of the path. Usually 0. Units: meters per second
  // initialAngVel: Robot angular velocity at the begining of the path. Usually 0. Units: degrees per second
  // initialAngle: Robot's angle at the begining of the path. Units: degrees 
  public void loadPath(String pathName, double initialXVel, double initialYVel, double initialAngleVel, double initialAngle) {
    PathPlannerTrajectory path = PathPlannerPath.fromPathFile(pathName).getTrajectory(new ChassisSpeeds(initialXVel, initialYVel, initialAngleVel*Math.PI/180.0), Rotation2d.fromDegrees(initialAngle));
    paths.add(path);
  }

  // Should be called once exactly 1 period prior to the start of calls to followPath() each time a new path is followed. pathIndex starts at 0 and incements by 1 for each path loaded into loadPath().
  public void resetPathController(int pathIndex) {
    double initialGoalAngle = paths.get(pathIndex).getInitialState().heading.getDegrees();
    xController.reset(getXPos(), 0.0);
    yController.reset(getYPos(), 0.0);
    angleController.reset(getAngleDistance(getFusedAng(), initialGoalAngle)*Math.PI/180.0, 0.0);
    xController.setPID(1.5, 0.0, 0.001);
    yController.setPID(1.5, 0.0, 0.001);
    angleController.setPID(2.0, 0.0, 0.001);
    pathTimer.restart();
  }
  
  // Tracks the path. Should be called each period. The path controller should be reset if followPath() is not called for a period or more.
  public void followPath(int pathIndex) {
    // Samples the trajectory at the current time.
    PathPlannerTrajectory.State currentGoal = paths.get(pathIndex).sample(pathTimer.get());
    pathXPos = currentGoal.positionMeters.getX();
    pathYPos = isRedAlliance() ? fieldWidth - currentGoal.positionMeters.getY() : currentGoal.positionMeters.getY();
    pathAngPos = currentGoal.targetHolonomicRotation.getDegrees();
    double pathXVel = currentGoal.velocityMps*currentGoal.heading.getCos();
    double pathYVel = currentGoal.velocityMps*currentGoal.heading.getSin();
    double xVelCorrection = xController.calculate(getXPos(), pathXPos);
    double yVelCorrection = yController.calculate(getYPos(), pathYPos);
    double angleDistance = getAngleDistance(getFusedAng(), pathAngPos);
    double angVelSetpoint = angleController.calculate(angleDistance*Math.PI/180.0, 0.0);
    double xVelSetpoint = pathXVel + xVelCorrection;
    double yVelSetpoint = pathYVel + yVelCorrection;

    // Checks to see if all 3 targets have been achieved. Sets velocities to 0 to prevent twitchy robot motions at near 0 velocities.
    atDriveGoal = atPathEndpoint(pathIndex);
    if (atDriveGoal) {
      xVelSetpoint = 0.0;
      yVelSetpoint = 0.0;
      angVelSetpoint = 0.0;
    }

    // Caps the velocities if the PID controllers return values above the specified maximums.
    if (Math.abs(xVelSetpoint) > maxVelAuto) {
      xVelSetpoint = xVelSetpoint > 0.0 ?  maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(yVelSetpoint) > maxVelAuto) {
      yVelSetpoint = yVelSetpoint > 0.0 ? maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(angVelSetpoint) > maxAngularVelAuto) {
      angVelSetpoint = angVelSetpoint > 0.0 ? maxAngularVelAuto : -maxAngularVelAuto;
    }

    drive(xVelSetpoint, yVelSetpoint, angVelSetpoint, true, 0.0, 0.0);
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  // pathIndex: Which path to check, pathXTol and pathYTol: the allowable difference in position in meters, pathAngTol: the allowable difference in angle in degrees
  public boolean atPathEndpoint(int pathIndex) {
    PathPlannerTrajectory.State endState = paths.get(pathIndex).getEndState();
    double endStateYPos = isRedAlliance() ? fieldWidth - endState.positionMeters.getY() : endState.positionMeters.getY();
    return Math.abs(getFusedAng() - endState.targetHolonomicRotation.getDegrees()) < angTol 
      && Math.abs(getXPos() - endState.positionMeters.getX()) < posTol 
      && Math.abs(getYPos() - endStateYPos) < posTol;
  }

  // Updates the position of the robot on the field. Should be called each period to remain accurate. Tends to noticably drift for periods of time >15 sec.
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getGyroAng()), getSMPs());
  }

  // Incorporates vision information to determine the position of the robot on the field. Should be used only when vision information is deemed to be highly reliable (>1 april tag, close to april tag...)
  // xSD, ySD, and angSD tell the pose estimator how much to trust vision estimates. Larger values are less trustworthy. Units: xSD and ySD are in meters and angSD is in degrees. Default values can be found in pose estimate initialization.
  public void addVisionEstimate(double xSD, double ySD, double angSD) {
    long currentFrame = LimelightHelpers.getLimelightNTTableEntry("limelight", "hb").getInteger(0); // Gets the Limelight frame number from network tables.
    double thor = LimelightHelpers.getLimelightNTTableEntry("limelight", "thor").getDouble(0); // The horizontal width of the box bounding the April Tags in pixels.
    double tvert = LimelightHelpers.getLimelightNTTableEntry("limelight", "tvert").getDouble(0); // The vertical width of the box bounding the April Tags in pixels.
    double ta = LimelightHelpers.getTA(""); // The area of the box bounding the April Tags in percent of the screen.
    boolean tv = LimelightHelpers.getTV(""); // Whether a target is detected.
    boolean isSquare = Math.abs(tvert / thor - 1.0) < 0.2; // Checks to see if the box bounding the April Tags is square, indicating that 1 April Tag is likely detected.
    if (currentFrame != lastFrame && tv && !isSquare && ta > 1.5 && getXVel() < 0.1 && getYVel() < 0.1 && getAngVel() < 0.1) { // >1 April Tag is detected, the robot is relatively close to the April Tags, the robot is relatively stationary, and there is a new frame.
      double[] botpose = isBlueAlliance() ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed(""); // Transforms the vision position estimate to the appropriate coordinate system for the robot's alliance color
      odometry.addVisionMeasurement(new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(getFusedAng())), Timer.getFPGATimestamp()-botpose[6]/1000.0, VecBuilder.fill(xSD, ySD, Units.degreesToRadians(angSD)));      
      lastFrame = currentFrame;
    }
  }

  // Should be called during disabledInit(). Wipes previous calibration data from the calibrator.
  public void resetCalibration() {
    calibrationArray = new double[3][maxCalibrationFrames];
    calibrationIndex = 0;
    calibrationFrames = 0;
    lastFrame = 0;
  }

  // Should be called during disabled(). Calibrates the robot's starting position based on any April Tags in sight of the Limelight.
  public void addCalibrationEstimate() {
    long currentFrame = LimelightHelpers.getLimelightNTTableEntry("limelight", "hb").getInteger(0); // Gets the Limelight frame number from network tables.
    boolean tv = LimelightHelpers.getTV(""); // Whether a target is detected.
    if (tv && currentFrame != lastFrame) { // Checks to see whether there is at least 1 vision target and the LL has provided a new frame.
      lastFrame = currentFrame;
      double[] botpose = isBlueAlliance() ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed(""); // Transforms the vision position estimate to the appropriate coordinate system for the robot's alliance color
      calibrationArray[0][calibrationIndex] = botpose[0]; // Adds an x-position entry to the calibrationPosition array. 
      calibrationArray[1][calibrationIndex] = botpose[1]; // Adds a y-position entry to the calibrationPosition array. 
      calibrationArray[2][calibrationIndex] = botpose[5]; // Adds a angle-position entry to the calibrationPosition array. 
      calibrationIndex = (calibrationIndex + 1) % maxCalibrationFrames; // Handles the looping of the calibrationIndex variable. 
      if (calibrationFrames < maxCalibrationFrames) { // Increments calibrationPoints until the calibrationPosition array is full.
        calibrationFrames++; 
      }
    } 
  }

  // Should be called during autoInit() or teleopInit() to update the robot's starting position based on its April Tag calibration
  public void pushCalibration() {
    if (calibrationFrames > minCalibrationFrames) {
      double[] calibrationSum = new double[5];
      for (int index = 0; index < calibrationFrames; index++) {
        calibrationSum[0] = calibrationSum[0] + calibrationArray[0][index];
        calibrationSum[1] = calibrationSum[1] + calibrationArray[1][index];
        calibrationSum[2] = calibrationSum[2] + Math.sin(calibrationArray[2][index]*Math.PI/180.0);
        calibrationSum[3] = calibrationSum[3] + Math.cos(calibrationArray[2][index]*Math.PI/180.0);
        calibrationSum[4] = calibrationSum[4] + Math.abs(calibrationArray[2][index]);
      }
      double calibrationAng = calibrationSum[4]/calibrationFrames > 90.0 ? Math.atan(calibrationSum[2]/calibrationSum[3]) + Math.PI : Math.atan(calibrationSum[2]/calibrationSum[3]);
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAng()), getSMPs(), new Pose2d(calibrationSum[0]/calibrationFrames, calibrationSum[1]/calibrationFrames, Rotation2d.fromRadians(calibrationAng))); // Averages the values in the calibrationPosition Array and sets the robot position based on the averages.
    }
  }

  // Resets the gyro to 0 based on the current orientation of the robot.
  public void resetGyro() {
    pigeon.setYaw(0.0);
    odometry.resetPosition(new Rotation2d(), getSMPs(), new Pose2d(getXPos(), getYPos(), new Rotation2d()));
  }
  
  // Returns the angular position of the robot in degrees. The angular position is referenced to the starting angle of the robot. CCW is positive. Will return 0 in the case of a gyro failure.
  public double getGyroAng() {
    return pigeon.getYaw().getValueAsDouble();
  }

  // Returns the pitch of the robot in degrees. An elevated front is positive. An elevated rear is negative.
  public double getGyroPitch() {
    return pigeon.getPitch().getValueAsDouble();
  }

  // Returns true if the robot is on the red alliance.
  public boolean isRedAlliance() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  // Returns true if the robot is on the blue alliance.
  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().get().equals(Alliance.Blue);
  }

  // Returns the last commanded x-velocity of the robot in meters per second.
  public double getXVel() {
    return xVel;
  }

  // Returns the last commanded y-velocity of the robot in meters per second.
  public double getYVel() {
    return yVel;
  }

  // Returns the last commanded angular-velocity of the robot in degrees per second.
  public double getAngVel() {
    return angVel;
  }
  
  // Returns the odometry calculated x position of the robot in meters. This is based on vision and gyro data combined.
  public double getXPos() {
    return odometry.getEstimatedPosition().getX();
  }

  // Returns the odometry calculated y position of the robot in meters. This is based on vision and gyro data combined.
  public double getYPos() {
    return odometry.getEstimatedPosition().getY();
  }

  // Returns the odometry calcualted angle of the robot in degrees. This is based on vision and gyro data combined.
  public double getFusedAng() {
    return odometry.getEstimatedPosition().getRotation().getDegrees();
  }
  
  // The distance between the robot's current position and the current trajectory position. Units: meters
  public double getPathPosError() {
    return Math.sqrt(Math.pow(pathYPos - getYPos(), 2) + Math.pow(pathXPos - getXPos(), 2));
  }

  // The angular distance to the current trajectory point. Units: degrees
  public double getPathAngleError() {
    return getAngleDistance(getFusedAng(), pathAngPos);
  }
  
  // Publishes all values to the dashboard. Should be called each period. Uncomment individual lines to publish that information.
  public void updateDash() {
    SmartDashboard.putNumber("Front Left Swerve Module Position", frontLeftModule.getDriveMotorPos());
    SmartDashboard.putNumber("Front Right Swerve Module Position", frontRightModule.getDriveMotorPos());
    SmartDashboard.putNumber("Back Right Swerve Module Position", backRightModule.getDriveMotorPos());
    SmartDashboard.putNumber("Back Left Swerve Module Position", backLeftModule.getDriveMotorPos());
    SmartDashboard.putNumber("Front Left Swerve Module Velocity", frontLeftModule.getDriveMotorVel());
    SmartDashboard.putNumber("Front Right Swerve Module Velocity", frontRightModule.getDriveMotorVel());
    SmartDashboard.putNumber("Back Right Swerve Module Velocity", backRightModule.getDriveMotorVel());
    SmartDashboard.putNumber("Back Left Swerve Module Velocity", backLeftModule.getDriveMotorVel());
    SmartDashboard.putNumber("Front Left Swerve Module Turn Motor Angle", frontLeftModule.getTurnMotorAngle());
    SmartDashboard.putNumber("Front Right Swerve Module Turn Motor Angle", frontRightModule.getTurnMotorAngle());
    SmartDashboard.putNumber("Back Right Swerve Module Turn Motor Angle", backRightModule.getTurnMotorAngle());
    SmartDashboard.putNumber("Back Left Swerve Module Turn Motor Angle", backLeftModule.getTurnMotorAngle());
    SmartDashboard.putNumber("Front Left Swerve Module Wheel Encoder Angle", frontLeftModule.getWheelEncoderAngle());
    SmartDashboard.putNumber("Front Right Swerve Module Wheel Encoder Angle", frontRightModule.getWheelEncoderAngle());
    SmartDashboard.putNumber("Back Right Swerve Module Wheel Encoder Angle", backRightModule.getWheelEncoderAngle());
    SmartDashboard.putNumber("Back Left Swerve Module Wheel Encoder Angle", backLeftModule.getWheelEncoderAngle());
    SmartDashboard.putBoolean("Front Left Swerve Module Turn Motor Failure", frontLeftModule.getTurnMotorFailure());
    SmartDashboard.putBoolean("Front Right Swerve Module Turn Motor Failure", frontRightModule.getTurnMotorFailure());
    SmartDashboard.putBoolean("Back Right Swerve Module Turn Motor Failure", backRightModule.getTurnMotorFailure());
    SmartDashboard.putBoolean("Back Left Swerve Module Turn Motor Failure", backLeftModule.getTurnMotorFailure());
    SmartDashboard.putBoolean("Front Left Swerve Module Drive Motor Failure", frontLeftModule.getDriveMotorFailure());
    SmartDashboard.putBoolean("Front Right Swerve Module Drive Motor Failure", frontRightModule.getDriveMotorFailure());
    SmartDashboard.putBoolean("Back Right Swerve Module Drive Motor Failure", backRightModule.getDriveMotorFailure());
    SmartDashboard.putBoolean("Back Left Swerve Module Drive Motor Failure", backLeftModule.getDriveMotorFailure());
    SmartDashboard.putBoolean("Front Left Swerve Module Encoder Failure", frontLeftModule.getEncoderFailure());
    SmartDashboard.putBoolean("Front Right Swerve Module Encoder Failure", frontRightModule.getEncoderFailure());
    SmartDashboard.putBoolean("Back Right Swerve Module Encoder Failure", backRightModule.getEncoderFailure());
    SmartDashboard.putBoolean("Back Left Swerve Module Encoder Failure", backLeftModule.getEncoderFailure());
    SmartDashboard.putNumber("Robot X Position", getXPos());
    SmartDashboard.putNumber("Robot Y Position", getYPos());
    SmartDashboard.putNumber("Robot Angular Position (Fused)", getFusedAng());
    SmartDashboard.putNumber("Robot Angular Position (Gyro)", getGyroAng());
    SmartDashboard.putNumber("Robot Pitch", getGyroPitch());
    SmartDashboard.putNumber("Robot Demanded X Velocity", getXVel());
    SmartDashboard.putNumber("Robot Demanded Y Velocity", getYVel());
    SmartDashboard.putNumber("Robot Demanded Angular Velocity", getAngVel());
    SmartDashboard.putNumber("Path X Position", pathXPos);
    SmartDashboard.putNumber("Path Y Position", pathYPos);
    SmartDashboard.putNumber("Path Angular Position", pathAngPos);
    SmartDashboard.putNumber("Path Position Error", getPathPosError());
    SmartDashboard.putNumber("Path Angle Error", getPathAngleError());
    SmartDashboard.putBoolean("Path At Endpoint", atPathEndpoint(0));
    SmartDashboard.putBoolean("isRedAllaince", isRedAlliance());
    SmartDashboard.putBoolean("isBlueAllaince", isBlueAlliance());   
  }

  private SwerveModulePosition[] getSMPs() {
    SwerveModulePosition[] SMPs = new SwerveModulePosition[modules.length];
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      SMPs[moduleIndex] = modules[moduleIndex].getSMP();
    }
    return SMPs;
  }
}