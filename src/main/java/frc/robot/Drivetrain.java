package frc.robot;

import java.util.ArrayList;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
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
  public static final double maxVel = 4.0; // User defined maximum speed of the robot. Enforced during auto and teleop. Unit: meters per second
  public static final double maxAngularVel = 2*Math.PI; // User defined maximum rotational speed of the robot. Enforced only during teleop. Unit: raidans per second
  public static final double maxAcc = 12.0; // User defined maximum acceleration of the robot. Enforced only during teleop. Unit: meters per second^2
  public static final double maxAngularAcc = 6*Math.PI; // User defined maximum rotational acceleration of the robot. Enforced only during teleop. Unit: raidans per second^2

  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left. Units: meters.
  private static final Translation2d frontLeftModulePos = new Translation2d(0.225, 0.225);
  private static final Translation2d frontRightModulePos = new Translation2d(0.225, -0.225); 
  private static final Translation2d backRightModulePos = new Translation2d(-0.225, -0.225);
  private static final Translation2d backLeftModulePos = new Translation2d(-0.225, 0.225);
  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftModulePos, frontRightModulePos, backRightModulePos, backLeftModulePos);

  // Initializes each swerve module object.
  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0, false, 148.7); 
  private final SwerveModule frontRightModule = new SwerveModule(3, 4, 1, true, 98.2);
  private final SwerveModule backRightModule = new SwerveModule(5, 6, 2, true, -83.1);
  private final SwerveModule backLeftModule = new SwerveModule(7, 8, 3, false, 93.1);
  private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backRightModule, backLeftModule};
  private boolean moduleFailure = false; // Indicates whether there is at least 1 swerve module engine failure.
  private boolean moduleDisabled = false; // Indcates whether at least 1 module is disabled, either on startup or by the driver.

  // Gyroscope variables
  private final AHRS gyro = new AHRS();
  private boolean gyroFailure = false; // Indicates whether the gyro has lost connection at any point after a yaw-reset.
  private boolean gyroDisabled = false; // Indicates whether the gyro was disabled on startup, or by the driver by calling toggleGyro()

  // Vision variables
  private boolean visionDisconnected = false; // Indicates whether the Limelight is currently not connected by checking whether new frames are being uploaded to Network Tables
  private boolean visionDisabled = false; // Indicates whether the Limelight has been disabled by the driver by calling toggleVision()
  private long lastVisionFrame = 0; // The frame number of the last recieved frame from the Limelight.
  private double lastVisionFrameTime = 0.0;

  // Path following
  private ArrayList<PathPlannerPath> paths = new ArrayList<PathPlannerPath>();
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getSMPs(), new Pose2d(), VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(0.5)), VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0)));
  private final Timer timer = new Timer();
 
  // Autonomous swerve controller parameters. Hard code these values.
  private final double kP_drive = 3.0; // Units: The number of meter/second of correction for every 1meter of error.
  private final double kI_drive = 0.0; // Units: The number of meter/second of correction for every 1meter*1second of error.
  private final double kD_drive = 0.0; // Units: The number of meter/second of correction for every 1meter/1second of error.
  private final double I_driveMax = 1.0; // The maxiumum number of meter/second of correction based on the I term.
  private final double kP_turn = 4.0; // Units: The number of radian/second of correction for every 1rad of error.
  private final double kI_turn = 0.0; // Units: The number of radian/second of correction for every 1radian*1second of error.
  private final double kD_turn = 0.0; // Units: The number of radian/second of correction for every 1radian/1second of error.
  private final double I_turnMax = 1.0; // The maxiumum number of radian/second of correction based on the I term.

  // Autonomous swerve controller
  private final PIDController xController = new PIDController(kP_drive, kI_drive, kD_drive); // Controls corrections to the x-velocity during path following.
  private final PIDController yController = new PIDController(kP_drive, kI_drive, kD_drive); // Controls corrections to the y-velocity during path following.
  private final ProfiledPIDController turnController = 
    new ProfiledPIDController(kP_turn, kI_turn, kD_turn, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVel, Drivetrain.maxAngularVel)); // Controls corrections to the angular velocity during path following.
  private final HolonomicDriveController swerveController = new HolonomicDriveController(xController, yController, turnController); // An object that contains all 3 controllers in one.
  
  // These variables are updated each period so they can be passed to the dashboard. 
  private double xVel = 0.0; // Unit: meters per second
  private double yVel = 0.0; // Unit: meters per second
  private double angVel = 0.0; // Unit: degrees per second
  private double pathXPos = 0.0; // Unit: meters
  private double pathYPos = 0.0; // Unit: meters
  private double pathAngPos = 0.0; // Unit degrees

  public Drivetrain() {
    // Sets autonomous swerve controller parameters
    xController.setIntegratorRange(-I_driveMax, I_driveMax);
    yController.setIntegratorRange(-I_driveMax, I_driveMax);
    turnController.setIntegratorRange(-I_turnMax, I_turnMax);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    Timer.delay(2); // Delay to give the gyro time for start-up calibration.
    resetGyro(); // Sets the gyro angle to 0 based on the current heading of the robot.
    gyroDisabled = gyroFailure;
    updateModuleStatus(); // Checks whether any modules are offline or did not start up properly.
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel. 
  // fieldRelative determines field-oriented control vs. robot-oriented control. field-relative control is automatically disabled in the case of a gyro failure.
  // Center of Rotation variables define where the robot will rotate from. 0,0 corresponds to rotations about the center of the robot. +x is towards the front. +y is to the left side.
  public void drive(double _xVel, double _yVel, double _angVel, boolean fieldRelative, double centerOfRotationX, double centerOfRotationY) {
    xVel = _xVel;
    yVel = _yVel;
    angVel = _angVel*180.0/Math.PI;
    SwerveModuleState[] moduleStates = fieldRelative && !gyroFailure && !gyroDisabled
      ? kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(_xVel, _yVel, _angVel, Rotation2d.fromDegrees(visionDisabled ? getGyroAng() : getFusedAng())), new Translation2d(centerOfRotationX, centerOfRotationY))
      : kinematics.toSwerveModuleStates(new ChassisSpeeds(_xVel, _yVel, _angVel), new Translation2d(centerOfRotationX, centerOfRotationY));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVel); // Makes sure the calculated velocities are attainable. If they are not, all modules velocities are scaled back.
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modules[moduleIndex].setSMS(moduleStates[moduleIndex]); // Sets the module angles and velocities.
    }
  }

  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive. Each path is stored and refered to by the provided index.
  // pathName: The name of the path in Path Planner
  // maxPathVel: Maximum robot velocity while following this path. Units: meters per second
  // maxPathAcc: Maximum robot acceleration while following this path. Units: meters per second^2
  // pathReversal: Whether the path should be followed in forwards or reverse. 
  public void loadPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      path.flipPath();
    }
    paths.add(path);
  }

  // Should be called once exactly 1 period prior to the start of calls to followPath() each time a new path is followed. 
  public void resetPathController() {
    xController.reset();
    yController.reset();
    turnController.reset(visionDisabled ? getGyroAng() : getFusedAng(), 0.0);
    timer.restart();
  }
  
  // Tracks the path. Should be called each period. The path controller should be reset if followPath() is not called for a period or more.
  public void followPath(int pathIndex) {
    if (!gyroFailure && !gyroDisabled && !moduleFailure && !moduleDisabled) {
      PathPlannerTrajectory.State currentGoal = paths.get(pathIndex).getTrajectory(new ChassisSpeeds(), new Rotation2d()).sample(timer.get());
      ChassisSpeeds adjustedSpeeds = swerveController.calculate(new Pose2d(getXPos(), getYPos(), Rotation2d.fromDegrees(visionDisabled ? getGyroAng() : getFusedAng())), currentGoal.getTargetHolonomicPose(), currentGoal.velocityMps, currentGoal.targetHolonomicRotation); // Calculates the required robot velocities to accurately track the trajectory.
      drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, false, 0.0, 0.0); // Sets the robot to the correct velocities. 
      pathXPos = currentGoal.positionMeters.getX();
      pathYPos = currentGoal.positionMeters.getY();
      pathAngPos = currentGoal.targetHolonomicRotation.getDegrees();
    } else {
      drive(0.0, 0.0, 0.0, false, 0.0, 0.0);
    }
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  // pathIndex: Which path to check, pathXTol and pathYTol: the allowable difference in position in meters, pathAngTol: the allowable difference in angle in degrees
  public boolean atEndpoint(int pathIndex, double pathXTol, double pathYTol, double pathAngTol) {
    if (!gyroDisabled && !gyroFailure && !moduleFailure && !moduleDisabled) {
      PathPlannerTrajectory.State endState = paths.get(pathIndex).getTrajectory(new ChassisSpeeds(), new Rotation2d()).getEndState();
      return Math.abs((visionDisabled ? getGyroAng() : getFusedAng()) - endState.targetHolonomicRotation.getDegrees()) < pathAngTol 
        && Math.abs(getXPos() - endState.positionMeters.getX()) < pathXTol 
        && Math.abs(getYPos() - endState.positionMeters.getY()) < pathYTol;
    } else {
      return false;
    }
  }

  // Updates the position of the robot on the field. Should be called each period to remain accurate. Tends to noticably drift for periods of time >15 sec.
  public void updateOdometry() {
    if (!gyroDisabled && !gyroFailure) {
      odometry.update(Rotation2d.fromDegrees(getGyroAng()), getSMPs());
    }
  }

  // Incorporates vision information to determine the position of the robot on the field. Should be used only when vision information is deemed to be highly reliable (>1 april tag, close to april tag...)
  // xSD, ySD, and angSD tell the pose estimator how much to trust vision estimates. Larger values are less trustworthy. Units: xSD and ySD are in meters and angSD is in degrees. Default values can be found in pose estimate initialization.
  public void addVisionEstimate(double xSD, double ySD) {
    if (!getVisionDisconnected() && !visionDisabled && LimelightHelpers.getTV("")) { // Checks to see whether there is at least 1 vision target and the limelight is connected and enabled
      double[] botpose = isBlueAlliance() ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed(""); // Transforms the vision position estimate to the appropriate coordinate system for the robot's alliance color
      odometry.addVisionMeasurement(new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(getGyroAng())), Timer.getFPGATimestamp()-botpose[6]/1000.0, VecBuilder.fill(xSD, ySD, Units.degreesToRadians(0.5)));
    }
  }
  
  // Indicates whether the limelight is disconnected by determining whether a new frame has been recently uploaded to network tables.
  public boolean getVisionDisconnected() {
    long currentVisionFrame = LimelightHelpers.getLimelightNTTableEntry("limelight", "hb").getInteger(0); // Gets the Limelight frame number from network tables.
    double currentVisionFrameTime = Timer.getFPGATimestamp();
    if (currentVisionFrameTime - lastVisionFrameTime > 0.5) { // Checks to see whether at least 0.5s has elapsed since the last check.
      visionDisconnected = currentVisionFrame == lastVisionFrame; // Compares the current frame number to the previous frame number to see whether a new frame was recieved.
      lastVisionFrame = currentVisionFrame;
      lastVisionFrameTime = currentVisionFrameTime;
    }
    return visionDisconnected;
  }

  // Indicates whether the limelight has been disabled by the driver
  public boolean getVisionDisabled() {
    return visionDisabled;
  }

  // Allows the driver to toggle whether vision information is used to determine the position of the robot
  public void toggleVision() {
    visionDisabled = !visionDisabled;
  }
  
  // Returns the angular position of the robot in degrees. The angular position is referenced to the starting angle of the robot. CCW is positive. Will return 0 in the case of a gyro failure.
  public double getGyroAng() {
    if (gyro.isConnected() && !gyroFailure && !gyroDisabled) {
      return -gyro.getYaw();
    } else {
      gyroFailure = true;
      return 0;
    }
  }

  // Returns true if the robot is on the red alliance.
  public boolean isRedAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  // Returns true if the robot is on the blue alliance.
  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  
  // Returns the pitch of the robot in degrees. An elevated front is positive. An elevated rear is negative.
  public double getGyroPitch() {
    if (gyro.isConnected() && !gyroFailure && !gyroDisabled) {
      return gyro.getPitch();
    } else {
      gyroFailure = true;
      return 0;
    }
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
    double angleError = (visionDisabled ? getGyroAng() : getFusedAng()) - pathAngPos;
    if (angleError > 180.0) {
      angleError = angleError - 360.0;
    } else if (angleError < -180.0) {
      angleError = angleError + 360.0;
    }
    return angleError;
  }

  // Resets the robot's odometry to the start point of the path loaded into loadPath()
  public void resetOdometryToPathStart(int pathIndex) {
    if (!gyroDisabled && !gyroFailure) {
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAng()), getSMPs(), paths.get(pathIndex).getTrajectory(new ChassisSpeeds(), new Rotation2d()).getInitialState().getTargetHolonomicPose());
    }
  }

  // Resets the robot's odometry pose to the desired value. Units: meters and degrees. 
  public void resetOdometry(double xPos, double yPos, double angPos) {
    if (!gyroDisabled && !gyroFailure) {
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAng()), getSMPs(), new Pose2d(xPos, yPos, Rotation2d.fromDegrees(angPos)));
    }
  }
  
  // Resets the gyro to 0. The current angle of the robot is now defined as 0 degrees. Also clears gyroFailures if a connection is re-established
  public void resetGyro() {
    gyroFailure = !gyro.isConnected();
    if (!gyroFailure) {
      gyro.zeroYaw();
      odometry.resetPosition(new Rotation2d(), getSMPs(), new Pose2d(getXPos(), getYPos(), new Rotation2d()));
    }
  }

  // Allows the driver to toggle whether the gyro is enabled or disabled. Disabled gyro stops auto and field-oriented control. Useful in case of gyro issues.
  public void toggleGyro() {
    gyroDisabled = !gyroDisabled;
    resetGyro();
  }
  
  // True if the gyro was disconnected at any point after a yaw reset.
  public boolean getGyroFailure() {
    return gyroFailure;
  }

  // True if the gyro was disconnected on startup or if the gyro was disabled by the driver.
  public boolean getGyroDisabled() {
    return gyroDisabled;
  }

  // The following 4 functions allow the driver to toggle whether each of the swerve modules is on. Useful in the case of an engine failure in match. 
  public void toggleFL() {
    frontLeftModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleFR() {
    frontRightModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleBL() {
    backLeftModule.toggleModule();
    updateModuleStatus();
  }

  public void toggleBR() {
    backRightModule.toggleModule();
    updateModuleStatus();
  }

  // True if a single module failed to configure on startup or reboot.
  public boolean getModuleFailure() {
    return moduleFailure;
  }
  
  // True if a single module is disabled by the driver or the turn motor and drive motor failed to configure on startup or reboot.
  public boolean getModuleDisabled() {
    return moduleDisabled;
  }

  private SwerveModulePosition[] getSMPs() {
    SwerveModulePosition[] SMPs = new SwerveModulePosition[modules.length];
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      SMPs[moduleIndex] = modules[moduleIndex].getSMP();
    }
    return SMPs;
  }
  
  // Updates moduleError and moduleOffline to reflect the current status of the swerve modules
  private void updateModuleStatus() {
    moduleDisabled = false;
    moduleFailure = false;
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      moduleFailure = moduleFailure || modules[moduleIndex].getModuleFailure();
      moduleDisabled = moduleDisabled || modules[moduleIndex].getModuleDisabled();
    }
  }
  
  // Publishes all values to the dashboard. Should be called each period. Uncomment individual lines to publish that information.
  public void updateDash() {
    double[] modulePositions = new double[modules.length];
    double[] moduleVelocities = new double[modules.length];
    double[] moduleAngles = new double[modules.length];
    double[] moduleWheelEncoders = new double[modules.length];
    boolean[] modulesDisabled = new boolean[modules.length];
    boolean[] moduleDriveMotorFailures = new boolean[modules.length];
    boolean[] moduleTurnMotorFailures = new boolean[modules.length];
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getPos();
      moduleVelocities[moduleIndex] = modules[moduleIndex].getVel();
      moduleAngles[moduleIndex] = modules[moduleIndex].getAngle();
      moduleWheelEncoders[moduleIndex] = modules[moduleIndex].getWheelEncoder();
      modulesDisabled[moduleIndex] = modules[moduleIndex].getModuleDisabled();
      moduleDriveMotorFailures[moduleIndex] = modules[moduleIndex].getTurnMotorFailure();
      moduleTurnMotorFailures[moduleIndex] = modules[moduleIndex].getTurnMotorFailure();
    }
    SmartDashboard.putNumberArray("Module Positions", modulePositions);
    SmartDashboard.putNumberArray("Module Velocities", moduleVelocities);
    SmartDashboard.putNumberArray("Module Angles", moduleAngles);
    SmartDashboard.putNumberArray("Module Wheel Encoders", moduleWheelEncoders);
    SmartDashboard.putBooleanArray("Modules Disabled", modulesDisabled);
    SmartDashboard.putBooleanArray("Module Turn Motor Failures", moduleTurnMotorFailures);
    SmartDashboard.putBooleanArray("Module Drive Motor Failures", moduleDriveMotorFailures);
    SmartDashboard.putNumberArray("Robot Position", new double[] {getXPos(), getYPos(), getGyroAng()});
    SmartDashboard.putNumber("Gyro Angle", getGyroAng());
    SmartDashboard.putNumberArray("Demanded Velocity", new double[] {xVel, yVel, angVel});
    SmartDashboard.putNumberArray("Path Position", new double[] {pathXPos, pathYPos, pathAngPos});
    SmartDashboard.putBoolean("visionDisabled", visionDisabled);
    SmartDashboard.putBoolean("visionDisconnected", getVisionDisconnected());
    SmartDashboard.putBoolean("gyroFailure", gyroFailure);
    SmartDashboard.putBoolean("gyroDisabled", gyroDisabled);
    SmartDashboard.putBoolean("moduleFailure", moduleFailure);
    SmartDashboard.putBoolean("moduleDisabled", moduleDisabled);
    SmartDashboard.putNumber("Path Position Error", getPathPosError());
    SmartDashboard.putNumber("Path Angle Error", getPathAngleError());
    SmartDashboard.putBoolean("Path atEndpoint", atEndpoint(0, 0.01, 0.01, 0.05));
  }
}