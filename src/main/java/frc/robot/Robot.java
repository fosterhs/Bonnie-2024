package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0); // Initializes the joystick.
  private final Drivetrain swerve = new Drivetrain(); // Initializes the drivetrain (swerve modules, gyro, and path follower)

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAccTeleop/Drivetrain.maxAngularVelTeleop);
  
  private final double minSpeedScaleFactor = 0.05; // The maximum speed of the robot when the throttle is at its minimum position, as a percentage of maxVel and maxAngularVel

  // Auto Chooser Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Auto 1";
  private static final String auto2 = "Auto 2";
  private static final String auto3 = "Auto 3"; 
  private static final String auto4 = "Auto 4"; 
  private String autoSelected;
  private int autoStage = 0;

  Thrower thrower = new Thrower();

  public void robotInit() {
    // Allows the user to choose which auto to do
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Test", 0.0, 0.0, 0.0, 180.0); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.
    
    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.atPathEndpoint(0);
    swerve.drive(0.1, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
  }

  public void robotPeriodic() {
    updateVision();
    swerve.updateDash();
    SmartDashboard.putBoolean("throwerFailure", thrower.motorFailure());
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    // Allows the driver to toggle whether each of the swerve modules is on. Useful in the case of an engine failure in match. 
    if (stick.getRawButtonPressed(5)) {
      swerve.toggleFL();
    }
    if (stick.getRawButtonPressed(6)) {
      swerve.toggleFR();
    }
    if (stick.getRawButtonPressed(3)) {
      swerve.toggleBL();
    }
    if (stick.getRawButtonPressed(4)) {
      swerve.toggleBR();
    }

    // Re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    if (stick.getRawButtonPressed(11)) {
      swerve.resetGyro();
    }

    // Toggles the gyro on/off. Useful in the case of a gyro failure. A disabled gyro leads to loss of auto and field-oriented control. 
    if (stick.getRawButtonPressed(12)) {
      swerve.toggleGyro();
    }

    // Toggles whether vision information is used to drive the robot.
    if (stick.getPOV() == 0) {
      swerve.toggleVision();
    }
  }
  
  public void autonomousInit() {
    swerve.pushCalibration();
    thrower.init();
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        swerve.resetTargetController(180.0);
        break;
      case auto2:
        // AutoInit 2 code goes here.
        swerve.resetPathController(0); // Must be called immediately prior to following a Path Planner path using followPath().
        break;
      case auto3: 
        // AutoInit 3 code goes here.
        break;
      case auto4:
        // AutoInit 4 code goes here.
        break;
    }
  }

  public void autonomousPeriodic() {
    thrower.periodic();
    switch (autoSelected) {
      case auto1:
        // Auto 1 code goes here. 
        if (autoStage == 1) {
          swerve.moveToTarget(1.35, 2.6, 180.0); // Code to execute during this stage.
          if (swerve.atTarget()) { // Condition to move to the next stage. The code in the if statement will execute once (like an autoStageInit()), then move on to the next stage.
            thrower.commandThrow(120.0);
            autoStage = 2;
          }
        } else {
          swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot after auto is completed.
        }
        break;
      case auto2:
        // Auto 2 code goes here.
        if (!swerve.atPathEndpoint(0)) { // Checks to see if the endpoint of the path has been reached within the specified tolerance.
          swerve.followPath(0); // Follows the path that was previously loaded from Path Planner using loadPath().
        } else {
          swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops driving.
        }
        break;
      case auto3: 
        // Auto 3 code goes here.
        if (isSquare()) {
          strafeToAprilTag();
        } else {
          swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0);   
        }
        break;
      case auto4: 
        // Auto 4 code goes here.
        rotateToAprilTag(5.0);
        break;
    }
  }

  public void teleopInit() {
    swerve.pushCalibration();
    thrower.init(); // Should be called in autoInit() and teleopInit(). Gets the thrower ready.
  }

  public void teleopPeriodic() {
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor); // Creates a scale factor for the maximum speed of the robot based on the throttle position.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(), 0.1)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(), 0.1)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(), 0.1)*speedScaleFactor)*Drivetrain.maxAngularVelTeleop;

    // Allows the driver to rotate the robot about each corner. Defaults to a center of rotation at the center of the robot.
    if (stick.getRawButton(7)) { // Front Left
      swerve.drive(xVel, yVel, angVel, true, 0.29, 0.29);
    } else if (stick.getRawButton(8)) { // Front Right
      swerve.drive(xVel, yVel, angVel, true, 0.29, -0.29);
    } else if (stick.getRawButton(9)) { // Back Left
      swerve.drive(xVel, yVel, angVel, true, -0.29, 0.29);
    } else if (stick.getRawButton(10)) { // Back Right
      swerve.drive(xVel, yVel, angVel, true, -0.29, -0.29);
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }

    thrower.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the thrower.
    if (stick.getRawButton(1)) {
      thrower.commandThrow(120.0); // Commands the thrower to throw a note with a flywheel velocity of 120 rotations per second.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary.
    if (stick.getRawButtonPressed(2)) {
      swerve.resetCalibration();
    }
    if (stick.getRawButton(2)) {
      swerve.addCalibrationEstimate();
    }
    if (stick.getRawButtonReleased(2)) {
      swerve.pushCalibration();
    }
  }

  public void disabledInit() {
    swerve.resetCalibration();
  }
  
  public void disabledPeriodic() {
    swerve.addCalibrationEstimate();
  }

  public void updateVision() {
    boolean isSquare = isSquare();
    SmartDashboard.putBoolean("isSquare", isSquare);
    double[] presentDistanceArray = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_targetspace").getDoubleArray(new double[6]);
    double presentDistance = -presentDistanceArray[2];
    SmartDashboard.putNumber("Distance to Tag", presentDistance);
    double ta = LimelightHelpers.getTA("");
    double tid = LimelightHelpers.getFiducialID("");
    if (!isSquare && ta > 1.5 && ((tid == 8 && swerve.isBlueAlliance()) || (tid == 4 && swerve.isRedAlliance()))) {
      swerve.addVisionEstimate(0.04, 0.04);
    }
  }

  ProfiledPIDController angController = new ProfiledPIDController(0.14, 0.0, 0.004, new TrapezoidProfile.Constraints(1/4*Math.PI, 1/2*Math.PI));
  public void rotateToAprilTag(double offset) {
    double tx = LimelightHelpers.getTX("");
    boolean tv = LimelightHelpers.getTV("");
    double output = angController.calculate(tx-offset);
    if (!tv){
      swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
    } else {
      swerve.drive(0.0, 0.0, output, true, 0.0, 0.0);
    }
  }

  ProfiledPIDController strafeController = new ProfiledPIDController(0.08, 0.0, 0.0, new TrapezoidProfile.Constraints(0.8, 0.4));
  public void strafeToAprilTag() {
    double tx = LimelightHelpers.getTX("");
    boolean tv = LimelightHelpers.getTV("");
    double output = strafeController.calculate(-tx);

    if (tv) {
      swerve.drive(0.0, output, 0.0, true, 0.0, 0.0);
    } else {
      swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
    }
  }

  ProfiledPIDController distanceController = new ProfiledPIDController(0.08, 0.0, 0.0, new TrapezoidProfile.Constraints(0.8, 0.4));
  public void robotToAprilTag(double targetDistance) {
    double[] presentDistanceArray = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_targetspace").getDoubleArray(new double[6]);
    double presentDistance = -presentDistanceArray[2];
    boolean tv = LimelightHelpers.getTV("");
    double output = distanceController.calculate(presentDistance);
    if (tv) {
      swerve.drive(output, 0.0, 0.0, true, 0.0, 0.0);
    } else {
      swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
    }
  }
  
  public boolean isSquare() {
    double thor = LimelightHelpers.getLimelightNTTableEntry("limelight", "thor").getDouble(0);
    double tvert = LimelightHelpers.getLimelightNTTableEntry("limelight", "tvert").getDouble(0);
    if (Math.abs(tvert/thor-1.0) < 0.2) {
      return true;
    } else {
      return false;
    }
  }

  public void getAim() {
    double robotX = swerve.getXPos(); // Need to be translated to noteX once arm geometry is determined.
    double robotY = swerve.getYPos(); // Need to be translated to noteY once arm geometry is determined.
    double speakerZ = 2.045; // The center Z position of the slot in the speaker in meters.
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The center y position of the slot in the speaker in meters.
    double armL = 0.4; // The length of the arm between the pivot and the point where the note loses contact with the flywheel in meters.
    double armPivotZ = 0.1; // The height of the arm pivot above the floor in meters.
    double g = -9.806; // The gravitational acceleration in meters per second squared.
    double minAngle = 10.0; // The lowest angle the arm can expect to shoot at.
    double maxAngle = 80.0; // The highest angle the arm can expect to shoot at.
    int totalAngles = 70; // The total number of angles that should be checked.
    double noteV = 8.0; // The velocity of the note as it leaves the thrower in meters per second.
    double[] errorZ = new double[totalAngles]; // An array that stores the z-level that the note will impact the speaker at relative to the center of the speaker slot.
    
    for (int index = 0; index < totalAngles; index++) {
      double currentAngle = minAngle + index*(maxAngle-minAngle)/totalAngles;
      double robotR = Math.sqrt(Math.pow(robotX, 2) + Math.pow(robotY-speakerY, 2));
      double noteRVel = noteV*Math.cos(currentAngle*Math.PI/180.0);
      double armZ = armL*Math.sin(currentAngle*Math.PI/180.0) + armPivotZ;
      double noteTime = robotR/noteRVel;
      double noteFinalZ =  armZ + noteV*Math.sin(currentAngle*Math.PI/180.0)*noteTime + 0.5*g*Math.pow(noteTime, 2);
      errorZ[index] = noteFinalZ - speakerZ;
    }
    
    boolean shotAvailable = false;
    double armAngle = -1;
    for (int index = 0; index < totalAngles-1; index++) {
        if (errorZ[index] < 0 && errorZ[index+1] > 0) {
            armAngle = minAngle + (index+0.5)*(maxAngle-minAngle)/totalAngles;
            shotAvailable = true;
        }
    }
    SmartDashboard.putBoolean("shotAvailable", shotAvailable);
    SmartDashboard.putNumber("arm angle", armAngle);
    
    double robotAngle;
    if (robotY == speakerY) {
        robotAngle = 180.0;
    } else if (robotY < speakerY) {
        robotAngle = Math.atan(robotX/(speakerY-robotY))*180.0/Math.PI + 90.0;
    } else {
        robotAngle = Math.atan(robotX/(speakerY-robotY))*180.0/Math.PI - 90.0;           
    }
    SmartDashboard.putNumber("robot angle", robotAngle);
  }
  }