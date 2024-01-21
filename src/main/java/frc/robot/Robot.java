package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAcc/Drivetrain.maxVel);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAcc/Drivetrain.maxAngularVel);
  
  private final double minSpeedScaleFactor = 0.05; // The maximum speed of the robot when the throttle is at its minimum position, as a percentage of maxVel and maxAngularVel

  // Auto Chooser Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Auto 1";
  private static final String auto2 = "Auto 2";
  private static final String auto3 = "Auto 3"; 
  private static final String auto4 = "Auto 4"; 
  private String autoSelected;

  Thrower thrower = new Thrower();
  CANSparkFlex vortex = new CANSparkFlex(11, MotorType.kBrushless);

  public void robotInit() {
    configVortex(vortex);

    // Allows the user to choose which auto to do
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Test"); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.
    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetPathController();
    swerve.followPath(0);
    swerve.atEndpoint(0, 0.01, 0.01, 0.5);
    swerve.drive(0.1, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
  }

  public void robotPeriodic() {
    updateVision();
    swerve.updateDash();
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
    if (stick.getRawButtonPressed(2)) {
      swerve.toggleVision();
    }
  }
  
  public void autonomousInit() {
    swerve.pushCalibrationEstimate();
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        initAim();
        break;
      case auto2:
        // AutoInit 2 code goes here.
        swerve.resetPathController(); // Must be called immediately prior to following a Path Planner path using followPath().
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
    switch (autoSelected) {
      case auto1:
        // Auto 1 code goes here. 
        aim(1.6, 4.4, 180);
        break;
      case auto2:
        // Auto 2 code goes here.
        if (!swerve.atEndpoint(0, 0.01, 0.01, 0.5)) { // Checks to see if the endpoint of the path has been reached within the specified tolerance.
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
        robotToAprilTag(1);
        break;
    }
  }

  public void teleopInit() {
    swerve.pushCalibrationEstimate();
    thrower.init(); // Should be called in autoInit() and teleopInit(). Gets the thrower ready.
  }

  public void teleopPeriodic() {
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor); // Creates a scale factor for the maximum speed of the robot based on the throttle position.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(),0.1))*Drivetrain.maxVel*speedScaleFactor;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(),0.1))*Drivetrain.maxAngularVel*speedScaleFactor;

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

    // vortex.set(stick.getThrottle());
  }

  public void disabledInit() {
    swerve.resetCalibrationEstimator();
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
  public void rotateToAprilTag() {
    double tx = LimelightHelpers.getTX("");
    boolean tv = LimelightHelpers.getTV("");
    double output = angController.calculate(tx);
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

  ProfiledPIDController xController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0,3.0));
  ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0,3.0));
  ProfiledPIDController angleController = new ProfiledPIDController(0.000000001, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0,3.0));
  public void initAim() {
    xController.reset(swerve.getXPos());
    yController.reset(swerve.getYPos());
    angController.reset(swerve.getGyroAng());
    angController.enableContinuousInput(-180.0,180.0);
  }

  public void aim(double targetX, double targetY, double targetAng) {
    double xVel = Math.abs(swerve.getXPos() - targetX) > 0.1 ? xController.calculate(swerve.getXPos(), targetX) : 0.0;
    double yVel = Math.abs(swerve.getYPos() - targetY) > 0.1 ? yController.calculate(swerve.getYPos(), targetY) : 0.0;
    double angVel = Math.abs(swerve.getFusedAng() - targetAng) > 3.0 ? angController.calculate(swerve.getFusedAng(), targetAng) : 0.0;
    swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0);
  }

  public void configVortex(CANSparkFlex motor) {
    motor.restoreFactoryDefaults(true);
    motor.setSmartCurrentLimit(20);
    motor.setIdleMode(IdleMode.kBrake);
    motor.getEncoder().setPosition(0);
    motor.getPIDController().setP(0, 0);
    motor.getPIDController().setI(0, 0);
    motor.getPIDController().setD(0, 0);
    motor.getPIDController().setIMaxAccum(0, 0);
    motor.getPIDController().setFF(0, 0);
    motor.getPIDController().setDFilter(0,0);
    motor.burnFlash();
  }
}