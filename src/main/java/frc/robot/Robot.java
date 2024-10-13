package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAccTeleop / Drivetrain.maxAngularVelTeleop);
  private double speedScaleFactor = 1.0; // Scales the speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "auto1";
  private static final String auto2 = "auto2";
  private String autoSelected;
  private int autoStage = 1;

  public void robotInit() {
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Example", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain. 
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    updateDash();

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Menu Button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
  }

  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
      break;

      case auto2:
        // AutoInit 2 code goes here.
      break;
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
          break;
        }
      break;

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
          break;
        }
      break;
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.addVisionEstimate(0.04, 0.04, 10); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    
    if (driver.getRawButtonPressed(4)) speedScaleFactor = 1.0; // Y Button sets the drivetrain in full speed mode.
    if (driver.getRawButtonPressed(2)) speedScaleFactor = 0.6; // B button sets the drivetrain in medium speed mode.
    if (driver.getRawButtonPressed(1)) speedScaleFactor = 0.15; // A button sets the drivetrain in low speed mode.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*speedScaleFactor)*Drivetrain.maxAngularVelTeleop;

    if (driver.getRawButton(3)) {
      swerve.xLock(); // X buttton forces the swerve modules into an x-lock pattern to resist movement.
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.
  }

  public void disabledInit() {    
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    //SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() {
    swerve.resetDriveController(0.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.pushCalibration();
    swerve.addCalibrationEstimate();
    swerve.pushCalibration();
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.addVisionEstimate(0.1, 0.1, 0.1);
    swerve.updateOdometry();
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getPathAngleError: " + swerve.getPathAngleError());
    System.out.println("swerve getPathPosError: " + swerve.getPathPosError());
    System.out.println("swerve getXPos: " + swerve.getXPos());
    System.out.println("swerve getXVel: " + swerve.getXVel());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVel: " + swerve.getYVel());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    swerve.updateDash();
    updateDash();
  }
}