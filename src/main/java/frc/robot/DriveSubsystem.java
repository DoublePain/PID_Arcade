package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
//Drive class, this is different than a method. Classes contain our variables and methods acting as a blue print, while a method
//gives those variables and methods actions. Hence why we create our variables in the CLASS and not each Method!

public class DriveSubsystem {
    
// Initialization and declaration of our drive motors
//Declaration defines the variable type and name but no value or memory assigned to that varible
//Initialization is when a value is assigned. You can do both immediately or you can initialize later.
    private final TalonSRX LeftFront = new TalonSRX(2);
    private final TalonSRX RightFront = new TalonSRX(1);
    private final TalonSRX LeftBack = new TalonSRX(3);
    private final TalonSRX RightBack = new TalonSRX(4);
    private final Timer timer = new Timer();
    private final Encoder leftEncoder = new Encoder(3,4);  // Port 2
    private final Encoder rightEncoder = new Encoder(1,2); // Port 1

    // NavX initialization and declaration
    private final  AHRS navx = new AHRS(SPI.Port.kMXP); 
    private ShuffleboardTab tab = Shuffleboard.getTab("Robot State");

    // PID controller initialization and Declaration
    private final PIDController pidController = new PIDController(0.05, 0, 0.01); //Pid controller to use for getting to setpoint
    private final PIDController positionPID = new PIDController(0.1, 0, 0); // PID for position (forward/backward)
    private final PIDController rotationPID = new PIDController(0.05, 0, 0.01); // PID for rotation (turning)
    private DifferentialDriveOdometry odometry;
    

//Method to get robot to drive to setpoint in feet, while maintaining a certain direction using gyro (NavX)
    public void driveToSetpointWithNavx(double setpoint, double currentEncoderPosition, double targetAngle, double kP, double kI, double kD, double ilimit, double dt, double errorsum, double lastError) {
        double currentAngle = navx.getYaw();
        double rotationCorrection = pidController.calculate(currentAngle, targetAngle);

        double error = setpoint - currentEncoderPosition;
        double errorRate = (error - lastError) / dt;

        if (Math.abs(error) < ilimit) {
            errorsum += error * dt;
        }

        double outputSpeed = kP * error + kI * errorsum + kD * errorRate;
        // Increase the weight of rotation correction if it turns to much and goes crazy
        Drive(outputSpeed, rotationCorrection / 2.5);
    }
void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
}


public void updateOdometry() {
    // Get the current pose from odometry
    Pose2d currentPose = odometry.getPoseMeters();

    // Extract x, y, and rotation from the pose
    double x = currentPose.getX();
    double y = currentPose.getY();
    Rotation2d rotation = currentPose.getRotation();
    double angle = rotation.getDegrees(); // Get angle in degrees

    // Send the odometry data to Shuffleboard
    tab.add("Odometry X", x);
    tab.add("Odometry Y", y);
    tab.add("Odometry Angle", angle);
}

public DriveSubsystem() {
    // Convert NavX angle from degrees to radians and create a Rotation2d
    Rotation2d initialAngle = Rotation2d.fromDegrees(navx.getAngle());
    
    // Assuming you have methods for getting the distance from the encoders
    double leftDistance = leftEncoder.getDistance(); // in meters (or whatever unit you're using)
    double rightDistance = rightEncoder.getDistance(); // in meters (or whatever unit you're using)

    // Initialize the DifferentialDriveOdometry
    odometry = new DifferentialDriveOdometry(initialAngle, leftDistance, rightDistance);
}

    public void updateShuffleboard() {
        // Send encoder positions
        tab.add("Left Front Encoder", LeftFront.getSelectedSensorPosition());
        tab.add("Right Front Encoder", RightFront.getSelectedSensorPosition());
        tab.add("Left Back Encoder", LeftBack.getSelectedSensorPosition());
        tab.add("Right Back Encoder", RightBack.getSelectedSensorPosition());

        // Send NavX data
        tab.add("NavX Angle", navx.getAngle());
        tab.add("NavX Pitch", navx.getPitch());
        tab.add("NavX Yaw", navx.getYaw());

        // You can add other states such as battery voltage, speed, etc.
        tab.add("Battery Voltage", RobotController.getBatteryVoltage());
    }

    //Normal drive method for teleop
    void Drive(double speed, double rotation){

        double leftSpeed = speed + rotation;
        double rightSpeed = speed - rotation;

        LeftFront.set(ControlMode.PercentOutput, leftSpeed);
        LeftBack.set(ControlMode.PercentOutput, leftSpeed);
        RightBack.set(ControlMode.PercentOutput, -rightSpeed);
        RightFront.set(ControlMode.PercentOutput, -rightSpeed);
    }
//Stop function to use for auto
    void Stop(){

        LeftFront.set(ControlMode.PercentOutput, 0);
        LeftBack.set(ControlMode.PercentOutput, 0);
        RightBack.set(ControlMode.PercentOutput, 0);
        RightFront.set(ControlMode.PercentOutput, 0);

    }
    public void driveForward(double targetDistance, double speed) {
        double initialPosition = LeftFront.getSelectedSensorPosition(); // Get encoder position at the start
        double targetPosition = initialPosition + targetDistance * 4096 / (Math.PI * 6); // Convert feet to encoder ticks (adjust 6 to wheel diameter)

        // PID for position control
        while (Math.abs(LeftFront.getSelectedSensorPosition() - targetPosition) > 50) {
            double error = targetPosition - LeftFront.getSelectedSensorPosition();
            double outputSpeed = positionPID.calculate(error, 0);
            Drive(speed + outputSpeed, 0); // Drive straight
        }
        Stop(); // Stop once the target distance is reached
    }

    public void driveBackward(double targetDistance, double speed) {
        driveForward(-targetDistance, speed); // Simply use the same method, but with negative distance
    }

    public void turnToAngle(double targetAngle, double speed) {
        double currentAngle = navx.getYaw();
        double angleError = targetAngle - currentAngle;
        
        // PID for rotation control
        while (Math.abs(angleError) > 1.0) { // Tolerance of 1 degree
            currentAngle = navx.getYaw();
            angleError = targetAngle - currentAngle;
            double rotationOutput = rotationPID.calculate(angleError, 0);
            Drive(0, rotationOutput); // Turn in place
        }
        Stop(); // Stop once the target angle is reached
    }

 public void autonomousinit(){
    timer.reset();
    timer.start();
    resetNavx();

 }

 public void autonomousPeriodic() {
    // Example of autonomous sequence: drive forward, then turn, then drive backward

    double currentTime = timer.get();
    
    if (currentTime < 2.0) {
        // Drive forward for 2 seconds (use encoders)
        driveForward(2.0, 0.5); // 2 feet, speed 50%
    } else if (currentTime < 4.0) {
        // Turn left for 2 seconds (use NavX for heading correction)
        turnToAngle(90, 0.5); // Turn 90 degrees left at 50% speed
    } else if (currentTime < 6.0) {
        // Drive backward for 2 seconds (use encoders)
        driveBackward(2.0, -0.5); // 2 feet, speed -50%
    } else {
        // Stop the robot after 6 seconds
        Stop(); 
    }
}
    //Using navX to keep robot following trajectory even when pushed
    public void driveStraight(double speed, double targetAngle) {
//Setting a variable to keep our NavX Yaw values
        double currentAngle = navx.getYaw(); 
// Our rotation correction variable is using our pid to calculate the difference in our current angle and what our targetangle is 
        double rotationCorrection = pidController.calculate(currentAngle, targetAngle);
// Then it drives our robot at the initial input speed we put into the method, as well as the rotation correction
//When I first ran this, it overcorrected way to much. To fix that you could either tune your PID values more or just lower the
// Correction by dividing it like I did. 
        Drive(speed, rotationCorrection/2.5);
    }

    //Function to reset NavX
    public void resetNavx() {
        navx.reset();
    }
    //Function to retrieve our NavX values
    public void NavXValues(){
        SmartDashboard.putNumber("NavXAngle", navx.getAngle());
        SmartDashboard.putNumber("NavXPitch", navx.getPitch());
    }
    

    //Attempt at an autobalance function, will try again another time
/* 
    public void balance(double speed, double TargetPitch){
       double currentPitch = navx.getPitch();
       double PitchCorrection = pidController.calculate(currentPitch, TargetPitch);
      Drive(speed, PitchCorrection/3);
    }
*/
}
