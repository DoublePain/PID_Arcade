// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*Hello welcome to PID and NavX practive with an arcade drive robot, please read all comments as they can help if you get confused
 * and they kinda just elaborate on what the code im writing does
*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/* 
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
*/
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

//Initialization and declaration of our motors
//Declaration defines the variable type and name but no value or memory assigned to that varible
//Initialization is when a value is assigned. You can do both immediately or you can initialize later.
  private TalonSRX leftMotor1 = new TalonSRX(2); // front left
  private TalonSRX leftMotor2 = new TalonSRX(3); // back left
  private TalonSRX rightMotor1 = new TalonSRX(1); // front right
  private TalonSRX rightMotor2 = new TalonSRX(4); // back right
 
  private static Joystick joy1 = new Joystick(0); // original instance of the joystick

  //Initializing and declaring our through bore encoders
  // Reverse so that positive values means that it's spinning to the right
  private Encoder encoder = new Encoder(4, 5, true);
  private Encoder encoderR = new Encoder (6,7,false);

  //Conversion from our encoder ticks, into feet
  private final double kDriveTick2Feet = 1.0 / 2350 * 6 * Math.PI / 12; // ticks * wheel diameter & Pi/12

  private final Timer timer = new Timer(); // Define a timer

  private static final double AUTO_SPEED = 0.15; // Wheel speed for the Autonomous
  private static final double AUTO_DURATION = 10.0; // Our Autonomous length in seconds
  private static final double TARGET_ANGLE = 0.0; // Desired heading in degrees
 // private static final double TargetPitch = 0.0;  // Desired pitch in degrees
  
  
//Declare and Initialize our limelight 
/*
  private static  NetworkTable table = NetworkTableInstance.getDefault().getTable("raider");

//Retrive data from our limelight and convert them into a variable we will store the data in
 /
 private static NetworkTableEntry tx = table.getEntry("tx");
  private static NetworkTableEntry ty = table.getEntry("ty");
  private static NetworkTableEntry tv = table.getEntry("tv");
*/
  //Retrieving our subsystems and storing their info at variables so that we can call them and retreive the methods inside of them
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(); 
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  @Override
  public void robotInit() {

  }
  

  @Override
  public void autonomousInit() {
    // Reset timer and start the timer after it's reset
timer.reset();
timer.start();

//Reset our NAVX
driveSubsystem.resetNavx();

  }

    // Method to get the shared joystick instance
    public static Joystick getJoystick() {
      return joy1;
  }

  
 //Anything in this method will happen periodically in the autonomous portion
  @Override
  public void autonomousPeriodic() {


// if our pre set timer is less that our auto duration variable, drive straight at the target angle of zero
//note that drive straight is a method I made in the DriveSubsystem, hence why I call it using driveSubsystem.

    if (timer.get() < AUTO_DURATION) {

      driveSubsystem.driveStraight(AUTO_SPEED, TARGET_ANGLE);

  } else {

      driveSubsystem.Stop();

  }

  
/* 
  if (timer.get() < AUTO_DURATION){

driveSubsystem.balance(AUTO_SPEED, TargetPitch);

  }
  else {

driveSubsystem.Stop();

  }
*/

   
  }

// Anything in this method will happen periodically as the robot is on
  @Override
  public void robotPeriodic() {

    //Send encoder values to the dashboard
    SmartDashboard.putNumber("Left encoder value", encoder.get() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right encoder value", encoderR.get() * kDriveTick2Feet);
    driveSubsystem.NavXValues();
  }

  @Override
  //Anything in here is called when driver control begins
  public void teleopInit() {
   lasterror = 0;
  //Reset encoder value when teleop is activated
    encoder.reset();
    encoderR.reset();
    // Reset NavX Value 
    driveSubsystem.resetNavx();
    
     lastTimestamp = Timer.getFPGATimestamp();
     errorsum = 0;
  }

//The proportional value of the PID controls the speeding up and slowing down of mechanisms, for this instance were using 
//it to control the speeding up and slowing down of the chassis, it sees the present errors but overshoots often.
  final double kP = 0.8; // .125

//The integral sees the past errors in the motor speed, and adjusts the motors to correct its velocity to slow down when reaching 
//the checkpoint
  final double kI = 0.01; // .3

  //The derivative variable is used to gather the future errors and adjust accordingly , slowing down BEFORE the robot reaches its setpoint
  //it works to stop the overshooting
  final double kD = 0.01; // .05

  final double ilimit = 1;

//Variables we will keep our values stored in, with an initial value of 0
  double lastTimestamp = 0;
  double lasterror = 0;
  double errorsum = 0; // Error sum is our I term in PID because integral gets the sum of our error while going to our setpoint
  double setpoint = 0; 



  
// Anything in this method will take action during the driver control
  @Override
  public void teleopPeriodic() {

      // get joystick command
      if (joy1.getRawButton(1)) {
        setpoint = 14; // setpoint = 6 when button 1 is pressed
    } else if (joy1.getRawButton(2)) {
        setpoint = 0; // setpoint = 0 when button 2 is pressed
    }


    // Intake controls
    if (joy1.getRawButton(3)) {
        intakeSubsystem.IntakeOn(-1);
    } else {
        intakeSubsystem.IntakeOff();
    }

    // get sensor positions for each wheel
    double sensorPosition = encoder.get() * kDriveTick2Feet;
    double sensorPositionR = encoderR.get() * kDriveTick2Feet;

    // calculations
    //get time distance from current timestamp to last timestamp
    //dt = delta time or basically the change in time
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
   // returns the current time from the FPGA (Field-Programmable Gate Array) in seconds.

//You have to hold the button for the angle to reman constant, I would like to try this in autonomous which I will try another time
    if (joy1.getRawButton(1)|| joy1.getRawButton(2)) {
        driveSubsystem.driveToSetpointWithNavx(setpoint, sensorPosition, TARGET_ANGLE, kP, kI, kD, ilimit, dt, errorsum, lasterror);
    } else {

     //the differences between the setpoint and the current positions for the left and right sides, respectively.
        double error = setpoint - sensorPosition;
        double errorR = setpoint - sensorPositionR;

      //the rates of change of the errors.
        double errorRate = (error - lasterror) / dt;
        double errorRateR = (errorR - lasterror) / dt;

        if (Math.abs(error) < ilimit) {
            errorsum += error * dt;
        }
      // Math.abs is Absolute Value
        if (Math.abs(errorR) < ilimit) {
            errorsum += errorR * dt;
        }

        //Calculating our output speeds using the formula output=kP×error+kI×errorsum+kD×errorRate.
        double outputSpeed = kP * error + kI * errorsum + kD * errorRate;
        double outputSpeedR = kP * errorR + kI * errorsum + kD * errorRateR;


        //The calculated speeds are applied to the motors. 
        // Note that the right motors have their outputs negated to account for the opposite orientation of the motors.
        leftMotor1.set(ControlMode.PercentOutput, outputSpeed);
        leftMotor2.set(ControlMode.PercentOutput, outputSpeed);
        rightMotor1.set(ControlMode.PercentOutput, -outputSpeedR);
        rightMotor2.set(ControlMode.PercentOutput, -outputSpeedR);

        //lastTimestamp is updated to the current time.
        lastTimestamp = Timer.getFPGATimestamp();

        //lasterror is updated to the current error for use in the next iteration.
        lasterror = error;

        //would preferable like to put all of this in a the drivesubsystem as a method and call it here, but im too lazy
        //But you should def do that if you use this code because it will be much cleaner!
    }
  }

  public double getLeftDistance() {
    return encoder.getDistance();
}

public double getRightDistance() {
    return encoderR.getDistance();
}

public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
