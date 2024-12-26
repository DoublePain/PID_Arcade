package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathRamsete;

public class pathplanner {
    // Motor controllers
    private final TalonSRX leftMaster = new TalonSRX(1);
    private final TalonSRX leftFollower = new TalonSRX(2);
    private final TalonSRX rightMaster = new TalonSRX(3);
    private final TalonSRX rightFollower = new TalonSRX(4);

    // NavX gyro
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    // Joystick
    private final Joystick joystick = new Joystick(0);

    // Differential Drive
   // private final DifferentialDrive tankDrive;

    // Differential drive kinematics (width between wheels)
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6); // Adjust based on your robot's wheelbase width

    // Controller and PID settings
    private final RamseteController ramseteController = new RamseteController(2.0, 0.7);
    private final PIDController leftPID = new PIDController(1.2, 0.01, 0.01);
    private final PIDController rightPID = new PIDController(1.2, 0.01, 0.01);
/* 
    public pathplanner() {
        // Make sure the follower motors follow the master motors
        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        // Create speed controller groups for left and right motors
        SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftFollower);
        SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightFollower);

        // Initialize the DifferentialDrive with speed controller groups
        tankDrive = new DifferentialDrive(leftMotors, rightMotors);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind joystick buttons to actions if needed
    }

    public Command getAutonomousCommand() {
        // Load your path from PathPlanner
        PathPlannerTrajectory path = PathPlanner.loadPath("YourPathName", 2.0, 1.5); // max velocity, max acceleration
        
        // Return the command to follow the path
        return followPathCommand(path);
    }

    // Command to follow the path
    public Command followPathCommand(PathPlannerTrajectory path) {
        // Create a new FollowPathRamsete command
        FollowPathRamsete ramseteCommand = new FollowPathRamsete(
            path,                       // The trajectory to follow
            this::getRobotPose,        // Pose supplier (robot's current position)
            ramseteController,         // Ramsete controller
            kinematics,                // Drive kinematics
            this::tankDriveVolts,      // Output the desired voltage to the motors
            leftPID,                   // Left motor PID controller
            rightPID,                  // Right motor PID controller
            this::getLeftEncoderPosition,  // Left encoder position
            this::getRightEncoderPosition, // Right encoder position
            this::getGyroHeading,      // Gyro heading
            this::getWheelSpeeds       // Current wheel speeds
        );

        // Run the path and stop the robot after finishing
        return new SequentialCommandGroup(
            ramseteCommand,
            new InstantCommand(() -> tankDrive.stopMotor())
        );
    }

    // Method to get the current robot pose (position and heading)
    public Pose2d getRobotPose() {
        return new Pose2d(0, 0, new Rotation2d(navX.getAngle()));  // Replace with actual pose tracking if needed
    }

    // Method to output motor voltages for the left and right side
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.set(ControlMode.PercentOutput, leftVolts / 12.0);
        rightMaster.set(ControlMode.PercentOutput, rightVolts / 12.0);
    }

    // Placeholder methods to get encoder values
    public double getLeftEncoderPosition() {
        return leftMaster.getSelectedSensorPosition();  // Replace with actual encoder value
    }

    public double getRightEncoderPosition() {
        return rightMaster.getSelectedSensorPosition();  // Replace with actual encoder value
    }

    // Placeholder method to get the current wheel speeds
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // Replace with actual speed calculations based on encoder rates
        return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(), rightMaster.getSelectedSensorVelocity());
    }

    // Get the robot's heading from the gyro
    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-navX.getAngle());  // NavX gives angle in degrees
    }*/
}