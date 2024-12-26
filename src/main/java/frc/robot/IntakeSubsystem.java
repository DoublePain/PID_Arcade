package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;



//Intake class, this is different than a method. Classes contain our variables and methods acting as a blue print, while a method
//gives those variables and methods actions. Hence why we create our variables in the CLASS and not each Method!
public class IntakeSubsystem {
//Initialization and declaration of our motors
//Declaration defines the variable type and name but no value or memory assigned to that varible
//Initialization is when a value is assigned. You can do both immediately, or you can initialize later.
  private TalonSRX intakeMotor = new TalonSRX(7);
  private VictorSPX intakemini1 = new VictorSPX(10);
  private TalonSRX intakemini2 = new TalonSRX(11);

//Method to turn our intake on 
  public void IntakeOn(double speed ) // this method will require a parameter of a double called speed
  {
    // sets all of our intake motors to whatever speed is, speed will be whatever we put in our parenthesis
    // when we call the method in the robot subsystem
      intakeMotor.set(ControlMode.PercentOutput,speed);
         
      intakemini1.set(ControlMode.PercentOutput,speed);
         
      intakemini2.set(ControlMode.PercentOutput,speed);
    
  }

  //Another method to turn off the intake, if we dont have this the intake will turn on and never turn off
  public void IntakeOff(){
   
      intakeMotor.set(ControlMode.PercentOutput,0);
         
      intakemini1.set(ControlMode.PercentOutput,0);
         
      intakemini2.set(ControlMode.PercentOutput,0);

    }
  }

