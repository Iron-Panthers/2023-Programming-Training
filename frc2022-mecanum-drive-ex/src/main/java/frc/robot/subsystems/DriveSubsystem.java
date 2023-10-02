package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;

public class DriveSubsystem extends SubsystemBase {

    private TalonSRX mFrontLeftTalon;
    private TalonSRX mRearLeftTalon;
    private TalonSRX mFrontRightTalon;
    private TalonSRX mRearRightTalon;
    

    private double m_frontLeftCoeff = 1;
    private double m_rearLeftCoeff = 1;
    private double m_frontRightCoeff = 1;
    private double m_rearRightCoeff = 1;
    private double FrontLeftWheel = 1;
    private double FrontRightWheel = 1;
    private double RearLeftWheel = 1;
    private double RearRightWheel = 1;

    // private int multiTH = 1; // wheels = t(left Front)   y(Right Front)
    // private int multiYG = 1; //          g(left back)    h(Right back)

    private ControlMode m_driveControlMode = ControlMode.PercentOutput;

    public DriveSubsystem( TalonSRX mFrontLeftTalon, TalonSRX mRearLeftTalon, TalonSRX mFrontRightTalon, TalonSRX mRearRightTalon) {
        this.mFrontLeftTalon = mFrontLeftTalon;
        this.mRearLeftTalon = mRearLeftTalon;
        this.mFrontRightTalon = mFrontRightTalon;
        this.mRearRightTalon = mRearRightTalon;
    }

   public void drive(DoubleSupplier ySpeed, DoubleSupplier xSpeed)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        // mRobotDrive.driveCartesian(ySpeed, xSpeed, zRot, 0.0);
        double y = ySpeed.getAsDouble();
        double x = xSpeed.getAsDouble();
   
// //IMININ
// if (x>y)
//         {
//           multiTH = -1;
//         }
//         else
//         {
//           multiTH = 1;
//         }
//         if (y>(x*-1))
//         {
//           multiYG = -1;
//         }
//         else
//         {
//           multiYG = 1;
//         }
//         double speed1 = Math.abs((Math.sqrt((x*x)+(y*y)))/Math.sqrt(2)); // finding distance of joystick to center
//         double speed2 = math.abs(0.5*(y + x)); // finding solution to split line equation x=y and joystick locaiton equation, y=-x+xSpeed+ySpeed
//         double speed = ((speed1*speed2));


//         mFrontLeftTalon.set(m_driveControlMode, speed*multiTH);
//         mFrontRightTalon.set(m_driveControlMode,speed*multiYG);
//         mRearLeftTalon.set(m_driveControlMode, speed*multiYG);
//         mRearRightTalon.set(m_driveControlMode,speed*multiTH);
//IMIMNINI
        
        // mFrontLeftTalon.set(m_driveControlMode, y + x);
        // mFrontRightTalon.set(m_driveControlMode, y + x);
        // mRearLeftTalon.set(m_driveControlMode, y + x);
        // mRearRightTalon.set(m_driveControlMode, y + x);

        mFrontLeftTalon.set(m_driveControlMode, FrontLeftWheel);
        mFrontRightTalon.set(m_driveControlMode, FrontRightWheel);
        mRearLeftTalon.set(m_driveControlMode, RearLeftWheel);
        mRearRightTalon.set(m_driveControlMode, RearRightWheel);
        
    
      if (Math.abs(y)>Math.abs(x)){ //forward or backward
        FrontLeftWheel = y;
        FrontRightWheel = y;
        RearLeftWheel = y*0.6;
        RearRightWheel = y*0.6;
      }else if (Math.abs(x)>Math.abs(y)){ //right
        FrontLeftWheel = x;
        FrontRightWheel = -x;
        RearLeftWheel = -x*0.6;
        RearRightWheel = x*0.6;
      }
    }

    
      public void setMotorCoeff(
        double frontLeftCoeff,
        double rearLeftCoeff,
        double frontRightCoeff,
        double rearRightCoeff) {
        
      m_frontLeftCoeff = frontLeftCoeff;
      m_rearLeftCoeff = rearLeftCoeff;
      m_frontRightCoeff = frontRightCoeff;
      m_rearRightCoeff = rearRightCoeff;
    }

      /**
   * Set control mode and velocity scale (opt)
    
   * @param controlMode control mode to use setting talon output
   * @param velocityScale velocity for full scale in ticks/100ms
   */
  public void setControlMode(ControlMode controlMode, double velocityScale) {
    m_driveControlMode = controlMode;
    
  }
}

