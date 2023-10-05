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
    private double RearLeftWheel = 1;
    private double FrontRightWheel = 1;
    private double RearRightWheel = 1;
    
  


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
        mFrontLeftTalon.set(m_driveControlMode, FrontLeftWheel);
        mFrontRightTalon.set(m_driveControlMode, FrontRightWheel);
        mRearLeftTalon.set(m_driveControlMode, RearLeftWheel);
        mRearRightTalon.set(m_driveControlMode, RearRightWheel);
      //   if(Math.abs(y)>Math.abs(x)){
      //     FrontLeftWheel = y;
      //     FrontRightWheel = y;
      //     RearLeftWheel = y*0.65;
      //     RearRightWheel = y*0.65;
      //     //Foward & Backward

      //   }
      //  else if (Math.abs(x)>Math.abs(y)){
      //     FrontLeftWheel = -x;
      //     FrontRightWheel = x;
      //     RearLeftWheel = x*0.65;
      //     RearRightWheel = -x*0.65;
      //     //Left
      //   }
        FrontLeftWheel = Math.sqrt((x*x) + (y*y));
        FrontRightWheel = Math.sqrt((x*x) + (y*y));
        RearLeftWheel = Math.sqrt((x*x) + (y*y))*0.65;
        RearRightWheel = Math.sqrt((x*x) + (y*y))*0.65;

      

    }
// public void SpinOff(DoubleSupplier ySpeed, DoubleSupplier xSpeed){
//        double y2 = ySpeed.getAsDouble();
//        double x2 = xSpeed.getAsDouble();
//         mFrontLeftTalon.set(m_driveControlMode, FrontLeftWheel);
//         mFrontRightTalon.set(m_driveControlMode, FrontRightWheel);
//         mRearLeftTalon.set(m_driveControlMode, RearLeftWheel);
//         mRearRightTalon.set(m_driveControlMode, RearRightWheel);

//         if(x2 > 0){
//           FrontLeftWheel = x2;
//           FrontRightWheel = -x2;
//           RearLeftWheel = x2*0.65;
//           RearRightWheel = -x2*0.65;
//         }
//         else if(x2 < 0){
//           FrontLeftWheel = -x2;
//           FrontRightWheel = x2;
//           RearLeftWheel = -x2*0.65;
//           RearRightWheel = x2*0.65;
//         }
// }

    
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
   * 
   * @param controlMode control mode to use setting talon output
   * @param velocityScale velocity for full scale in ticks/100ms
   */
  public void setControlMode(ControlMode controlMode, double velocityScale) {
    m_driveControlMode = controlMode;
  }

      public void drive(double y, double x) {
      }
}

