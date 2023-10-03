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


    private ControlMode m_driveControlMode = ControlMode.PercentOutput;

    public DriveSubsystem( TalonSRX mFrontLeftTalon, TalonSRX mRearLeftTalon, TalonSRX mFrontRightTalon, TalonSRX mRearRightTalon) {
        this.mFrontLeftTalon = mFrontLeftTalon;
        this.mRearLeftTalon = mRearLeftTalon;
        this.mFrontRightTalon = mFrontRightTalon;
        this.mRearRightTalon = mRearRightTalon;
    }


   public void drive(DoubleSupplier ySpeedSupplier, DoubleSupplier xSpeedSupplier)
   {

    double ySpeed = ySpeedSupplier.getAsDouble();
    double xSpeed = xSpeedSupplier.getAsDouble();
        if (ySpeed >=0.5 || ySpeed <= -0.5){
          mFrontLeftTalon.set(m_driveControlMode, ySpeed);
          mFrontRightTalon.set(m_driveControlMode, ySpeed);
          mRearLeftTalon.set(m_driveControlMode, ySpeed);
          mRearRightTalon.set(m_driveControlMode, ySpeed);
        }
       
         if (xSpeed >= 0.5){
          mFrontLeftTalon.set(m_driveControlMode, xSpeed);
          mFrontRightTalon.set(m_driveControlMode, -xSpeed);
          mRearLeftTalon.set(m_driveControlMode, -xSpeed);
          mRearRightTalon.set(m_driveControlMode, xSpeed);
         } else if (xSpeed <= -0.5){
          mFrontLeftTalon.set(m_driveControlMode, -xSpeed);
          mFrontRightTalon.set(m_driveControlMode, xSpeed);
          mRearLeftTalon.set(m_driveControlMode, xSpeed);
          mRearRightTalon.set(m_driveControlMode, -xSpeed);
         }
         if (xSpeed == 0 || ySpeed==0){
          mFrontLeftTalon.set(m_driveControlMode, 0);
          mFrontRightTalon.set(m_driveControlMode, 0);
          mRearLeftTalon.set(m_driveControlMode, 0);
          mRearRightTalon.set(m_driveControlMode, 0);
         }
    }
         
        
        /*
         if rotate right, set all talons to 1, rotate left, set all talons to -1
         */
        
   


    
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
}

