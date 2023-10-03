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

   public void drive(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRot)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        double mXSpeed = xSpeed.getAsDouble();
        double mYSpeed = -ySpeed.getAsDouble();
        double mZRot = zRot.getAsDouble();

        double[][] talonSpeeds = new double[2][2];

        //reset speeds
        for(int i = 0; i < talonSpeeds.length; i++){
          for(int j = 0; j < talonSpeeds[i].length; j++){
            talonSpeeds[i][j] = 0d;
          }
        }

        //foreward motion
        for(int i = 0; i < talonSpeeds.length; i++){
          for(int j = 0; j < talonSpeeds[i].length; j++){
            talonSpeeds[i][j] += mYSpeed;
          }
        }

        //sideways motion
        for(int i = 0; i < talonSpeeds.length; i++){
          for(int j = 0; j < talonSpeeds[i].length; j++){
            talonSpeeds[i][j] += (((i == j)?1:-1) * mXSpeed);
          }
        }

        //rotation
        for(int i = 0; i < talonSpeeds.length; i++){
          for(int j = 0; j < talonSpeeds[i].length; j++){
            talonSpeeds[i][j] += (((j == 0)?1:-1) * mZRot);
          }
        }
        
        //set the speeds
        mFrontLeftTalon.set(m_driveControlMode, talonSpeeds[0][0]);
        mFrontRightTalon.set(m_driveControlMode, talonSpeeds[0][1]);
        mRearLeftTalon.set(m_driveControlMode, talonSpeeds[1][0]);
        mRearRightTalon.set(m_driveControlMode, talonSpeeds[1][1]);
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
   * 
   * @param controlMode control mode to use setting talon output
   * @param velocityScale velocity for full scale in ticks/100ms
   */
  public void setControlMode(ControlMode controlMode, double velocityScale) {
    m_driveControlMode = controlMode;
  }
}

