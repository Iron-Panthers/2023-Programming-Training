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
    private int multiTH = 1; // wheels = t(left Front)   y(Right Front)
    private int multiYG = 1; //          g(left back)    h(Right back)

    private ControlMode m_driveControlMode = ControlMode.PercentOutput;

    public DriveSubsystem( TalonSRX mFrontLeftTalon, TalonSRX mRearLeftTalon, TalonSRX mFrontRightTalon, TalonSRX mRearRightTalon) {
        this.mFrontLeftTalon = mFrontLeftTalon;
        this.mRearLeftTalon = mRearLeftTalon;
        this.mFrontRightTalon = mFrontRightTalon;
        this.mRearRightTalon = mRearRightTalon;
    }

   public void drive(DoubleSupplier yLSpeed, DoubleSupplier xLSpeed, DoubleSupplier yRSpeed,DoubleSupplier xRSpeed)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        // mRobotDrive.driveCartesian(ySpeed, xSpeed, zRot, 0.0); th right
        // Wheels represented by T Y
        //                     G H
        double y = yLSpeed.getAsDouble();
        double x = xLSpeed.getAsDouble();
        //double rY = yRSpeed.getAsDouble();
        double rX = xRSpeed.getAsDouble();
        
        if (y>x)// Straight
        {
          multiTH = -1;
        }
        else
        {
          multiTH = 1;
        }
        if (x>(y*-1))
        {
          multiYG = -1;
        }
        else
        {
          multiYG = 1;
        }
        double speed1 = Math.abs((Math.sqrt((x*x)+(y*y)))); // 1 finding distance of joystick to center 
        double speedTH = 1-(Math.abs(0.5*(y+x)));
        double speedYG = Math.abs(0.5*(y + x)); // 0.5finding solution to split line equation x=y and joystick locaiton equation, y=-x+xSpeed+ySpeed
        double speedTurn = rX;
        double[][] talonSpeeds = new double[2][2];

        talonSpeeds[0][0]=speed1*multiTH*speedTH;
        talonSpeeds[0][1]=speed1*multiYG*speedYG;
        talonSpeeds[1][0]=speed1*multiYG*0.7*speedYG;
        talonSpeeds[1][1]=speed1*multiTH*0.7*speedTH;

        for (int i=0;i>talonSpeeds.length;i++)
        {
          int i1=i-1;
          for (int j1=2;j1>talonSpeeds.length;j1--)
          {
            
            if (speedTurn*(j1+j1-1)*talonSpeeds[i1][j1]<0)//times -1 if TY, if wheel speed and turn are the same direction
            { 
              talonSpeeds[i1][j1]*=speedTurn;
            }
            else // wheel speed and turn diff direction 
            {
              talonSpeeds[i1][j1]*=1-speedTurn; 
            }
          }
        }


        mFrontLeftTalon.set(m_driveControlMode, talonSpeeds[0][0]);
        mFrontRightTalon.set(m_driveControlMode,talonSpeeds[0][1]);
        mRearLeftTalon.set(m_driveControlMode, talonSpeeds[1][0]);
        mRearRightTalon.set(m_driveControlMode,talonSpeeds[1][1]);
    }//end of method


    
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

