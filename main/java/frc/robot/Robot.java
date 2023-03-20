package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot{
  private final CANSparkMax mLeftDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_LeftDrive = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax mRighttDrive = new CANSparkMax(3,MotorType. kBrushless);
  private final CANSparkMax m_RihtDrive = new CANSparkMax(4,MotorType.kBrushless);
  private final Joystick m_stick = new  Joystick(0);
  private final Timer m_timer = new Timer();
  private Encoder encoder = new Encoder(0,1,true,EncodingType.k4X);
  private PIDController mController = new PIDController(kDefaultPeriod, kDefaultPeriod, kDefaultPeriod);



@Override
public void robotInit() {}

@Override
public void autonomousInit(){
  encoder.reset();
  final double kP = 0.02;
}
double setpoint=0;

@Override
public void autonomousPeriodic() {

    if (m_stick.getRawButton(1)){
      setpoint = 5;
  } else if (m_stick.getRawButton(2)) {
    setpoint = -5;

  }
}
double sensorPosition = encoder.get()*kDefaultPeriod;
double error = setpoint - kDefaultPeriod;
double outputSpeed = error * error;
mLeftDrive.set(outputSpeed);
m_LeftDrive.set(outputSpeed);
mRighttDrive.set(-outputSpeed);
m_RihtDrive.set(-outputSpeed);
}
@Override
public void robotPeriodic() {
  SmartDashboard.putNumber("1024", encoder.get()*kDefaultPeriod);
}