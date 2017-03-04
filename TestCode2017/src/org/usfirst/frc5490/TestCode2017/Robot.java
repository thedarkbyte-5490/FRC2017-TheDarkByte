package org.usfirst.frc5490.TestCode2017;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;

public class Robot extends SampleRobot {
    
    private SpeedController motor1Right;
    private SpeedController motor1Left;  
    private SpeedController motor2Left;
    private SpeedController motor2Right;
    
    private SpeedController doorMotor;
    private SpeedController clothMotor;
    
    private Joystick stick;
    private Joystick xbox;

    RobotDrive drive;
    //public SerialPort com;
    
    Timer timer;
    public UsbCamera camera;

    private final double k_updatePeriod = 0.005; // update every 5 milliseconds (200Hz)
    
    //ADIS16448_IMU imu;
    
    public Robot() {
        
        motor1Right = new Talon(0);
        motor2Right = new Talon(1);

        motor1Left = new Talon(2);
        motor2Left = new Talon(3);

        doorMotor = new Talon(4);
        clothMotor = new Talon(5);
              
        stick = new Joystick(0);
        xbox = new Joystick(1);
        
        //com = new SerialPort(9600, SerialPort.Port.kMXP);
        camera = new UsbCamera("camera", 2);
    }
    
    @Override
    public void robotInit() {
        drive = new RobotDrive(motor1Right,motor2Right,motor1Left,motor2Left);
        Timer timer = new Timer();
        
        new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();
    }

    public void motorSet(double left, double right) {
        motor1Left.set(left);
        motor2Left.set(left);
        motor1Right.set(right);
        motor2Right.set(right);
    }

    public void operatorControl() {
    	timer.reset();
    	double startTime;
    	double doorTime;
    	int direction;
    	
        while (isOperatorControl() && isEnabled()) {

            double xboxLeft = xbox.getRawAxis(1);
            double xboxRight = xbox.getRawAxis(2);
            boolean a = xbox.getRawButton(1);
            boolean b = xbox.getRawButton(2);
            boolean x = xbox.getRawButton(3);
            boolean y = xbox.getRawButton(4);

            double yy = stick.getY();
            double xx = stick.getX();
            double zz = stick.getZ();

            if (Math.pow(xx*xx+yy*yy,0.5) > 0.3) {
                drive.arcadeDrive(stick);
            }
            else if (Math.abs(zz) > 0.25) {
                motorSet(-zz, -zz); //
            }
            else {
                motorSet(0, 0);
            }
            
            if (Math.abs(xboxRight) > 0.2) doorMotor.set(xboxRight/2);
            else doorMotor.set(0);
            
            if (Math.abs(xboxLeft) > 0.2) clothMotor.set(xboxLeft/2);
            else clothMotor.set(0);
            
            if (y) {
            	startTime = timer.get();
            	doorTime = 1;
            	direction = 1; // 1 (up) , -1 (down)
            }
            if (a) {
            	startTime = timer.get();
            	doorTime = 2;
            	direction = -1; // 1 (up) , -1 (down)            }
            if (x) {
            	startTime = timer.get();
            	doorTime = 1.2;
            	direction = -1; // 1 (up) , -1 (down)            }
            if (b) {
            	startTime = timer.get();
            	doorTime = 0.5;
            	direction = 1; // 1 (up) , -1 (down)            }
        	if (timer.get() - startTime < doorTime) doorMotor.set(0.5*direction);
        	else doorMotor.set(0);
        	
            Timer.delay(k_updatePeriod);
        }
    }

    public void autonomous() {
    	while (isAutonomous() && isEnabled()) {
    		
    		
    		Timer.delay(k_updatePeriod);
    	}
    }
}



