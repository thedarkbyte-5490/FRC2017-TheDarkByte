package org.usfirst.frc5490.TestCode2017;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;

import org.opencv.core.Mat;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.analog.adis16448.frc.ADIS16448_IMU;

import org.usfirst.frc5490.TestCode2017.GripPipeline;


public class Robot extends SampleRobot {
	
    private SpeedController motor1Right;	// the motor to directly control with a joystick
    private SpeedController motor1Left;  
    private SpeedController motor2Left;
    private SpeedController motor2Right;
    
    private SpeedController doorMotor;
    private SpeedController clothMotor;
    
    private Joystick stick1;
    private Joystick stick2;
    private Joystick xbox;
    
    private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;
    private VisionThread visionThread;
    private double centerX;
    private final Object imgLock = new Object();
    
    
	int fCount = 0;


	private final double k_updatePeriod = 0.005; // update every 0.005 seconds/5 milliseconds (200Hz)
	
	//ADIS16448_IMU imu;
	
    public Robot() {
        
    	motor1Right = new Talon(0);	// RIGHT 
        motor1Left = new Talon(2); // LEFT
        motor2Right = new Talon(1);
        motor2Left = new Talon(3);
        doorMotor = new Talon(4);
        clothMotor = new Talon(5);
              
        stick1 = new Joystick(0);	// initialize the joystick on port 0 RIGHT
        stick2 = new Joystick(1);	// initialize the joystick on port 0 LEFT
        xbox = new Joystick(2);
             
    }
    
    @Override
    public void robotInit()	{
    	
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    	camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
    		fCount++;
    		if(!pipeline.filterContoursOutput().isEmpty())	{
    			Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    			synchronized (imgLock)	{
    				centerX = r.x + (r.width / 2);
    			}
    		}
    	});
    	visionThread.start();
    	
    	
    	
    }

    public void operatorControl() {
    	double speedLimit = stick1.getThrottle();
    	if(speedLimit <= 0) speedLimit = .1;

    	
        while (isOperatorControl() && isEnabled()) {
        	//RIGHT MOTOR SPEEDS
        	/*
        	if (Math.abs(stick1.getZ()) < 0.2)
        	{
        	double rightSpeed = stick1.getY();
        	if (Math.abs(rightSpeed) < 0.2)	{
        		rightSpeed = 0;
        	}
        	else	{
        		rightSpeed = (stick1.getY()*speedLimit);
        	}
        	
        	//FAIL SAFE
        	if (rightSpeed > 1) {
        		rightSpeed = 1;
        	} else if (rightSpeed < -1) {
        		rightSpeed = -1;
        	}
        	
        	//set motors
        	motor1Right.set(rightSpeed);
        	motor2Right.set(rightSpeed);
        	
        	
        	
        	//LEFT MOTOR SPEEDS
        	double leftSpeed = stick2.getY();
        	if (Math.abs(leftSpeed) < 0.2)	{
        		leftSpeed = 0;
        	}
        	else	{
        		leftSpeed = (stick2.getY()*speedLimit);
        	}
        	
        	//FAIL SAFE
        	if (leftSpeed > 1) {
        		leftSpeed = 1;
        	} else if (leftSpeed < -1) {
        		leftSpeed = -1;
        	}
        	
        	//set motors
        	motor1Left.set(-leftSpeed);
        	motor2Left.set(-leftSpeed);
        }
        	else
        	{
        		motor1Right.set(stick1.getZ());
        		motor2Right.set(stick1.getZ());
        		motor1Left.set(stick1.getZ());
        		motor2Left.set(stick1.getZ());
        	}
        	*/
        	
        	double yy = (Math.pow(stick1.getY(), 3)*(speedLimit));
        	double xx = (Math.pow(stick1.getX(), 3)*(speedLimit));
        	double zz = (Math.pow(stick1.getZ(), 3)*(speedLimit));
        	
        	if (yy < 0.05) yy = 0;
        	if (xx < 0.05) xx = 0;
        	if (zz < 0.05) zz = 0;
        	
        	motor1Right.set(yy+xx+zz);
    		motor1Left.set(yy-xx-zz);
    		
    		if(Math.abs(yy+xx+zz) > 1)
    		{
    			motor1Right.set(1);
        		motor1Left.set(1);
    		}
    		
    		if(Math.abs(yy-xx-zz) > 1)
    		{
    			motor1Right.set(1);
        		motor1Left.set(1);
    		}
    		
    		motor2Right.set(motor1Right.get());
    		motor2Left.set(motor1Left.get());
        	
//        	if (Math.abs(zz) < 0.5)
//        	{
//        		motor1Right.set(0);
//        		motor2Right.set(0);
//        		motor1Left.set(0);
//        		motor2Left.set(0);
//        	}
//        	else
//        	{
//        		motor1Right.set(zz);
//        		motor2Right.set(zz);
//        		motor1Left.set(-zz);
//        		motor2Left.set(-zz);
//        	}
//        	if(Math.abs(xx) < 0.1)
//        	{
//        		motor1Right.set(0);
//        		motor2Right.set(0);
//        		motor1Left.set(0);
//        		motor2Left.set(0);
//        	}
//        	if (xx>0.1)
//        	{
//        		motor1Right.set(0);
//        		motor2Right.set(0);
//        		motor1Left.set(xx);
//        		motor2Left.set(xx);
//        	}
//        	else
//        	{
//        		motor1Right.set(xx);
//        		motor2Right.set(xx);
//        		motor1Left.set(0);
//        		motor2Left.set(0);
//        	}
        	/*//speed control - changes max speed
        	boolean button5 = stick1.getRawButton(5); // 3/4th speed 0.75
        	boolean button3 = stick1.getRawButton(3); // half speed 0.5
        	boolean button6 = stick1.getRawButton(6); // full speed 1.0
        	boolean button4 = stick1.getRawButton(4); // slowest speed 0.2
        	
        	//drive straight
        	boolean button2 = stick1.getRawButton(2); //drive straight based on speedLimit
        	boolean button1 = stick1.getRawButton(1); //drive straight slowly af
        	
        	
        	boolean button10 = stick1.getRawButton(10);
        	boolean button9 = stick1.getRawButton(9);
        	
        	boolean button11 = stick1.getRawButton(11);
        	boolean button12 = stick1.getRawButton(12);
        	
        	boolean button7 = stick1.getRawButton(7);
        	boolean button8 = stick1.getRawButton(8); */
        	
        	
        	double xboxleft = xbox.getRawAxis(1);
        	boolean a = xbox.getRawButton(1);
        	boolean y = xbox.getRawButton(4);
        	boolean x = xbox.getRawButton(3);
        	boolean b = xbox.getRawButton(2);
        	
        	
        	/*//fastest is 6 down to slowest is 3
        	if (button6) {
        		speedLimit = 1;
        	}
        	
        	if (button5) {
        		speedLimit = 0.75;
        	}
        	
        	if (button4) { 
        		speedLimit = 0.5;
        	}
        	
        	if (button3) {
        		speedLimit = 0.25;
        	}*/
        	
        	if (Math.abs(xboxleft) > 0.2) doorMotor.set(xboxleft/2);
        	if (Math.abs(xboxleft) < 0.2) doorMotor.set(0);
        	
        	/*if (button9) clothMotor.set(-1);
        	if (button10) clothMotor.set(1);
        	if (!(button9 || button10)) clothMotor.set(0);
        	
        	if (button7 || y) 
        		{
        			doorMotor.set(0.5);
        			Timer.delay(1);
        			doorMotor.set(0);
        		}
        	if (button8 || a) 
        		{
        			doorMotor.set(-0.5);
        			Timer.delay(2);
        			doorMotor.set(0);
        		}
        	*/
        	if (x)
        	{
        		doorMotor.set(-0.5);
    			Timer.delay(1.2);
    			doorMotor.set(0);
        	}
        	if (b)
        	{
        		doorMotor.set(0.5);
    			Timer.delay(1.7);
    			doorMotor.set(0);
        	}
        		
        	
            Timer.delay(k_updatePeriod);	// wait 5ms to the next update

        }
        
    }
    public void autonomous() {
    	
    	//double normSpeed = 0.3;
    	
    	while (isAutonomous() && isEnabled())	{
    		double centerX;
    		synchronized (imgLock)	{
    			centerX = this.centerX;
    		}

    		SmartDashboard.putNumber("centerX", centerX);
    		SmartDashboard.putNumber("frames", fCount);
    		Timer.delay(1/20);
    		
    		/*
    		if (turn > 0)
    		{
    			motor1Right.set(normSpeed + turn/5);
    			motor2Right.set(normSpeed + turn/5);
    			motor1Left.set(normSpeed);
    			motor2Left.set(normSpeed);
    		}
    		else
    		{
    			motor1Right.set(normSpeed);
    			motor2Right.set(normSpeed);
    			motor1Left.set(normSpeed + turn/5);
    			motor2Left.set(normSpeed + turn/5);
    		}
    		 */
    	}

    }

}
