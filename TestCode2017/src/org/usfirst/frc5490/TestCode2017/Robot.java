package org.usfirst.frc5490.TestCode2017;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends SampleRobot {
    
    private SpeedController motor1Right;
    private SpeedController motor1Left;  
    private SpeedController motor2Left;
    private SpeedController motor2Right;
    
    private SpeedController doorMotor;
    private SpeedController clothMotor;
    
    private Joystick stick;
    private Joystick xbox;
    
    
    private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;
    private VisionThread visionThread;
    private double centerX = 0;
    private final Object imgLock = new Object();
    public UsbCamera camera;

    

   

    RobotDrive drive;
    //public SerialPort com;
    
    Timer timer;
    //public UsbCamera camera;

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
        camera = new UsbCamera("camera", "cam2");
    }
    
    @Override
    public void robotInit() {
        drive = new RobotDrive(motor1Left,motor2Left,motor1Right,motor2Right);
        Timer timer = new Timer();
        timer.start();
        /*
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
        */
    	camera = CameraServer.getInstance().startAutomaticCapture();
    	camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	    	
    	visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
    		if(!pipeline.filterContoursOutput().isEmpty())	{
    			Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    			synchronized (imgLock)	{
    				centerX = r.x + (r.width / 2);
    			}
    		}
    	});
    	visionThread.start();
    }

    public void motorSet(double left, double right) {
        motor1Left.set(left);
        motor2Left.set(left);
        motor1Right.set(right);
        motor2Right.set(right);
    }

    public void operatorControl() {
    	//timer.reset();
    	double startTime = 0;
    	double doorTime = 0;
    	int direction = 0;
    	
    	boolean unloaded = true;
    	double clothTime = 4.5;
    	
        while (isOperatorControl() && isEnabled()) {

            double xboxLeft = xbox.getRawAxis(1);
            double xboxRight = xbox.getRawAxis(5);
            boolean a = xbox.getRawButton(1);
            boolean b = xbox.getRawButton(2);
            boolean x = xbox.getRawButton(3);
            boolean y = xbox.getRawButton(4);
            boolean left = xbox.getRawButton(5);

            double yy = stick.getY();
            double xx = stick.getX();
            double zz = stick.getZ();
            
           
            //speed limit
            
            
            
         
            double speedLimit = -0.8;
            boolean button25Percent = stick.getRawButton(5);
            boolean button50Percent = stick.getRawButton(3);
            boolean button75Percent = stick.getRawButton(4);
            boolean button100Percent = stick.getRawButton(6);
            boolean buttonMaximumOverdrive = stick.getRawButton(1);
            
            if (button25Percent) { speedLimit = -0.8 * 0.25; }
            if (button50Percent) { speedLimit = -0.8 * 0.50; }
            if (button75Percent) { speedLimit = -0.8 * 0.75; }
            if (button100Percent) { speedLimit = -0.8 * 1.00; }
            
            if (buttonMaximumOverdrive) {
            	drive.arcadeDrive(-1*stick.getY(), -1*stick.getX());
            } else {
            	drive.arcadeDrive(speedLimit*stick.getY(), speedLimit*stick.getX());
            }
            
                      
            if (Math.abs(xboxRight) > 0.2) doorMotor.set(xboxRight/2);
            else doorMotor.set(0);
            
            if (Math.abs(xboxLeft) > 0.2) clothMotor.set(-1*xboxLeft/2);
            else clothMotor.set(0);
            
            if (y) {
            	startTime = timer.get();
            	doorTime = 1;
            	direction = 1; // 1 (up) , -1 (down)
            }
            if (a) {
            	startTime = timer.get();
            	doorTime = 2;
            	direction = -1; // 1 (up) , -1 (down)
            }
            if (x) {
            	startTime = timer.get();
            	doorTime = 1.2;
            	direction = -1; // 1 (up) , -1 (down)            
            }
            if (b) {
            	startTime = timer.get();
            	doorTime = 0.5;
            	direction = 1; // 1 (up) , -1 (down)            
            }
            if (left) {
            	if (unloaded) {
            		startTime = timer.get();
            		direction = 1;
            	}
            	else {
            		startTime = timer.get();
            		direction = -1;
            	}
            }
            //if (timer.get() - startTime < doorTime) doorMotor.set(0.5*direction);
        	//else doorMotor.set(0);
            
            //if (timer.get() - startTime < clothTime) clothMotor.set(0.5*direction);
        	//else clothMotor.set(0);
        	
            Timer.delay(k_updatePeriod);
        }
    }

    public void autonomous() {
    	while (isAutonomous() && isEnabled()) {
    		//motorSet(0.2,-0.18);
    		
    		double centerX;
    		synchronized (imgLock)	{
    			centerX = this.centerX;
    		}

    		System.out.println(centerX);
    		Timer.delay(k_updatePeriod);
    	}
    

    }
    
}


