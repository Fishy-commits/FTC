
package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "Allcode", group = "Opmode RamEaters")

public class Allcode extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // Declare Hardware properties
    private DcMotor leftWheelF = null;              //Left Wheel Front
    private DcMotor leftWheelR = null;              //Left Wheel Back
    private DcMotor rightWheelF = null;             //Right Wheel Front
    private DcMotor rightWheelR = null;				//Right Wheel Back
    private DcMotor liftmotor0 = null;              //Lift Motor 0 to control the primary single bar
    private DcMotor liftmotor1 = null;              //Lift Motor 1 to control the secondary singale bar
    private DcMotor liftmotor2 = null;				//Lift Motor 2 to control the primary single bar
    private Servo clawCenter = null;				//Servo to tune the angle of the claw holder
    private Servo clawLeft = null;					//Claw Left Servo
    private Servo clawRight = null;					//Claw Right Servo
    private Servo airplane = null;					//Airplan Servo to launch the paper airplane
    private int target0 = 0;
    private int target1 = 0;
    private int target2 = 0;
    private double tclawCenter = 0.50;				//Claw Center Servo initial position
	private double planeTarget = 0.5;				//Airplane Servo initial position
	
    @Override
	/*At the beginning, all motors and servos are set at the init status.
	* Make sure hardware definition matches the Control Station (Android phone) configuration file.
	* All Names are case sensitive.
	*/
    public void init() {
		//4 Wheels are connected to the Control Hub - Motor ports
 		leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");
		
		//3 Lift Motors are connected to the Expansion - Motor ports
        liftmotor0 = hardwareMap.get(DcMotor.class, "Em0");
        liftmotor1 = hardwareMap.get(DcMotor.class, "Em1");
        liftmotor2 = hardwareMap.get(DcMotor.class, "Em2");
		
        liftmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
        liftmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		
		//3 Servos are connected to the Control Hub-Servo ports
        clawCenter = hardwareMap.get(Servo.class, "Es0");
        clawCenter.setPosition(tclawCenter);
		
        clawLeft = hardwareMap.get(Servo.class, "Es1");
        clawRight = hardwareMap.get(Servo.class, "Es2");
		
		airplane = hardwareMap.get(Servo.class, "s5");
        airplane.setPosition(planeTarget);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        runtime.reset();

        //sleep(1000);
        move();
    }

    @Override
    public void start() {
        runtime.reset();
    }
    
	//To close the claw 
    private void closeClaw(){
		clawLeft.setPosition(0.2);
        clawRight.setPosition(0);
	}
	
	//To open the claw
	private void openClaw() {
		clawLeft.setPosition(0);
        clawRight.setPosition(0.2);
	}
	
	//To grab the pixel, move both primary and secondary single bars to their positions
	private void grabPixel() {
		target2 = 1000;
		target0 = 0 - target2;
		target1 = 1050;
		liftmotor0.setTargetPosition(target0);
		liftmotor1.setTargetPosition(target1);
		liftmotor2.setTargetPosition(target2);
		liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor0.setPower(0.1);
		liftmotor1.setPower(0.1);
		liftmotor2.setPower(0.1);
		telemetry.addData("Status", "gamepad2.x");
		telemetry.update();
	} 
	
	//To move both primary and secondary single bars to their initial positions
	private void initPosition() {
		target0 = 0;
		target1 = 0;
		target2 = 0;
		
		liftmotor0.setTargetPosition(target0);
		liftmotor1.setTargetPosition(target1);
		liftmotor2.setTargetPosition(target2);
		liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		liftmotor0.setPower(0.15);
		liftmotor1.setPower(0.1);
		liftmotor2.setPower(0.15);
		
		clawCenter.setPosition(0.50);
		
		telemetry.addData("Status", "gamepad2.y");
		telemetry.update();
	}
	
	//To move the robot to the board, move both primary and secondary single bars to their positions
	private void backboardPosition(){
		target2 = 875;
		target0 = 0 - target2;
		target1 = 500;
		
		liftmotor0.setTargetPosition(target0);
		liftmotor1.setTargetPosition(target1);
		liftmotor2.setTargetPosition(target2);
		liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		liftmotor0.setPower(0.15);
		liftmotor1.setPower(0.1);
		liftmotor2.setPower(0.15);
		telemetry.addData("Status", "gamepad2.a");
		telemetry.update();
	}
	
	//To put the pixel to the board, move both primary and secondary single bars to their positions
	private void putPixel(){
		target2 = 650;
		target0 = 0 - target2;
		target1 = 500;
		
		liftmotor0.setTargetPosition(target0);
		liftmotor1.setTargetPosition(target1);
		liftmotor2.setTargetPosition(target2);
		liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		liftmotor0.setPower(0.15);
		liftmotor1.setPower(0.1);
		liftmotor2.setPower(0.15);
		telemetry.addData("Status", "gamepad2.b");
		telemetry.update();
	}
		
	/* Control 4 wheels to move the robot
	*/
    private void move() {
        double drive;
        //Power for forward and back motion
        double strafe;				// Power for left and right motion
        double rotateLeft;			// Power for the robot counterclockwise rotation
        double rotateRight;			// Power for the robot clockwise rotation
        //int intake;

        double drive2;
        double strafe2;

        drive = -gamepad1.left_stick_y;		// Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;
        //intake = gamepad2.left_trigger;

        drive2 = -gamepad1.right_stick_y;
        strafe2 = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;
        // double powerIntake;
        //intakeWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeWheel1.setPower(1);
        //if full power on left stick
        
        if (gamepad2.x) {		//when the X button of gamepad 2 is pressed, the robot is at the grab pixel position
            
           grabPixel();
           
        }
        else if (gamepad2.y) {	// when the Y button of gamepad 2 is pressed, the robot returns to the init position
           
		   initPosition();

        } else if (gamepad2.a) {//when the A button of gamepad 2 is pressed, the robot is at the backboard position
            
		   backboardPosition();
    
        } else if (gamepad2.b) {//when the B button of gamepad 2 is pressed, the robot is at the put pixel position
            
           putPixel();
            
        }  
        else if (gamepad2.right_bumper) {	//when the right_bumper of gamepad 2 is pressed, claws are closed
           
			closeClaw();

        } else if (gamepad2.left_bumper) {	//when the left_bumper of gamepad2 is pressed, claws are opened
            
            openClaw();
           
        } else if (gamepad2.dpad_left) {	//when the dpad_left of gamepad 2 is pressed, airplane servo releases the airplane
            
           airplane.setPosition(0);
		   telemetry.addData("Status", "gamepad2.dpad_left");
           telemetry.update();
        
        } else if (gamepad2.dpad_up) {		//when the dpad_up of gamepad 2 is pressed, claws rotate upwards
            
           tclawCenter = tclawCenter + 0.001;
           clawCenter.setPosition(tclawCenter);
           telemetry.addData("Status", "gamepad2.dpad_up");
           telemetry.update();
        
        } else if (gamepad2.dpad_down) {	//when dpad_down of gamepad 2 is pressed, claws rotate downwards
            
          tclawCenter = tclawCenter - 0.001;
          clawCenter.setPosition(tclawCenter);
          telemetry.addData("Status", "gamepad2.dpad_down");
          telemetry.update();
        
        } else if (gamepad1.dpad_up) {		//when dpad-up of gamepad 1 is pressed, the robot raises both bars to get into hanging position
            
           target2 = 500;
           target0 = 0 - target2;
           target1 = 400;
           liftmotor0.setTargetPosition(target0);
           liftmotor1.setTargetPosition(target1);
           liftmotor2.setTargetPosition(target2);
           liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           liftmotor0.setPower(0.15);
           liftmotor1.setPower(0.1);
           liftmotor2.setPower(0.15);
           telemetry.addData("Status", "gamepad1.dpad_up");
           telemetry.update();
		   
           clawLeft.setPosition(0.2);
           clawRight.setPosition(0); 
        
        } else if (gamepad1.dpad_down) { //when dpad-down of gamepad 1 is pressed, the robot retracks the bar connecting to the claw
            
            target2 = 500;
            target0 = 0 - target2;
            target1 = 3000;
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);
           
            telemetry.addData("Status", "gamepad1.dpad_down");
            telemetry.update();
         
        }
		else if (gamepad1.dpad_left) { //when dpad-left of gamepad 1 is pressed, the robot increases the tightness of the hanging position
			target2 = 700;
			target0 = 0 - target2;
			target1 = 3400;
			liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);
			
            telemetry.addData("Status", "gamepad1.dpad_left");
            telemetry.update();
			
        } else if (gamepad1.dpad_right) {  //when dpad-right of gamepad 1 is pressed, the robot decreases the tightness of the hanging position
			target2 = 300;
			target0 = 0 - target2;
			target1 = 3400;
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);

            telemetry.addData("Status", "gamepad1.dpad_right");
            telemetry.update();
			
        } else{
			// Do nothing
		}
		
		
		if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) { //when using the left joystick, all wheels receive 0.5 power
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            //powerIntake = intake;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.5);
            leftWheelR.setPower(-powerLeftR*0.5);

            rightWheelF.setPower(powerRightF*0.5);
            rightWheelR.setPower(powerRightR*0.5);

            //intakeWheel1.setPower(powerIntake);

        }
		else {
            //when using the right joystick, all wheels receive 0.3 power
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;

            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.3);
            leftWheelR.setPower(-powerLeftR*0.3);

            rightWheelF.setPower(powerRightF*0.3);
            rightWheelR.setPower(powerRightR*0.3);
        }
    }
}