package org.firstinspires.ftc;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import java.util.List;

@Autonomous(name = "TestAutoMove", group = "Opmode RamEaters")
public class TestAutoMove extends LinearOpMode {

    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    private DcMotor slideMotor = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private double TURN_P = 0.010;

    
    String test = "";
    
    


    
    @Override
    public void runOpMode() {


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");

 


        leftWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //imuInit();

        waitForStart();

        if (opModeIsActive()) {
            int r1 = 1;

            caseLoc(r1);


            sleep(15000);
        }



    }


    private void move(double drive,
                      double strafe,
                      double rotate, double power, int iSleep) {

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;

        powerLeftF = drive + strafe + rotate;
        powerLeftR = drive - strafe + rotate;

        powerRightF = drive - strafe - rotate;
        powerRightR = drive + strafe - rotate;

        leftWheelF.setPower(power);
        leftWheelR.setPower(power);
        rightWheelF.setPower(power);
        rightWheelR.setPower(power);

        leftWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftWheelF.setTargetPosition(leftWheelF.getCurrentPosition() + (int) (-powerLeftF));
        leftWheelR.setTargetPosition(leftWheelR.getCurrentPosition() + (int) (-powerLeftR));

        rightWheelF.setTargetPosition(rightWheelF.getCurrentPosition() + (int) (powerRightF));
        rightWheelR.setTargetPosition(rightWheelR.getCurrentPosition() + (int) (powerRightR));

        leftWheelF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(iSleep);

        leftWheelF.setPower(0);
        leftWheelR.setPower(0);
        rightWheelF.setPower(0);
        rightWheelR.setPower(0);

    }

   

    private void caseLoc(int loc) {

        gyroTurn(0);

       
        sleep(500);

       
        

        move(-500,0,0,0.5,500);
        telemetry.addData("test ", "move (%s)", "-500,0,0,0.5,500");

        move(500,0,0,0.5,500);
        telemetry.addData("test ", "move (%s)", "500,0,0,0.5,500");

        move(0,-1500,0,0.5,500);
        telemetry.addData("test ", "move (%s)", "0,-500,0,0.5,500");

        move(0,1500,0,0.5,500);
        telemetry.addData("test ", "move (%s)", "0,500,0,0.5,500");

        // call twice
        gyroTurn(0);

        move(0,0,200,0.3,500);

        gyroTurn(-90);

        move(0,0,-200,0.3,500);

        gyroTurn(0);

        sleep(1000);
        
        /*

        move(0,50,0,0.5,500);

        justTurn(-90);

        move(0,-50,0,0.5,500);


        gyroTurn(-180);

        move(0,50,0,0.5,500);


        justTurn(-90);

        */

        telemetry.update();

    }




    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }




    private void justTurn(double deg) {

        leftWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int i = 0;
        int iMAX = 200;

        double target_angle = getHeading() - deg;
        while (i < iMAX && opModeIsActive() && Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(error_degrees * TURN_P, -.6, .6); //Get Correction

            if (Math.abs(motor_output) < 0.020)
                i = 10001;
            // Send corresponding powers to the motors
            leftWheelF.setPower(1 * motor_output);
            leftWheelR.setPower(1 * motor_output);
            rightWheelF.setPower(1 * motor_output);
            rightWheelR.setPower(1 * motor_output);

            telemetry.addData("motor_output : ", motor_output);
            telemetry.addData("target_angle : ", target_angle);

            i++;
            telemetry.addData("i : ", i);
            telemetry.update();

        }


    }



    private void gyroTurn(double target_angle) {

        leftWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double currentHeading = getHeading();

        double delta = Math.abs((currentHeading - target_angle));

        int i = 0;
        int iMAX = 200;

        while (i < iMAX && opModeIsActive() && delta > 0.01)
        {

            double error_degrees = (target_angle - currentHeading) % 360.0;

            double motor_output = Range.clip(error_degrees * TURN_P, -0.6, 0.6);

            if (Math.abs(motor_output) < 0.020)
                i = 10001;

            leftWheelF.setPower(1 * motor_output);
            leftWheelR.setPower(1 * motor_output);
            rightWheelF.setPower(1 * motor_output);
            rightWheelR.setPower(1 * motor_output);


            currentHeading = getHeading();
            telemetry.addData("motor_output : ", motor_output);
            telemetry.addData("target_angle : ", target_angle);
            telemetry.addData("currentHeading : ", currentHeading);
            delta = Math.abs((currentHeading- target_angle));
            telemetry.addData("delta : ", delta);

            i++;
            telemetry.addData("i : ", i);
            telemetry.update();

        }

        //sleep(500);

        leftWheelF.setPower(0);
        leftWheelR.setPower(0);
        rightWheelF.setPower(0);
        rightWheelR.setPower(0);

    }
}
