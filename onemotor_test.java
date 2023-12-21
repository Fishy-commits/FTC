
package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "onemotor_test", group = "Opmode RamEaters")

public class onemotor_test extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // Declare Hardware
    private DcMotor liftmotor1 = null;               //Lift Motor 1
    private int target1 = 0;
     
    @Override
    public void init() {

 
        liftmotor1 = hardwareMap.get(DcMotor.class, "Em1");
        liftmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        runtime.reset();
        move();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    private void move() {
        
        
        if (gamepad2.x) { //when x pressed, grab pixel position
            
            
            
            target1 = -250;
            liftmotor1.setTargetPosition(target1);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setPower(1);            
            telemetry.addData("Status", "gamepad2.x");
            telemetry.update();
            
        }
        else if (gamepad2.y) {  // when y pressed, goes back to init position
            
            
            target1 = 0;
            
           
            liftmotor1.setTargetPosition(target1);
            
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            liftmotor1.setPower(1);
            
            telemetry.addData("Status", "gamepad2.y");
            telemetry.update();
            
        } 
    }
}
