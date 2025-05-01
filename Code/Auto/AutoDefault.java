package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Back & Forth", group="Linear OpMode")
public class AutoDefault extends LinearOpMode {
    private ElapsedTime timer;
    private static final double START_DELAY = 1000.0f; // ms
    private static final double SWAP_DELAY = 400.0f; // ms
    // private static final double START_DELAY = 10000.0; // ms
    // TODO: find better way and make sure this is set right
    
    // DriveTrain
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    // Need for calling setPower?
    private static final double MOTOR_SPEED = 0.7f;
    
    // Drive Train Encoders
    private static final double TICKS_PER_REV = 1464; // For Studica Maverick
    private static final double GEAR_RATIO = 1.0;
    private static final double TOP_VELOCITY = (100f / 60f) * TICKS_PER_REV; // 100 RPM -> Ticks Per Second
    private static final double WHEEL_DIAM = 100f; // mm
    private static final double DISTANCE_PER_REV = WHEEL_DIAM * Math.PI * GEAR_RATIO;
    private static final double DISTANCE_PER_TICK = DISTANCE_PER_REV / TICKS_PER_REV;
    private static final double ROBOT_LENGTH = 305f; // mm  TODO: determine once front added
    private static final double AREA_LENGTH = 169.5f * 25.4f; // inches -> mm
    private static final int TRAVEL_DISTANCE = (int)(AREA_LENGTH - ROBOT_LENGTH);
    private static final int TRAVEL_DISTANCE_TICKS = (int)(TRAVEL_DISTANCE / DISTANCE_PER_TICK);
    private static final int TRAVEL_THRESHOLD = (int)(60.0f / DISTANCE_PER_TICK); // 60mm -> ticks
    private boolean game_started = false;
    private boolean is_leaving_start = true;
    private double target_velocity = TOP_VELOCITY * MOTOR_SPEED;
    private int target_position = 0;
    private boolean swapping = false;
    
    
    private boolean isWithinThreshold() {
        boolean within_thresh = Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) <= TRAVEL_THRESHOLD && Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) <= TRAVEL_THRESHOLD;
        return within_thresh || (!leftMotor.isBusy() && !rightMotor.isBusy());
    }
    
    private void swapTarget() {
        leftMotor.setTargetPosition(target_position);
        rightMotor.setTargetPosition(target_position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        target_velocity *= -1;
        leftMotor.setVelocity(target_velocity);
        rightMotor.setVelocity(target_velocity);
        swapping = false;
        // TODO: add set power?
    }

    @Override
    public void runOpMode() {
        // Init
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // Encoder Reset
        // NOTE: make sure robot starts at proper location
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        timer = new ElapsedTime();
        game_started = false;
        is_leaving_start = true;
        while (opModeIsActive()) {
            if (timer.milliseconds() > START_DELAY && !game_started) {
                game_started = true;
                target_position = TRAVEL_DISTANCE_TICKS;
                swapTarget();
            }
            
            if (game_started) {
                if (isWithinThreshold() && !swapping) {
                    leftMotor.setVelocity(0.0f);
                    rightMotor.setVelocity(0.0f);
                    target_position = (is_leaving_start) ? 0 : TRAVEL_DISTANCE_TICKS;
                    is_leaving_start = !is_leaving_start;
                    swapping = true;
                    timer.reset();
                }
                
                if (swapping && timer.milliseconds() > SWAP_DELAY) {
                    swapTarget();
                }
            }
            
            // Telemetry
            telemetry.addData("Waiting", !game_started);
            telemetry.addData("Robot Position (inches)", (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2 * DISTANCE_PER_TICK / 25.4);
            telemetry.addData("Robot Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Robot Right Position", rightMotor.getCurrentPosition());
            telemetry.addData("Left Target Position", rightMotor.getTargetPosition());
            telemetry.addData("Right Target Position", rightMotor.getTargetPosition());
            telemetry.addData("Left Velocity", leftMotor.getVelocity());
            telemetry.addData("Right Velocity", rightMotor.getVelocity());
            telemetry.addData("Target Velocity", target_velocity);
            telemetry.update();
        }

    }
}
