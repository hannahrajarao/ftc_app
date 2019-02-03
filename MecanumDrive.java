package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS, on 1/8/2018.
 */

public class MecanumDrive {
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    boolean usingEncoders = false;

    //Driving encoder values
    static final double TICKS_PER_MOTOR_REV = 1120;	//Ticks per motor revolution
    static final double DRIVE_GEAR_REDUCTION = 1.5;	//This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER = 4.0;
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);

    public void init(HardwareMap hwMap, boolean encoder) {
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        setEncoder(encoder);
        usingEncoders = encoder;

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(Gamepad gamepad1, OpMode opmode) {
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,
                gamepad1.right_stick_y, gamepad1.right_trigger, opmode);
    }

    /**
     * Teleop mecanum drive
     * @param leftx left_stick_x value of gamepad
     * @param lefty left_stick_y value of gamepad
     * @param rightx right_stick_x value of gamepad
     * @param righty right_stick_y value of gamepad
     * @param rtrigger right trigger value of gamepad
     * @param opmode used to update telemetry values for driver
     */

    public void drive(double leftx, double lefty, double rightx, double righty, double rtrigger, OpMode opmode) {
        lefty = -lefty;
        righty = -righty;

        //values for each motor
        double rightF = (lefty - rightx) + leftx;
        double rightB = (lefty - rightx) - leftx;
        double leftF = (lefty + rightx) - leftx;
        double leftB = (lefty + rightx) + leftx;

        //motor power is proportional to pushing of the right trigger - this acts as
        //a power reducer and a brake system
        rightF *= (1 - rtrigger);
        rightB *= (1 - rtrigger);
        leftF *= (1 - rtrigger);
        leftB *= (1 - rtrigger);

        //set power to each wheel
        leftFront.setPower(leftF);
        leftBack.setPower(leftB);
        rightFront.setPower(rightF);
        rightBack.setPower(rightB);

        opmode.telemetry.addData("leftF", "%.2f", leftF);
        opmode.telemetry.addData("rightF", "%.2f", rightF);
        opmode.telemetry.addData("leftB", "%.2f", leftB);
        opmode.telemetry.addData("rightB", "%.2f", rightB);
        opmode.telemetry.addData("rtrigger", (1 - rtrigger));
    }

    public void doubleDrive(Gamepad gamepad1, Gamepad gamepad2, OpMode opmode) {
        if(gamepad1.atRest()) {
            drive(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x,
                    gamepad2.right_stick_y, gamepad2.right_trigger, opmode);
        }
        else {
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad1.right_stick_y, gamepad1.right_trigger, opmode);
        }
    }

    public void driveForward(double distance, double power) {

        leftFront.setTargetPosition((int)(leftFront.getCurrentPosition() + (distance * TICKS_PER_INCH)));
        leftBack.setTargetPosition((int)(leftBack.getCurrentPosition() + (distance * TICKS_PER_INCH)));
        rightFront.setTargetPosition((int)(rightFront.getCurrentPosition() + (distance * TICKS_PER_INCH)));
        rightBack.setTargetPosition((int)(rightBack.getCurrentPosition() + (distance * TICKS_PER_INCH)));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        //wait
        while(leftFront.isBusy() &&
                leftBack.isBusy() &&
                rightFront.isBusy() &&
                rightBack.isBusy()) {}

    }

    public void driveLeft(double distance, double power) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition((int)((-distance * TICKS_PER_INCH)));
        leftBack.setTargetPosition((int)((distance * TICKS_PER_INCH)));
        rightFront.setTargetPosition((int)((distance * TICKS_PER_INCH)));
        rightBack.setTargetPosition((int)((-distance * TICKS_PER_INCH)));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        //wait
        while(leftBack.isBusy()) {}

    }

    public void driveRight(double distance, double power) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition((int)(distance * TICKS_PER_INCH));
        leftBack.setTargetPosition((int)(-distance * TICKS_PER_INCH));
        rightFront.setTargetPosition((int)(-distance * TICKS_PER_INCH));
        rightBack.setTargetPosition((int)(distance * TICKS_PER_INCH));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        //wait
        while(leftBack.isBusy()) {}

    }

    public void driveBackward(double distance, double power) {

        leftFront.setTargetPosition((int)(-distance * TICKS_PER_INCH));
        leftBack.setTargetPosition((int)(-distance * TICKS_PER_INCH));
        rightFront.setTargetPosition((int)(-distance * TICKS_PER_INCH));
        rightBack.setTargetPosition((int)(-distance * TICKS_PER_INCH));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        //wait
        while(leftBack.isBusy()) {}

    }

    public void turn(double turnAngle, double power, WAGSAdafruitIMU imu) {
        double newHeading;
        double[] angles = imu.getAngles();
        double currYaw = angles[0];
        if (turnAngle > 0) { //positive param, right turn
            newHeading = currYaw + turnAngle;
            while (currYaw < newHeading) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);

            }
        } else { //negative param, left turn
            newHeading = currYaw - turnAngle;
            while (currYaw < newHeading) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(power);
                rightBack.setPower(power);
            }
        }
        setPower(0);
    }

    public void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    public void setEncoder(boolean encoder) {
        if(encoder != usingEncoders) /*desired setting is not the current setting*/ {
            if(encoder) {
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
}
