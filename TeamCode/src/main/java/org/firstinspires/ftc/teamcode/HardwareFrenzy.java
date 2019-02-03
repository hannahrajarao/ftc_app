package org.firstinspires.ftc.teamcode.FrozenFrenzy;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.WAGSAdafruitIMU;
import org.firstinspires.ftc.teamcode.WAGSVuMarkIdentifier;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS, on 1/14/2018.
 */

public class HardwareFrenzy {

    // Public OpMode members
    public MecanumDrive driveTrain;

    public Servo clampLeft;
    public Servo clampRight;
    public DcMotor pulley;

    public Servo jewel;

    public DcMotor relicArm;
    public Servo relicRotate;
    public Servo relicClaw;

    public ColorSensor colorSensor;
    public WAGSAdafruitIMU imu;
    public WAGSVuMarkIdentifier vision;

    //Local OpMode members
    HardwareMap hwMap = null;

    double JEWEL_UP = 0.5;
    double JEWEL_DOWN = 1.0;

    public void init(HardwareMap aHwMap) {
        init(aHwMap, true);
    }

    public void init(HardwareMap aHwMap, boolean auton) {
        hwMap = aHwMap;

        driveTrain = new MecanumDrive();
        driveTrain.init(hwMap, auton);

        clampLeft = hwMap.servo.get("clampLeft");
        clampRight = hwMap.servo.get("clampRight");
        pulley = hwMap.dcMotor.get("pulley");

        jewel = hwMap.servo.get("jewel");

        relicClaw = hwMap.servo.get("relicClaw");
        relicRotate = hwMap.servo.get("relicRotate");
        relicArm = hwMap.dcMotor.get("relic");

        colorSensor = hwMap.colorSensor.get("color");
        imu = new WAGSAdafruitIMU("imu", hwMap);
        if(auton)
            vision = new WAGSVuMarkIdentifier(hwMap);
    }

    public void setPower(double power) {
        driveTrain.setPower(power);
    }

    public void driveForward(double distance, double power) {
        driveTrain.driveForward(distance, power);
    }
    public void driveBackward(double distance, double power) {
        driveTrain.driveBackward(distance, power);
    }
    public void driveLeft(double distance, double power) {
        driveTrain.driveLeft(distance, power);
    }

    public void driveRight(double distance, double power) {
        driveTrain.driveRight(distance, power);

    }

    public void clampIn() {
        clampLeft.setPosition(1);
        clampRight.setPosition(0);
    }

    public void clampOut() {
        clampLeft.setPosition(0.5);
        clampRight.setPosition(0.60);
    }

    public void rotateRelic() {
        double position = relicRotate.getPosition();
        if(position == 0)
            relicRotate.setPosition(0.5);
        else if(position == 0.5)
            relicRotate.setPosition(0);
    }

    public void relicClaw() {
        double position = relicClaw.getPosition();
        if(position == 0)
            relicClaw.setPosition(0.5);
        else if(position == 0.5)
            relicClaw.setPosition(0);
    }

    public void jewelDown() {
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp() {
        jewel.setPosition(JEWEL_UP);
    }

    public String jewelColor() {
        String color = "";
        if(colorSensor.red()<colorSensor.blue())
            color = "blue";
        if(colorSensor.red()>=colorSensor.blue())
            color = "red";
        return color;
    }
}
