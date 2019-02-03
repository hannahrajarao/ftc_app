package org.firstinspires.ftc.teamcode.PostSeason;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Hannah Rajarao, Team 6037 WAGS
 */

public class Hardware0518 {
    public MecanumDrive driveTrain;
    public DcMotor intakeLeft;
    public DcMotor intakeRight;
    public DcMotor flipper;
    public Servo holder;

    HardwareMap hwMap;

    public void init(HardwareMap aHwMap) {
        hwMap = aHwMap;

        driveTrain = new MecanumDrive();
        driveTrain.init(hwMap, false);

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");
        flipper = hwMap.dcMotor.get("flipper");
        holder = hwMap.servo.get("holder");

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
