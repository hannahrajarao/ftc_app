package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Hannah Rajarao on 12/26/2017.
 */

public class JewelTest extends LinearOpMode {
    HardwareScuffle robot = new HardwareScuffle();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("color", robot.jewelColor());
        telemetry.update();
    }
}
