package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


// import com.acmerobotics.dashboard.FtcDashboard;


@TeleOp(name="Driver control 2024", group="TeleOp")
public class DriverControl2024 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime(); //clock

    Hardwarerobot robot = new Hardwarerobot();
    double slowfactor = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double speed;
        double angle;

        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

        if (gamepad1.right_trigger > 0) {
            leftFrontPower = leftFrontPower * slowfactor;
            leftBackPower = leftBackPower * slowfactor;
            rightFrontPower = rightFrontPower * slowfactor;
            rightBackPower = rightBackPower * slowfactor;
        }

        //gotta go fast haha wait just kidding

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.rightBackDrive.setPower(rightBackPower);
        telemetry.addData("Status", "HELLO!");
        telemetry.addData("Status", "DON'T GET ANY PENALTIES OR ELSE...");
        telemetry.addData("Status","I'M GOING TO LEAVE THE END OF THIS SENTENCE UP FOR INTERPRETATION");
        telemetry.addData("Status", "FEEL FREE TO BE CREATIVE");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
    //Slay!!!!!!!!!!

}