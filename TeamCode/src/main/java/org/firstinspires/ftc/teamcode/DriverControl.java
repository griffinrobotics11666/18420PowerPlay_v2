package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


 // import com.acmerobotics.dashboard.FtcDashboard;


@TeleOp(name="Driver control catz are cool", group="TeleOp")
public class DriverControl extends OpMode {
    private ElapsedTime runtime = new ElapsedTime(); //clock

    Hardwarerobot robot = new Hardwarerobot();
    double slowfactor = 0.5;
    static final double ARM_POWER_LIMIT = .5;
    static double CLAW_OPENED_POSITION = .90; // flip closed and open
    static double CLAW_CLOSED_POSITION = 1;
    static double ARM_COUNTS_PER_INCH = 80; //Figure out right number //114.75
    int newTarget = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready");
        robot.armExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtendor.setTargetPosition(newTarget);
        robot.armExtendor.setPower(0);
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
        double armExtendorPower;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double extend = -gamepad1.right_stick_y;
        double strafe = gamepad1.left_stick_x;
        double addToClaw = .0001;
        double speed;
        double angle;

        if (gamepad2.a) {
            robot.claw.setPosition(CLAW_CLOSED_POSITION);
        }
        if (gamepad2.b) {
            robot.claw.setPosition(CLAW_OPENED_POSITION);
        }

        if (-gamepad2.left_stick_y > .1) { //me when go up
            newTarget = (int) (newTarget + 5 * -gamepad2.left_stick_y);
            if (newTarget / ARM_COUNTS_PER_INCH > 35.5) {
                newTarget = (int) (35.5 * ARM_COUNTS_PER_INCH);
            }
            robot.armExtendor.setTargetPosition(newTarget);
        }
        if (gamepad2.left_stick_y > .1) {  //go down hehe
            newTarget = (int) (newTarget - 5 * gamepad2.left_stick_y);

            robot.armExtendor.setTargetPosition(newTarget);
        }
        if (gamepad2.x) {
            robot.armExtendor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtendor.setTargetPosition(0);
            robot.armExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.dpad_down) {
            goTo0();
        } else if (gamepad2.dpad_left) {
            goTo1();
        } else if (gamepad2.dpad_right) {
            goTo2();
        } else if (gamepad2.dpad_up) {
            goTo3();
        }

        //haha look see? this is so beautiful. you smell. <3


        //armExtendorPower = Range.scale(extend,-1 ,1,-ARM_POWER_LIMIT,ARM_POWER_LIMIT);
        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (gamepad1.right_trigger > 0) {
            leftFrontPower = leftFrontPower * slowfactor;
            leftBackPower = leftBackPower * slowfactor;
            rightFrontPower = rightFrontPower * slowfactor;
            rightBackPower = rightBackPower * slowfactor;
        }

        //gotta go fast haha wait just kidding
        //SLAY!!!!!!!

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.rightBackDrive.setPower(rightBackPower);
        //robot.armExtendor.setPower(armExtendorPower);
        robot.armExtendor.setPower(Math.abs(1));
        telemetry.addData("Status", "HELLO!");
        telemetry.addData("Status", "DON'T GET ANY PENALTIES OR ELSE...");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("claw", "open (%.2f), closed (%.2f)", CLAW_OPENED_POSITION, CLAW_CLOSED_POSITION);
        telemetry.addData("arm", "arm_status (%.2f)", robot.armExtendor.getCurrentPosition() / ARM_COUNTS_PER_INCH);
        telemetry.addData("servo position", robot.claw.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    public void goTo0() {
        double distance = 0;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }

    public void goTo1() {
        double distance = 20;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }

    public void goTo2() {
        double distance = 32;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }

    public void goTo3() {
        double distance = 45;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }

    public void goTo4() {
        double distance = 34;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }
}