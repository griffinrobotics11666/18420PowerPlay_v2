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

    //

    Hardwarerobot robot = new Hardwarerobot();
    double slowfactor = 0.5;
    static final double ARM_POWER_LIMIT = .5;
    static double CLAW_OPENED_POSITION = .90; // flip closed and open
    static double CLAW_CLOSED_POSITION = 1;
    static double ARM_COUNTS_PER_INCH = 80; //Figure out right number //114.75
    int newTarget = 0;

    //for LexLifeHack
//    static final double P_DRIVE_COEFF           = 0.05;
//    static final double COUNTS_PER_MOTOR_REV = 1440;
//    static final double DRIVE_GEAR_REDUCTION = 1.0;
//    static final double WHEEL_DIAMETER_INCHES = 4.5;
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415) / 2.29; //find out actual number
//    static final double STRAFE_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415) / 1.86;
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
    //end of LexLifeHack


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready");
        robot.armExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtendor.setTargetPosition(newTarget);
        robot.armExtendor.setPower(0);
        //LexLifeHack
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
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
        // Setup a variable for each drive wheel to save power level for telemetry
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
        //LexLifeHack
        double speed;
        double angle;


        if (gamepad2.a) {
            robot.claw.setPosition(CLAW_CLOSED_POSITION);
        }
        if (gamepad2.b) {
            robot.claw.setPosition(CLAW_OPENED_POSITION);
        }



/*
        if (gamepad1.dpad_up){
            CLAW_OPENED_POSITION = CLAW_OPENED_POSITION + addToClaw;
        }
        if (gamepad1.dpad_down){
            CLAW_OPENED_POSITION = CLAW_OPENED_POSITION - addToClaw;
        }
        if (gamepad1.dpad_left){
            CLAW_CLOSED_POSITION = CLAW_CLOSED_POSITION + addToClaw;
        }
        if (gamepad1.dpad_right){
            CLAW_CLOSED_POSITION = CLAW_CLOSED_POSITION - addToClaw;
        }
*/


        if (-gamepad2.left_stick_y > .1) { //me when go up
            newTarget = (int) (newTarget + 5 * -gamepad2.left_stick_y);
            if (newTarget / ARM_COUNTS_PER_INCH > 35.5) {
                newTarget = (int) (35.5 * ARM_COUNTS_PER_INCH);
            }
            robot.armExtendor.setTargetPosition(newTarget);
        }
        if (gamepad2.left_stick_y > .1) {  //go down hehe
            newTarget = (int) (newTarget - 5 * gamepad2.left_stick_y);
            /*
            if (newTarget/ARM_COUNTS_PER_INCH < 1) {
                newTarget = (int)(1 * ARM_COUNTS_PER_INCH)
            } */

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


        //LexLifeHack:
//        if (gamepad1.dpad_up) {
//            gyroDrive(.8, -5, 90);
//            gyroTurn(.4, 180);
//            gyroDrive(.8,14,90);
//        }
//        if (gamepad1.dpad_down) {
//            gyroDrive(.8,-14,90);
//            gyroTurn(.4, 180);
//            //gyroDrive(.8, 6, 90);
//        }



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


//added for LexLifeHack
//
//    public void gyroDrive(double speed,
//                          double distance,
//                          double angle) {
//
//        int newLeftFrontTarget;
//        int newLeftBackTarget;
//        int newRightFrontTarget;
//        int newRightBackTarget;
//        int moveCounts;
//        double max;
//        double error;
//        double steer;
//        double leftSpeed;
//        double rightSpeed;
//
//        if (true) {
//
//            moveCounts = (int) (distance * COUNTS_PER_INCH);
//            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
//            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
//            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
//            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;
//            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
//            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
//            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
//            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
//
//            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.leftFrontDrive.setPower(speed);
//            robot.leftBackDrive.setPower(speed);
//            robot.rightFrontDrive.setPower(speed);
//            robot.rightBackDrive.setPower(speed);
//
//            while (robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy()) {
//
//                leftSpeed = speed;
//                rightSpeed = speed;
//
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.leftFrontDrive.setPower(leftSpeed);
//                robot.leftBackDrive.setPower(leftSpeed);
//                robot.rightFrontDrive.setPower(rightSpeed);
//                robot.rightBackDrive.setPower(rightSpeed);
//
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.update();
//            }
//            robot.leftFrontDrive.setPower(0);
//            robot.leftBackDrive.setPower(0);
//            robot.rightFrontDrive.setPower(0);
//            robot.rightBackDrive.setPower(0);
//            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    //lexlifehack
//    public void gyroTurn (  double speed, double angle) {
//
//        // keep looping while we are still active, and not on heading.
//        while (!onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        robot.leftFrontDrive.setPower(leftSpeed);
//        robot.leftBackDrive.setPower(leftSpeed);
//        robot.rightFrontDrive.setPower(rightSpeed);
//        robot.rightBackDrive.setPower(rightSpeed);
//
//
//        return onTarget;
//    }
//
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        gravity  = imu.getGravity();
//        robotError = targetAngle - angles.firstAngle;
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
//    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
//    static final double     HEADING_THRESHOLD       = 1 ;
//    //lexlifehack:out
}

