/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Color Yellow Test Yay", group="Linear Opmode")
@Disabled
public class ColorYellowTest extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double minPosition = 0;
    double minDistance =100;
    double currentDistance= 100;
    boolean rampUp = true;
    DistanceSensor colorBoi;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo turnTable = null;
    private ColorSensor sensor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        servo = hardwareMap.get(Servo.class, "ServoScan_UrMother");

        /* 1.) get color sensor inputs
        2.) get the robot to swivel camera from one angle to another
        3.) how measure each color while swivel?
        4.) for loop: angles and distances
        5.) for loop: figure out which angle has smallest distance from all gathered data in step 4

         */


        colorBoi = hardwareMap.get(DistanceSensor.class, "sensor_color");


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentDistance=colorBoi.getDistance(DistanceUnit.CM);
            //moves position a smidge
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if (minDistance< currentDistance) {
                minPosition=position;
                minDistance=currentDistance;
            }

            telemetry.addData("Distance (cm)", "%.3f", currentDistance);
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData("Minimum Distance (cm)", "%.3f", minDistance);
            telemetry.addData("Minimum Servo Position", "%5.2f", minPosition);
            telemetry.addData(">", "Press Stop to end test.");

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            telemetry.update();
            }
        }

    }
