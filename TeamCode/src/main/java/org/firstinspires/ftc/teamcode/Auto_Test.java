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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "draft: Auto Test", group = "draft")
@Disabled

public class Auto_Test extends LinearOpMode {
    // Declare DcMotor members
    private DcMotor Motor_Left = null;      // DcMotors for left wheels
    private DcMotor Motor_Right = null;     // DcMotors for right wheels
    private DcMotor Motor_Convey = null;    // DcMotors to pick up boxes
    private DcMotor Motor_Elevator = null;  // DcMotor to elevate boxes

    int rpc=1440;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Call hardware
        Motor_Left = hardwareMap.get(DcMotor.class, "Motor_L");
        Motor_Right = hardwareMap.get(DcMotor.class, "Motor_R");
        Motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set directions of DcMotors
        Motor_Left.setDirection(DcMotor.Direction.REVERSE);
        Motor_Right.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Run_encoder(5.000, 5.000,0.25);
                // Run_encoder(-1.615, +1.615,0.25);
            } else if (gamepad1.b) {
                Run_encoder(4.500, 5.000,0.25);
                // Run_encoder(-1.615, +1.615,0.25);
            } else if (gamepad1.x) {
                Run_encoder(0.93, 0.93,0.25);
                // Run_encoder(-1.615, +1.615,0.25);
            } else if (gamepad1.y) {
                Run_encoder(1.48, 1.48,0.25);
                // Run_encoder(-1.615, +1.615,0.25);
            } else if (gamepad1.left_bumper) {
                Run_encoder(-1.500, +1.500,0.25);
            } else if (gamepad1.right_bumper) {
                Run_encoder(+1.600, -1.600,0.25);
            }
        }
    }

    public void Run_encoder(double leftspin, double rightspin, double run_power
    ) {
        int left_position;
        int right_position;
        // setup the reset
        Motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // setup the position for the location
        Motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_position =  Motor_Left.getCurrentPosition() + (int) (leftspin * rpc);
        right_position = Motor_Right.getCurrentPosition() + (int) (rightspin * rpc);

        Motor_Left.setTargetPosition(left_position);
        Motor_Right.setTargetPosition(right_position);

        // run to position
        Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_Left.setPower(run_power*0.97);
        Motor_Right.setPower(run_power);
        while(Motor_Left.isBusy() && Motor_Right.isBusy() ){

        }
        Motor_Left.setPower(0);
        Motor_Right.setPower(0);

    }
}
