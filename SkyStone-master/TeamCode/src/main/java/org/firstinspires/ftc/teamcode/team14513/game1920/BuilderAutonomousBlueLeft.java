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
package org.firstinspires.ftc.teamcode.team14513.game1920;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BuilderAutonomousBlueLeft", group="game1920")
public class BuilderAutonomousBlueLeft extends OpMode {
    private double BLUE_RED = 1.0;

    DriveTrainTeleOp driveTrainTeleOp;
    ArmHandTeleOp armHandTeleOp;
    private int loopCount = 0;

    Servo lockerServo;
    Servo markerServo;

    public BuilderAutonomousBlueLeft() {
        super();
    }

    public BuilderAutonomousBlueLeft(OpMode parent) {
        super();
        super.gamepad1 = parent.gamepad1;
        super.gamepad2 = parent.gamepad2;
        super.hardwareMap = parent.hardwareMap;
        super.internalOpModeServices = parent.internalOpModeServices;
        super.msStuckDetectInit = parent.msStuckDetectInit;
        super.msStuckDetectInitLoop = parent.msStuckDetectInitLoop;
        super.msStuckDetectLoop = parent.msStuckDetectLoop;
        super.msStuckDetectStart = parent.msStuckDetectStart;
        super.msStuckDetectStop = parent.msStuckDetectStop;
        super.telemetry = parent.telemetry;
        super.time = parent.time;
        BLUE_RED = -1.0;
    }

    @Override
    public void init() {
        driveTrainTeleOp = new DriveTrainTeleOp(this);
        driveTrainTeleOp.init();

        armHandTeleOp = new ArmHandTeleOp(this);
        armHandTeleOp.init();
    }

    @Override
    public void loop() {
        switch (loopCount) {
            case 0: // flip out arm
                armHandTeleOp.inOut(-1.0, 700);
                break;
            case 10: // move left(red) or right(blue) to center line of foundation
                driveTrainTeleOp.moveLiner(0.5 * BLUE_RED, 0.0, 600);
                break;
            case 12: // stop
                driveTrainTeleOp.moveLiner(0.0, 0.0, 300);
                break;
            case 14: // move backward to foundation
                driveTrainTeleOp.moveLiner(0.0, -1.0, 500);
                break;
            case 16:
                driveTrainTeleOp.moveLiner(0.0, -0.3, 500);
                break;
            case 20: // lock foundation
                armHandTeleOp.lock();
                break;
            case 30: // stop
                driveTrainTeleOp.moveLiner(0.0, 0.0, 1000);
                break;
            case 32: // move forward and drag foundation to touch the corner
                driveTrainTeleOp.moveLiner(0.0, 0.6, 1000);
                break;
            case 34:
                driveTrainTeleOp.moveLiner(0.0, 0.5, 600);
                break;
            case 40: // unlock
                armHandTeleOp.unlock();
                break;
            case 50: // stop
            case 51:
            case 52:
                driveTrainTeleOp.moveLiner(0.0, 0.0, 3000);
                break;

            case 53: // move left(blue) or right(red) to park
                driveTrainTeleOp.moveLiner(-0.8 * BLUE_RED, 0.0, 1000);
                break;
            case 54: // back out space for turnning
                driveTrainTeleOp.moveLiner(0.0, -0.5, 150);
                break;
            case 56: // turn left(blue) or right(red) to drop capping stone and lower arm
                driveTrainTeleOp.turn(-0.7 * BLUE_RED, 1100);
                break;
            case 60: // flip out arm
                armHandTeleOp.inOut(-1.0, 800);
                break;
            case 62:
                armHandTeleOp.inOut(-0.5, 500);
                break;
            case 64: // drop capping stone
                armHandTeleOp.openClose(1.0, 1000);
                break;

            case 70: // park under gate
                driveTrainTeleOp.moveLiner(0.0, -0.5, 400);
                driveTrainTeleOp.moveLiner(0.7 * BLUE_RED, 0.0, 400);
                break;
            default:
                break;
        }
        loopCount++;
    }
}
