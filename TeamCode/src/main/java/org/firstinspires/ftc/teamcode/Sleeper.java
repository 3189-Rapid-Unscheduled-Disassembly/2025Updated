package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

class Sleeper {
    ElapsedTime timer = new ElapsedTime();

    class Sleep implements Action {
        boolean initialized = false;
        double sleepTimeMs;

        public Sleep(double sleepTimeMs) {
            this.sleepTimeMs = sleepTimeMs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }


            return !(timer.milliseconds() >= sleepTimeMs);
        }
    }

    public Action sleep(double sleepTimeMs) {
        return new Sleep(sleepTimeMs);
    }
}