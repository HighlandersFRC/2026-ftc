package org.firstinspires.ftc.teamcode;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

/** FTC/Android trajectory reader. SI units, lower-left field origin, CCW heading. */
public final class MecanumTrajectory {
    public static final class State {
        public final double t, x, y, heading, vx, vy, omega;
        public State(double t, double x, double y, double heading,
                     double vx, double vy, double omega) {
            this.t=t; this.x=x; this.y=y; this.heading=heading;
            this.vx=vx; this.vy=vy; this.omega=omega;
        }
        /** Linear wheel speeds in m/s, FL, BL, BR, FR; X-pattern rollers.
         * halfLength/halfWidth are axle offsets from robot center in meters.
         * Motor polarity and encoder ticks/sec conversion belong to your robot code.
         */
        public double[] wheelSpeeds(double robotHeading, double halfLength, double halfWidth) {
            if (!(halfLength > 0) || !(halfWidth > 0)
                    || !Double.isFinite(halfLength) || !Double.isFinite(halfWidth)
                    || !Double.isFinite(robotHeading)) throw new IllegalArgumentException("Invalid robot geometry/heading");
            double forward = Math.cos(robotHeading)*vx + Math.sin(robotHeading)*vy;
            double left = -Math.sin(robotHeading)*vx + Math.cos(robotHeading)*vy;
            double turn = (halfLength+halfWidth)*omega;
            return new double[]{forward-left-turn, forward+left-turn,
                                forward-left+turn, forward+left+turn};
        }
    }
    private final State[] states;
    private final double halfLength;
    private final double halfWidth;
    private final double maxWheelSpeedMetersPerSecond;

    private MecanumTrajectory(State[] states, double halfLength, double halfWidth,
                              double maxWheelSpeedMetersPerSecond) {
        this.states = states;
        this.halfLength = halfLength;
        this.halfWidth = halfWidth;
        this.maxWheelSpeedMetersPerSecond = maxWheelSpeedMetersPerSecond;
    }

    public double duration() { return states[states.length-1].t; }

    public double[] wheelSpeeds(State state) {
        return state.wheelSpeeds(state.heading, halfLength, halfWidth);
    }

    public double maxWheelSpeedMetersPerSecond() {
        return maxWheelSpeedMetersPerSecond;
    }

    public static MecanumTrajectory load(InputStream stream) throws IOException, JSONException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        byte[] buffer = new byte[4096];
        int count;
        while ((count=stream.read(buffer)) != -1) bytes.write(buffer,0,count);
        JSONObject trajectory = new JSONObject(new String(bytes.toByteArray(), StandardCharsets.UTF_8))
                .getJSONObject("trajectory");
        if (!"Mecanum".equals(trajectory.getString("sampleType")))
            throw new JSONException("Expected an FTC Mecanum trajectory");

        JSONObject config = trajectory.getJSONObject("config");
        JSONObject frontLeft = config.getJSONObject("frontLeft");
        double halfLength = Math.abs(frontLeft.getDouble("x"));
        double halfWidth = Math.abs(frontLeft.getDouble("y"));
        double wheelRadius = config.getDouble("radius");
        double gearing = config.getDouble("gearing");
        double maxAngularSpeed = config.getDouble("vmax") / gearing;
        double maxWheelSpeed = wheelRadius * maxAngularSpeed;
        if (!(halfLength > 0) || !(halfWidth > 0) || !(maxWheelSpeed > 0)
                || !Double.isFinite(halfLength) || !Double.isFinite(halfWidth)
                || !Double.isFinite(maxWheelSpeed)) {
            throw new JSONException("Invalid mecanum wheel geometry or speed config");
        }

        JSONArray samples = trajectory.getJSONArray("samples");
        if (samples.length() < 2) throw new JSONException("Generate the path before exporting");
        State[] states = new State[samples.length()];
        for (int i=0; i<states.length; i++) {
            JSONObject s=samples.getJSONObject(i);
            double[] values = new double[7];
            String[] keys = {"t","x","y","heading","vx","vy","omega"};
            for (int k=0; k<keys.length; k++) {
                values[k]=s.getDouble(keys[k]);
                if (!Double.isFinite(values[k])) throw new JSONException("Non-finite sample");
            }
            states[i]=new State(values[0],values[1],values[2],values[3],values[4],values[5],values[6]);
            if (values[0]<0 || (i>0 && values[0]<=states[i-1].t))
                throw new JSONException("Sample times must increase");
        }
        return new MecanumTrajectory(states, halfLength, halfWidth, maxWheelSpeed);
    }
    /** Clamp to path bounds and interpolate; heading follows the shortest arc. */
    public State sample(double time) {
        if (!Double.isFinite(time)) throw new IllegalArgumentException("Non-finite time");
        if (time<=states[0].t) return states[0];
        if (time>=duration()) return states[states.length-1];
        int lo=0, hi=states.length-1;
        while (hi-lo>1) { int mid=(lo+hi)/2; if (states[mid].t<=time) lo=mid; else hi=mid; }
        State a=states[lo], b=states[hi];
        double f=(time-a.t)/(b.t-a.t);
        double dh=Math.atan2(Math.sin(b.heading-a.heading), Math.cos(b.heading-a.heading));
        return new State(time, mix(a.x,b.x,f), mix(a.y,b.y,f), a.heading+f*dh,
                mix(a.vx,b.vx,f),mix(a.vy,b.vy,f),mix(a.omega,b.omega,f));
    }
    private static double mix(double a,double b,double f) { return a+(b-a)*f; }
}
