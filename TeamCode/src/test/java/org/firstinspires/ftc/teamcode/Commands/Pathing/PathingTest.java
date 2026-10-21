package org.firstinspires.ftc.teamcode.Commands.Pathing;

import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Test;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import static org.junit.Assert.*;

public class PathingTest {
    private String export() throws Exception {
        return new String(Files.readAllBytes(Paths.get(
                "src/main/java/org/firstinspires/ftc/teamcode/Deploy/finally.json")), StandardCharsets.UTF_8);
    }
    private double[] control(TimedPath.Sample target, double x, double y, double h) {
        return MecanumPathController.calculate(target, x, y, h, .3, 2, 3, 1, 1, 2, .4);
    }
    @Test public void loadsActualExportAndInterpolates() throws Exception {
        TimedPath path = TimedPath.load(new StringReader(export()));
        assertEquals(2.8515785289223614, path.duration(), 1e-9);
        assertEquals((path.sample(.05).x + path.sample(.1).x) / 2, path.sample(.075).x, 1e-9);
        assertEquals(path.last().x, path.sample(100).x, 0);
        assertEquals(0, path.sample(100).vx, 0);
    }
    @Test public void respectsMecanumFramesAndTurnSign() {
        TimedPath.Sample target = new TimedPath.Sample(0, .1, 0, 0, 0, 0, 0);
        double[] facingX = control(target, 0, 0, 0);
        assertEquals(0, facingX[0], 1e-9);
        assertTrue(facingX[1] > 0);
        target = new TimedPath.Sample(0, .1, 0, Math.PI / 2, 0, 0, 0);
        double[] facingY = control(target, 0, 0, Math.PI / 2);
        assertTrue(facingY[0] > 0);
        assertEquals(0, facingY[1], 1e-9);
        assertTrue(control(new TimedPath.Sample(0, 0, 0, .1, 0, 0, 0), 0, 0, 0)[2] < 0);
    }
    @Test public void wrapsHeadingAcrossPi() {
        double[] p = control(new TimedPath.Sample(0, 0, 0, Math.toRadians(-179), 0, 0, 0),
                0, 0, Math.toRadians(179));
        assertEquals(-3 * Math.toRadians(2) / 2, p[2], 1e-9);
    }
    @Test public void limitsEveryWheel() {
        for (double h = -Math.PI; h <= Math.PI; h += .1) {
            double[] p = control(new TimedPath.Sample(0, 10, -10, 1, 2, 3, 4), 0, 0, h);
            double x = p[0], y = p[1], r = p[2];
            for (double wheel : new double[] {y+x+r, y-x-r, y-x+r, y+x-r})
                assertTrue(Math.abs(wheel) <= .400000001);
        }
    }
    @Test public void rejectsWrongUnitsAndUnorderedTime() throws Exception {
        JSONObject root = new JSONObject(export());
        root.getJSONObject("meta_data").put("distance_unit", "inches");
        assertThrows(JSONException.class, () -> TimedPath.load(new StringReader(root.toString())));
        root.getJSONObject("meta_data").put("distance_unit", "meters");
        root.getJSONArray("sampled_points").getJSONObject(1).put("time", 0);
        assertThrows(JSONException.class, () -> TimedPath.load(new StringReader(root.toString())));
    }
    @Test public void interpolatesHeadingOnShortestArc() throws Exception {
        JSONObject root = new JSONObject(export());
        root.getJSONArray("sampled_points").getJSONObject(0).put("angle", Math.toRadians(179));
        root.getJSONArray("sampled_points").getJSONObject(1).put("angle", Math.toRadians(-179));
        TimedPath path = TimedPath.load(new StringReader(root.toString()));
        assertEquals(Math.PI, Math.abs(path.sample(.025).heading), 1e-9);
    }
    @Test public void followsExportInIdealKinematicSimulation() throws Exception {
        TimedPath path = TimedPath.load(new StringReader(export()));
        double x = path.first().x - .08, y = path.first().y + .06, h = .12;
        for (double t = 0; t < path.duration() / .3 + 3; t += .01) {
            double[] p = control(path.sample(t * .3), x, y, h);
            // Independent robot-right/forward -> field transform; nominal speed 1 m/s, 2 rad/s.
            x += (p[1] * Math.cos(h) + p[0] * Math.sin(h)) * .01;
            y += (p[1] * Math.sin(h) - p[0] * Math.cos(h)) * .01;
            h -= 2 * p[2] * .01;
        }
        assertTrue(Math.hypot(x - path.last().x, y - path.last().y) < .05);
        assertTrue(Math.abs(TimedPath.wrap(h - path.last().heading)) < Math.toRadians(3));
    }
}
