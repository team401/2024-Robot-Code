package frc.robot.utils;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Iterator;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class ControllerJSONReader {

    private static HashMap<Integer, CommandGenericHID> controllers;

    private static HashMap<String, Trigger> triggers;
    private static HashMap<String, DoubleSupplier> axes;
    private static HashMap<String, IntSupplier> pov;

    // gets json objects from chosen json file
    public static void pullConfiguration(String configuration) {
        JSONParser jsonParser = new JSONParser();
        JSONObject jsonObject;
        try {
            jsonObject =
                    (JSONObject)
                            jsonParser.parse(
                                    (new FileReader(
                                            new File(
                                                    Filesystem.getDeployDirectory(),
                                                    "controllerconfig/"
                                                            + configuration
                                                            + ".json"))));

        } catch (Exception e) {
            throw new RuntimeException("Controller not found, please try again");
        }
        setControllers((JSONArray) jsonObject.get("controllers"));
        setTriggers((JSONArray) jsonObject.get("buttons"));
        setAxes((JSONArray) jsonObject.get("axes"));
        setPOV((JSONArray) jsonObject.get("pov"));
    }

    // initializes controllers based on json file data
    private static HashMap<Integer, CommandGenericHID> setControllers(JSONArray controllersJSON) {
        HashMap<Integer, CommandGenericHID> controllersList =
                new HashMap<Integer, CommandGenericHID>();
        Iterator<JSONObject> iterator = controllersJSON.iterator();
        while (iterator.hasNext()) {
            // initializes each controller
            JSONObject controller = iterator.next();
            // joysticks
            if (((String) controller.get("type")).equals("joystick")) {
                controllersList.put(
                        ((Long) controller.get("port")).intValue(),
                        new CommandJoystick(((Long) controller.get("port")).intValue()));
            } else {
                // xbox controller
                controllersList.put(
                        ((Long) controller.get("port")).intValue(),
                        new CommandXboxController(((Long) controller.get("port")).intValue()));
            }
        }
        controllers = controllersList;
        return controllersList;
    }

    // assigns triggers to string keys
    private static HashMap<String, Trigger> setTriggers(JSONArray triggersJSON) {
        HashMap<String, Trigger> triggerList = new HashMap<String, Trigger>();
        Iterator<JSONObject> iterator = triggersJSON.iterator();
        while (iterator.hasNext()) {
            JSONObject trigger = iterator.next();
            Trigger t;

            int port = ((Long) trigger.get("controller")).intValue();

            // first case is for xbox controllers, second case is for joysticks
            switch (((String) trigger.get("button"))) {
                case "a":
                case "1":
                    t = controllers.get(port).button(1);
                    break;
                case "b":
                case "2":
                    t = controllers.get(port).button(2);
                    break;
                case "x":
                case "3":
                    t = controllers.get(port).button(3);
                    break;
                case "y":
                case "4":
                    t = controllers.get(port).button(4);
                    break;
                case "leftBumper":
                case "5":
                    t = controllers.get(port).button(5);
                    break;
                case "rightBumper":
                case "6":
                    t = controllers.get(port).button(6);
                    break;
                case "back":
                case "7":
                    t = controllers.get(port).button(7);
                    break;
                case "start":
                case "8":
                    t = controllers.get(port).button(8);
                    break;
                case "leftJoystickPress":
                case "9":
                    t = controllers.get(port).button(9);
                    break;
                case "rightJoystickPress":
                case "10":
                    t = controllers.get(port).button(10);
                    break;
                case "pov0":
                    t = controllers.get(port).pov(0);
                    break;
                case "pov45":
                    t = controllers.get(port).pov(45);
                    break;
                case "pov90":
                    t = controllers.get(port).pov(90);
                    break;
                case "pov135":
                    t = controllers.get(port).pov(135);
                    break;
                case "pov180":
                    t = controllers.get(port).pov(180);
                    break;
                case "pov225":
                    t = controllers.get(port).pov(225);
                    break;
                case "pov270":
                    t = controllers.get(port).pov(270);
                    break;
                case "pov315":
                    t = controllers.get(port).pov(315);
                    break;
                case "povCenter":
                    t = controllers.get(port).pov(-1);
                    break;
                default:
                    t = null;
                    break;
            }

            triggerList.put((String) trigger.get("command"), t);
        }
        triggers = triggerList;
        return triggerList;
    }

    // assigns axis doublesuppliers to string keys
    private static HashMap<String, DoubleSupplier> setAxes(JSONArray axisJSON) {
        HashMap<String, DoubleSupplier> axisList = new HashMap<String, DoubleSupplier>();
        Iterator<JSONObject> iterator = axisJSON.iterator();
        while (iterator.hasNext()) {
            JSONObject axis = iterator.next();
            DoubleSupplier t;

            int port = ((Long) axis.get("controller")).intValue();

            // first case is for xbox controller, second case is for joysticks
            switch (((String) axis.get("axis"))) {
                case "leftX":
                case "xAxis":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(0);
                    else t = () -> controllers.get(port).getRawAxis(0);
                    SmartDashboard.putNumber("readaxis", t.getAsDouble());
                    break;
                case "leftY":
                case "yAxis":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(1);
                    else t = () -> controllers.get(port).getRawAxis(1);
                    break;
                case "leftTrigger":
                case "zRotate":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(2);
                    else t = () -> controllers.get(port).getRawAxis(2);
                    break;
                case "rightTrigger":
                case "slider":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(3);
                    else t = () -> controllers.get(port).getRawAxis(3);
                    break;
                case "rightX":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(4);
                    else t = () -> controllers.get(port).getRawAxis(4);
                    break;
                case "rightY":
                    if ((boolean) axis.get("negate"))
                        t = () -> -controllers.get(port).getRawAxis(5);
                    else t = () -> controllers.get(port).getRawAxis(5);
                    break;
                default:
                    t = null;
                    break;
            }

            axisList.put((String) axis.get("command"), t);
        }
        axes = axisList;
        return axisList;
    }

    // assigns pov intsuppliers to string keys
    private static HashMap<String, IntSupplier> setPOV(JSONArray povJSON) {
        HashMap<String, IntSupplier> povList = new HashMap<String, IntSupplier>();
        Iterator<JSONObject> iterator = povJSON.iterator();
        while (iterator.hasNext()) {
            JSONObject p = iterator.next();

            if (controllers.get(p.get("controller")) == null) {
                povList.put((String) p.get("command"), () -> 0);
            } else
                povList.put(
                        (String) p.get("command"),
                        () -> controllers.get(p.get("controller")).getHID().getPOV());
        }
        pov = povList;
        return povList;
    }

    // methods to get controllers, triggers, axes, povs for robotcontainer use

    public static HashMap<Integer, CommandGenericHID> getControllers() {
        if (controllers == null) {
            throw new RuntimeException("Controllers not yet generated, run pullConfiguration");
        } else {
            return controllers;
        }
    }

    public static HashMap<String, Trigger> getTriggers() {
        if (triggers == null) {
            throw new RuntimeException("Triggers not yet generated, run pullConfiguration");
        } else {
            return triggers;
        }
    }

    public static HashMap<String, DoubleSupplier> getAxes() {
        if (axes == null) {
            throw new RuntimeException("Axes not yet generated, run pullConfiguration");
        } else {
            return axes;
        }
    }

    public static HashMap<String, IntSupplier> getPOVs() {
        if (pov == null) {
            throw new RuntimeException("POVs not yet generated, run pullConfiguration");
        } else {
            return pov;
        }
    }
}
