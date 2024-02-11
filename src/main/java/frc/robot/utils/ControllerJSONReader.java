package frc.robot.utils;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.function.BiConsumer;

import javax.crypto.spec.PBEKeySpec;
import javax.management.RuntimeErrorException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilderException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerJSONReader {

    private static HashMap<String, Trigger> triggers;
    private static HashMap<Integer, CommandGenericHID> controllers;

    public static void pullConfiguration(String configuration) {
        JSONParser jsonParser = new JSONParser();
        JSONObject jsonObject;
        try {
            jsonObject = (JSONObject) jsonParser.parse((new FileReader(new File(
                        Filesystem.getDeployDirectory(), "controllerconfig/" + configuration + ".json"))));
            
        } catch (Exception e) {
            throw new RuntimeException(
                "Controller not found, please try again");
        }
        setControllers((JSONArray) jsonObject.get("controllers"));
        setTriggers((JSONArray) jsonObject.get("buttons"));
    }

    private static HashMap<Integer, CommandGenericHID> setControllers(JSONArray controllersJSON) {
        HashMap<Integer, CommandGenericHID> controllersList = new HashMap<Integer, CommandGenericHID>();
        Iterator<JSONObject> iterator = controllersJSON.iterator();
        while(iterator.hasNext()) {
            JSONObject controller = iterator.next();
            if (((String)controller.get("type")).equals("joysticks")) 
                controllersList.put((Integer) controller.get("port"), new CommandJoystick((int) controller.get("port")));
            else 
                controllersList.put((Integer) controller.get("port"), new CommandXboxController((int) controller.get("port")));
        }
        controllers = controllersList;
        return controllersList;
    }

    private static HashMap<String, Trigger> setTriggers(JSONArray triggersJSON) {
        HashMap<String, Trigger> triggerList = new HashMap<String, Trigger>();
        Iterator<JSONObject> iterator = triggersJSON.iterator();
        while(iterator.hasNext()) {
            JSONObject trigger = iterator.next();
            Trigger t;
            
            int port = (int) trigger.get("controller");
            switch (((String) trigger.get("button"))) {
                //gamepad buttons
                case "a":
                case "0":
                    t = controllers.get(port).button(0);
                    break;
                case "b":
                case "1":
                    t = controllers.get(port).button(1);
                    break;
                case "x":
                case "2":
                    t = controllers.get(port).button(2);
                    break;
                case "y":
                case "3":
                    t = controllers.get(port).button(3);
                    break;
                case "leftBumper":
                case "4":
                    t = controllers.get(port).button(4);
                    break;
                case "rightBumper":
                case "5":
                    t = controllers.get(port).button(5);
                    break;
                case "back":
                case "6":
                    t = controllers.get(port).button(6);
                    break;
                case "start":
                case "7":
                    t = controllers.get(port).button(7);
                    break;
                case "leftJoystickPress":
                case "8":
                    t = controllers.get(port).button(8);
                    break;
                case "rightJoystickPress":
                case "9":
                    t = controllers.get(port).button(9);
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
                default:
                    t = null;
                    break;
            }

            triggerList.put((String) trigger.get("command"), t);
        }
        triggers = triggerList;
        return triggerList;
    }

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
}
