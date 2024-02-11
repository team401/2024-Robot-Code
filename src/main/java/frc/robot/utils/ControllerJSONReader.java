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
    private static HashMap<Integer, CommandXboxController> gamepads;
    private static HashMap<Integer, CommandJoystick> joysticks;

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

    private static void setControllers(JSONArray controllersJSON) {
        HashMap<Integer, CommandXboxController> gamepadsList = new HashMap<Integer, CommandXboxController>();
        HashMap<Integer, CommandJoystick> joysticksList = new HashMap<Integer, CommandJoystick>();
        Iterator<JSONObject> iterator = controllersJSON.iterator();
        while(iterator.hasNext()) {
            JSONObject controller = iterator.next();
            if (((String)controller.get("type")).equals("joysticks")) 
                joysticksList.put((Integer) controller.get("port"), new CommandJoystick((int) controller.get("port")));
            else 
                gamepadsList.put((Integer) controller.get("port"), new CommandXboxController((int) controller.get("port")));
        }
        gamepads = gamepadsList;
        joysticks = joysticksList;
    }

    private static HashMap<String, Trigger> setTriggers(JSONArray triggersJSON) {
        HashMap<String, Trigger> triggerList = new HashMap<String, Trigger>();
        Iterator<JSONObject> iterator = triggersJSON.iterator();
        while(iterator.hasNext()) {
            JSONObject trigger = iterator.next();
            Trigger t;
            
            int port = (int) trigger.get("controller");
            switch (((String) trigger.get("button"))) {
                //gamepad cases
                case "x":
                    t = gamepads.get(port).x();
                case "y":
                    t = gamepads.get(port).y();
                case "a":
                    t = gamepads.get(port).a();
                case "b":
                    t = gamepads.get(port).b();
                case "back":
                    t = gamepads.get(port).back();
                case "start":
                    t = gamepads.get(port).start();
                case "leftTrigger":
                    t = gamepads.get(port).leftTrigger();
                case "leftBumper":
                    t = gamepads.get(port).leftBumper();
                case "rightTrigger":
                    t = gamepads.get(port).rightTrigger();
                case "rightBumper":
                    t = gamepads.get(port).rightBumper();
                case "pov0":
                    t = gamepads.get(port).pov(0);
                    case "pov0":
                    t = gamepads.get(port).pov(0);
            }
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
