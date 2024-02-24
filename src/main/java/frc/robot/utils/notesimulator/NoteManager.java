package frc.robot.utils.notesimulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.function.Supplier;

public class NoteManager {

    private static ArrayList<Note> notesOnField = new ArrayList<Note>();
    private static Note noteInRobot;

    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    public NoteManager(Supplier robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public static void addNote(Note note) {
        if (note.inRobot() && noteInRobot == null) noteInRobot = note;
        else {
            notesOnField.add(note);
        }
    }

    public static void intake() {
        if (noteInRobot == null) {
            for (Note note : notesOnField) {
                if (note.intakeNote()) {
                    noteInRobot = note;
                    notesOnField.remove(note);
                }
            }
        }
    }

    public static Command shoot(double aimRPM, double aimAngle) {
        if (noteInRobot != null) {
            Note shotNote = noteInRobot;
            noteInRobot = null;
            return shotNote.shoot(aimRPM, aimAngle);
        } else return null;
    }
}
