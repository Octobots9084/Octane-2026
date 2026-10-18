package frc.robot.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

public class CreateAutoDuplicates {


    private static String oldAutoName = "Wait Left";//DONT DO \"    //put the path name here to duplicate, it will go in Generated Flipped Autos by default
    private static String goalAutoFolder = "Right";//DONT DO \"    //put the auto folder here if you dont want it in Generated Flipped Autos
    public static String goalPathFolder = "Right Side";//DONT DO \"    //put the path folder here if you dont want it in Generated Flipped Paths


    private static String oldAuto;
    private static String newAutoName;
    private static String newAuto;

    public static void main(String[] args) {
        try {
            newAutoName = replaceSideName(oldAutoName);

            if (newAutoName == null) {
              newAutoName = "(Flipped) " + oldAutoName;
            }

            goalPathFolder = "\"" + goalPathFolder + "\"";
            goalAutoFolder = "\"" + goalAutoFolder + "\"";

            oldAuto = "src/main/deploy/pathplanner/autos/" + oldAutoName + ".auto";
            newAuto = "src/main/deploy/pathplanner/autos/" + newAutoName + ".auto";
            Files.deleteIfExists(Path.of(newAuto));

            BufferedReader inputFile = new BufferedReader(new FileReader(oldAuto));
            double numLines = Files.lines(Paths.get(oldAuto)).count();
            String line;
            
            for (int i = 0; i < numLines; i++) {
                String p1;
                String path;
                line = inputFile.readLine();
                System.out.println(line);
                
                if (line.contains("\"pathName\": ")) {
                    path = replaceSideName(line);

                    if (path != null) {
                        CreatePathDuplicatesForAuto.main(new String[]{
                            line.substring(
                              line.indexOf("\"pathName\": ") + 13,
                              line.length() - 1
                            )
                          ,
                          path.substring(
                              path.indexOf("\"pathName\": ") + 13,
                              path.length() - 1
                            )
                          ,
                            goalPathFolder
                        });//name, side
                      Files.writeString(Path.of(newAuto), path + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } else {
                        CreatePathDuplicates.main(new String[]{line.substring(line.indexOf("\"pathName\": ") + 13, line.length() - line.substring(line.indexOf("\"pathName\": ")).indexOf("\"") - 1), goalPathFolder});//name, side
                        Files.writeString(Path.of(newAuto), line.substring(0, line.indexOf("\"pathName\": ") + 13) + "(Flipped) " + line.substring(line.indexOf("\"pathName\": ") + 13, line.length() - line.substring(line.indexOf("\"pathName\": ")).indexOf("\"") - 1) + line.substring(line.length() - line.substring(line.indexOf("\"pathName\": ")).indexOf("\"") - 1) + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    }
                } else if (line.contains("\"folder\": ")) {
                    p1 = line.substring(0, line.indexOf(":") + 1);
                    if (goalAutoFolder.equals("")) {
                        Files.writeString(Path.of(newAuto), p1 + " " + "\"Generated Flipped Autos\"," + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } else {
                        Files.writeString(Path.of(newAuto), p1 + " " + goalAutoFolder + "," + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    }
                } else {
                    if (i == numLines - 1) {
                        Files.writeString(Path.of(newAuto), line, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } else {
                        Files.writeString(Path.of(newAuto), line + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    }
                }
            }

            inputFile.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static String replaceSideName (String name) {
          String newAutoName = name;
          if (name.toLowerCase().contains("left")) {
              newAutoName = newAutoName.replace("left", "right");
              newAutoName = newAutoName.replace("Left", "Right");
              newAutoName = newAutoName.replace("LEFT", "RIGHT");
              return newAutoName;
          } else if (name.toLowerCase().contains("right")) {
              newAutoName = newAutoName.replace("right", "left");
              newAutoName = newAutoName.replace("Right", "Left");
              newAutoName = newAutoName.replace("RIGHT", "LEFT");
              return newAutoName;
          }
            return null;
        }
}