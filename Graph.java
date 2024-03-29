import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;

public class Graph {

    public static void main(String[] args) {
        if (args.length != 1) {
            System.out.println("Usage: java Graph <filename>");
            return;
        }

        String filename = args[0]; 

        try {
            File file = new File(filename);
            Scanner fileScanner = new Scanner(file);

            fileScanner.nextLine();
            
            if (fileScanner.hasNextLine()) {
                String secondLine = fileScanner.nextLine();
                String[] parts = secondLine.split(" ");
                int numberOfVertices = Integer.parseInt(parts[0]);
                int numberOfEdges = Integer.parseInt(parts[1]);

                System.out.println("Number of vertices: " + numberOfVertices);
                System.out.println("Number of edges: " + numberOfEdges);
            } 
            
            while (fileScanner.hasNextLine()) {
                String line = fileScanner.nextLine();
                String[] coords = line.split(" ");
                if(Character.isDigit(coords[0].charAt(0)))
                {
                    int sourceNum = Integer.parseInt(coords[0]);
                    int destinationNum = Integer.parseInt(coords[1]);
                    String road = coords[2];
                    //addEdge()
                }else
                {
                    String label = coords[0];
                    double latitude = Double.parseDouble(coords[1]);
                    double longitude = Double.parseDouble(coords[2]);
                    //addvertex()
                }
                System.out.println("Read line: " + line); 
            }

            fileScanner.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found: " + filename);
        }
    }
}
