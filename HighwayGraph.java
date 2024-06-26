/**
 * An undirected, adjacency-list based graph data structure developed
 * specifically for METAL highway mapping graphs.
 * 
 * Starter implementation for the METAL Learning Module
 * Working with METAL Data
 * 
 * @author Chris LaDuke, Andrew Jop, Andrew Brockley, Fred DeKoker
 * @version April 5 2024
 */

import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Scanner;
import java.util.Random;

public class HighwayGraph {

    private static final DecimalFormat df = new DecimalFormat("#.###");

    // Small, internal data structure representing a
    // latitude-longitude pair. It has the added benefit
    // of being able to compute its distance to another
    // LatLng object.
    private class LatLng {
        private double lat, lng;

        public LatLng(double lat, double lng) {
            this.lat = lat;
            this.lng = lng;
        }

        /**
         * compute the distance in miles from this LatLng to another
         * 
         * @param other another LatLng
         * @return the distance in miles from this LatLng to other
         */
        public double distanceTo(LatLng other) {
            /** radius of the Earth in statute miles */
            final double EARTH_RADIUS = 3963.1;

            // did we get the same point?
            if (equals(other))
                return 0.0;

            // coordinates in radians
            double rlat1 = Math.toRadians(lat);
            double rlng1 = Math.toRadians(lng);
            double rlat2 = Math.toRadians(other.lat);
            double rlng2 = Math.toRadians(other.lng);

            return Math.acos(Math.cos(rlat1) * Math.cos(rlng1) * Math.cos(rlat2) * Math.cos(rlng2) +
                    Math.cos(rlat1) * Math.sin(rlng1) * Math.cos(rlat2) * Math.sin(rlng2) +
                    Math.sin(rlat1) * Math.sin(rlat2)) * EARTH_RADIUS;
        }

        /**
         * Compare another LatLng with this for equality, subject to the
         * specified tolerance.
         * 
         * @param o the other LatLng
         * @pre o instanceof LatLng
         * @return whether the two lat/lng pairs should be considered equal
         */
        public boolean equals(Object o) {
            final double TOLERANCE = 0.00001;
            LatLng other = (LatLng) o;

            return ((Math.abs(other.lat - lat) < TOLERANCE) &&
                    (Math.abs(other.lng - lng) < TOLERANCE));
        }

        public String toString() {
            return "(" + lat + "," + lng + ")";
        }
    }

    // our private internal structure for a Vertex
    private class Vertex {
        private String label;
        private LatLng point;
        private Edge head;

        public Vertex(String l, double lat, double lng) {
            label = l;
            point = new LatLng(lat, lng);
        }

    }

    // our private internal structure for an Edge
    private class Edge {

        // the edge needs to know its own label, its destination vertex (note that
        // it knows its source as which vertex's list contains this edge), an
        // optional array of points that improve the edge's shape, and its length
        // in miles, which is computed on construction
        private String label;
        private int dest;
        private LatLng[] shapePoints;
        private double length;
        private int numHouses;

        // and Edge is also a linked list
        private Edge next;

        public Edge(String l, int dst, LatLng startPoint, LatLng points[], LatLng endPoint, Edge n) {
            label = l;
            dest = dst;
            shapePoints = points;
            next = n;
            length = 0.0;
            LatLng prevPoint = startPoint;
            if (points != null) {
                for (int pointNum = 0; pointNum < points.length; pointNum++) {
                    length += prevPoint.distanceTo(points[pointNum]); //Calculate distaces from point to point
                    prevPoint = points[pointNum];
                }
            }
            length += prevPoint.distanceTo(endPoint);
            numHouses = (int) (10 * length); //10 houses a mile
        }
    }

    // vertices -- we know how many at the start, so these
    // are simply in an array
    private Vertex[] vertices;

    // number of edges
    private int numEdges;

    // construct from a TMG format file that comes from the given
    // Scanner (likely over a File or URLConnection, but does not
    // matter here)
    public HighwayGraph(Scanner s) {

        // read header line -- for now assume it's OK, but should
        // check
        s.nextLine();

        // read number of vertices and edges
        int numVertices = s.nextInt();
        numEdges = s.nextInt();

        // construct our array of Vertices
        vertices = new Vertex[numVertices];

        // next numVertices lines are Vertex entries
        for (int vNum = 0; vNum < numVertices; vNum++) {
            vertices[vNum] = new Vertex(s.next(), s.nextDouble(), s.nextDouble());
        }

        // next numEdge lines are Edge entries
        for (int eNum = 0; eNum < numEdges; eNum++) {
            int v1 = s.nextInt();
            int v2 = s.nextInt();
            String label = s.next();
            // shape points take us to the end of the line, and this
            // will be just a new line char if there are none for this edge
            String shapePointText = s.nextLine().trim();
            String[] shapePointStrings = shapePointText.split(" "); //storing coord pairs(lat,lng)
            LatLng v1Tov2[] = null;
            LatLng v2Tov1[] = null;
            if (shapePointStrings.length > 1) {
                // build arrays in both orders
                v1Tov2 = new LatLng[shapePointStrings.length / 2];
                v2Tov1 = new LatLng[shapePointStrings.length / 2];
                for (int pointNum = 0; pointNum < shapePointStrings.length / 2; pointNum++) {
                     // Extract coordinates from the input array and create a LatLng object
                    LatLng point = new LatLng(Double.parseDouble(shapePointStrings[pointNum * 2]),
                            Double.parseDouble(shapePointStrings[pointNum * 2 + 1]));
                    v1Tov2[pointNum] = point;
                    v2Tov1[shapePointStrings.length / 2 - pointNum - 1] = point;
                }
            }

            // build our Edge structures and add to each adjacency list
            vertices[v1].head = new Edge(label, v2, vertices[v1].point, v1Tov2, vertices[v2].point, vertices[v1].head);
            vertices[v2].head = new Edge(label, v1, vertices[v2].point, v2Tov1, vertices[v1].point, vertices[v2].head);
        }
    }

    //dfs that returns a string array (AI generated)
    public String[] depthFirstTraversal(int startVertexIndex) {
        // Array to store the traversal result
        String[] traversalResult = new String[vertices.length];
        // Boolean array to mark visited vertices
        boolean[] visited = new boolean[vertices.length];
    
        // Recursive depth-first traversal function
        dfs(startVertexIndex, traversalResult, visited, 0);
    
        return traversalResult;
    }
    private int dfs(int vertexIndex, String[] traversalResult, boolean[] visited, int currentIndex) {
        // Mark the current vertex as visited
        visited[vertexIndex] = true;
        // Add the label of the current vertex to the traversal result array
        traversalResult[currentIndex] = vertices[vertexIndex].label;
        currentIndex++;
    
        // Traverse all adjacent vertices of the current vertex
        Edge currentEdge = vertices[vertexIndex].head;
        while (currentEdge != null) {
            int adjacentVertexIndex = currentEdge.dest;
            // If the adjacent vertex has not been visited, recursively traverse it
            if (!visited[adjacentVertexIndex]) {
                currentIndex = dfs(adjacentVertexIndex, traversalResult, visited, currentIndex);
            }
            currentEdge = currentEdge.next;
        }
    
        return currentIndex;
    }

    //dfs that returns a house count (AI generated)
    public int dfsHouseCount(int startVertexIndex) {
        boolean[] visited = new boolean[vertices.length];
        int[] sumOfNumHouses = new int[1];
        dfsWithSum(startVertexIndex, visited, sumOfNumHouses);
        return sumOfNumHouses[0];
    }
    private void dfsWithSum(int vertexIndex, boolean[] visited, int[] sumOfNumHouses) {
        visited[vertexIndex] = true;
        
        Edge currentEdge = vertices[vertexIndex].head;
        while (currentEdge != null) {
            int adjacentVertexIndex = currentEdge.dest;
            if (!visited[adjacentVertexIndex]) {
                sumOfNumHouses[0] += currentEdge.numHouses;
                dfsWithSum(adjacentVertexIndex, visited, sumOfNumHouses);
            }
            currentEdge = currentEdge.next;
        }
    }

    //Generated using AI
    public void removeEdge(int vertexIndex1, int vertexIndex2) {
        // Remove edge from vertexIndex1's adjacency list
        Edge currentEdge = vertices[vertexIndex1].head;
        Edge prevEdge = null;
        while (currentEdge != null && currentEdge.dest != vertexIndex2) {
            prevEdge = currentEdge;
            currentEdge = currentEdge.next;
        }
        if (currentEdge != null) {
            if (prevEdge != null) {
                prevEdge.next = currentEdge.next;
            } else {
                vertices[vertexIndex1].head = currentEdge.next;
            }
        }

        // Remove edge from vertexIndex2's adjacency list
        currentEdge = vertices[vertexIndex2].head;
        prevEdge = null;
    
        // Loop through the linked list until reaching the end or finding an edge leading to vertexIndex1
        while (currentEdge != null && currentEdge.dest != vertexIndex1) {
            prevEdge = currentEdge;
            // Remove the currentEdge from the linked list by updating the next pointer of the previous edge
            currentEdge = currentEdge.next;
        }
        if (currentEdge != null) {
            if (prevEdge != null) {
                prevEdge.next = currentEdge.next;
            } else {
            // If prevEdge is null, meaning the edge to be removed is the first in the list
            // Update the head of the linked list to the next edge after currentEdge
                vertices[vertexIndex2].head = currentEdge.next;
            }
        }
    }

    // Generated Using AI
    public void addEdge(int vertexIndex1, int vertexIndex2, String label, LatLng[] shapePoints) {
        // Create a new edge between the vertices
        Edge newEdge1 = new Edge(label, vertexIndex2, vertices[vertexIndex1].point, shapePoints, vertices[vertexIndex2].point, vertices[vertexIndex1].head);
        Edge newEdge2 = new Edge(label, vertexIndex1, vertices[vertexIndex2].point, shapePoints, vertices[vertexIndex1].point, vertices[vertexIndex2].head);

        // Update the adjacency list of both vertices
        vertices[vertexIndex1].head = newEdge1;
        vertices[vertexIndex2].head = newEdge2;
    }

    //Check whether or not the input graph is connected
    public boolean connectedGraph(){
        boolean connected;
        String[] dftResult = depthFirstTraversal(0);
        int counter = 0;
        for (String v : dftResult){
            if(v != null){
                counter++;
            }
        }
        if(counter == vertices.length){
            connected = true;
        }
        else{
            connected = false;
        }
        return connected;
    }

    //Check input graph for bridges
    public String[] bridgeDetection(){
        String bridges[] = new String[numEdges];
        if(connectedGraph()){
            int currentVertex = 0;
            boolean visited;
            int bridgeCounter = 0;
            while (currentVertex<vertices.length){
                Edge currentEdge = vertices[currentVertex].head;
                while (currentEdge != null){
                    visited = false;
                    removeEdge(currentVertex, currentEdge.dest);
                    String[] traversed = depthFirstTraversal(currentVertex);
                    // Check if the destination vertex of the current edge is visited during traversal
                    for (String v : traversed) {
                        if(v != null && v.equals(vertices[currentEdge.dest].label)){
                            visited = true;
                        }
                    }
                    if (visited == false){
                        boolean newBridge = true;
                        String bridge;
                        if(currentVertex < currentEdge.dest){
                            bridge = vertices[currentVertex].label + " <-> " + vertices[currentEdge.dest].label;
                        }
                        else{
                            bridge = vertices[currentEdge.dest].label + " <-> " + vertices[currentVertex].label;
                        }
                        // Check if the bridge already exists
                        for (String b : bridges) {
                            if(b != null && b.equals(bridge)){
                                newBridge = false;
                            }
                        }
                        if (newBridge == true){
                            bridges[bridgeCounter] = bridge;
                            bridgeCounter++;
                        }
                    }
                    // Restore the removed edge
                    addEdge(currentVertex, currentEdge.dest, currentEdge.label, currentEdge.shapePoints);
                    currentEdge = currentEdge.next; //Move to next edge of current vertex
                }
                currentVertex++;
            }
        }
        else{
            return bridges;
        }
        return bridges;
    }

    public void bridgePrint(){
        String[] bridges = bridgeDetection();
        int bridgeCount = 0;
        for(String b : bridges){
            if(b != null){
                bridgeCount++;
            }
        }
        System.out.println("This graph has " + bridgeCount + " bridges:");
        for (int i=0; i<bridgeCount; i++){
            System.out.println(bridges[i]);
        }
        System.out.println("");
    }

    public void bridgeDetectionWithOutput(){
        if(connectedGraph()){
            System.out.println("Graph is connected, bridge detection will proceed.");
            System.out.println("");
            int currentVertex = 0;
            boolean visited;
            String bridges[] = new String[numEdges];
            int bridgeCounter = 0;
            while (currentVertex<vertices.length){
                Edge currentEdge = vertices[currentVertex].head;
                while (currentEdge != null){
                    visited = false;
                    removeEdge(currentVertex, currentEdge.dest);
                    System.out.println("Edge " + vertices[currentVertex].label + " to " + vertices[currentEdge.dest].label + " removed.");
                    String[] traversed = depthFirstTraversal(currentVertex);
                    for (String v : traversed) {
                        if(v != null && v.equals(vertices[currentEdge.dest].label)){
                            visited = true;
                        }
                    }
                    if (visited == false){
                        System.out.println("Edge " + vertices[currentVertex].label + " to " + vertices[currentEdge.dest].label + " is a bridge.");
                        boolean newBridge = true;
                        String bridge;
                        if(currentVertex < currentEdge.dest){
                            bridge = vertices[currentVertex].label + " <-> " + vertices[currentEdge.dest].label;
                        }
                        else{
                            bridge = vertices[currentEdge.dest].label + " <-> " + vertices[currentVertex].label;
                        }
                        for (String b : bridges) {
                            if(b != null && b.equals(bridge)){
                                newBridge = false;
                            }
                        }
                        if (newBridge == true){
                            bridges[bridgeCounter] = bridge;
                            bridgeCounter++;
                        }
                    }
                    addEdge(currentVertex, currentEdge.dest, currentEdge.label, currentEdge.shapePoints);
                    System.out.println("Edge " + vertices[currentVertex].label + " to " + vertices[currentEdge.dest].label + " added back.");
                    currentEdge = currentEdge.next;
                }
                currentVertex++;
            }
            System.out.println("");
            bridgePrint();
        }
        else{
            System.out.println("");
            System.out.println("Graph is not connected, bridge detection will not proceed.");
            System.out.println("");
        }
    }

    public void stormSimulation(){
        if(connectedGraph()){
            System.out.println("Storm incoming...");
            System.out.println("");

            String[] bridgeArray = bridgeDetection();
            int bridgeCount = 0;
            for(String b : bridgeArray){
                if(b != null){
                    bridgeCount++;
                }
            }

            int newBridgeCount = 0;

            String destroyed[] = new String[numEdges];
            int destroyedCount = 0;
            Random rand = new Random();

            int currentVertex = 0;
            while (currentVertex < vertices.length){
                Edge currentEdge = vertices[currentVertex].head;
                while (currentEdge != null){
                    int num = rand.nextInt(12);
                    if(num == 0){
                        String road;
                        if(currentVertex < currentEdge.dest){
                            road = vertices[currentVertex].label + " <-> " + vertices[currentEdge.dest].label;
                        }
                        else{
                            road = vertices[currentEdge.dest].label + " <-> " + vertices[currentVertex].label;
                        }
                        boolean alreadyDestroyed = false;
                        for(String db : destroyed){
                            if(db != null && db.equals(road)){
                                alreadyDestroyed = true;
                            }
                        }
                        if(!alreadyDestroyed){
                            destroyed[destroyedCount] = road;
                            destroyedCount++;
                            removeEdge(currentVertex, currentEdge.dest);
                            for(String b : bridgeArray){
                                if(b != null && b.equals(road)){
                                    String destroyedBridge;
                                    int leftHouseCount;
                                    int rightHouseCount;
                                    if(currentVertex < currentEdge.dest){
                                        destroyedBridge = vertices[currentVertex].label + " <--[" + currentEdge.numHouses + " homes]--> " + vertices[currentEdge.dest].label;
                                        leftHouseCount = dfsHouseCount(currentVertex);
                                        rightHouseCount = dfsHouseCount(currentEdge.dest);
                                    }
                                    else{
                                        destroyedBridge = vertices[currentEdge.dest].label + " <--[" + currentEdge.numHouses + " homes]--> " + vertices[currentVertex].label;
                                        rightHouseCount = dfsHouseCount(currentVertex);
                                        leftHouseCount = dfsHouseCount(currentEdge.dest);
                                    }
                                    System.out.println("Bridge destroyed! [" + leftHouseCount + " homes] " + destroyedBridge + " [" + rightHouseCount + " homes]");
                                    System.out.println("");
                                    return;
                                }
                            }
                            System.out.println("Road destroyed: " + road);
                            
                            bridgeArray = bridgeDetection();
                            newBridgeCount = 0;
                            for(String b : bridgeArray){
                                if(b != null){
                                    newBridgeCount++;
                                }
                            }
                            if(newBridgeCount > bridgeCount){
                                System.out.println("New bridges created.");
                                System.out.println("");
                                bridgePrint();
                                bridgeCount = newBridgeCount;
                            }
                        }
                    }
                    currentEdge = currentEdge.next;
                }
                currentVertex++;
            }
            System.out.println("No bridges were destroyed in the storm.");
            System.out.println("");
        }
        else{
            System.out.println("");
            System.out.println("Graph is not connected, bridge detection will not proceed.");
            System.out.println("");
        }
    }

    // try it out
    public static void main(String args[]) throws IOException {

        if (args.length != 1) {
            System.err.println("Usage: java HighwayGraph tmgfile");
            System.exit(1);
        }

        // read in the file to construct the graph
        Scanner s = new Scanner(new File(args[0]));
        HighwayGraph g = new HighwayGraph(s);
        s.close();

        // String[] dftResult = g.depthFirstTraversal(0);
        // System.out.println("Depth-First Traversal:");
        // for (String label : dftResult) {
        //     System.out.println(label);
        // }
        // System.out.println("");

        g.bridgeDetectionWithOutput();
        System.out.println("--------------------------------------------------------------------------------------------------");
        System.out.println("");
        g.stormSimulation();
    }

}
