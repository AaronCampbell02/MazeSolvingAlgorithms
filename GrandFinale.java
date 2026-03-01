import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;
import java.util.Collections;

/*
* My approach to the grand finale was to create a graph of junctions using array based structures which maps junctions together, by connections creating a graph.
* This is a similar approach to how route A attempts to solve the client’s request. This allows the program to solve all types of mazes.
* By forming the shortest path from the junctions and their connections, combined with the distance between them.
* This approach seemed the most logical to support the robot learning by recreating the shortest path for the previous exploration by utilising Dijkstra’s algorithm.
* This approach is more effective for solving loopy mazes than route B. To improve the effectiveness of Dijkstra’s algorithm I allowed the robot to move
* into the been before square then return back to the previous square and backtrack, such that if it was a junction a connection could be made.
* This made the output shortest path more direct to the end, reducing the number of steps and junctions encountered. 
* 
* My robot is robust and able to solve new mazes as well as loopy mazes by building upon exercise 3 and resetting data every time a new maze is encountered
* so that a shortest path can be generated. This works by resetting variables and creating a new robotData object, allowing the robot to explore the new maze
* and create a new graph. On the first run of the maze the robot explores it in DFS style, any run after this the shortest path solution would be used instead.
* 
* Connections are added as undirected edges in addConnection as this produces an improved solution, finding a shorter path than if the connections where directed.
* While this is not necessary for non loopy mazes this durastically improves the peformance for the loopzy mazes
* Absolute headings are used throughout then only translate into relative headings before being returned to the control robot method,
* making the graph and shortest path independent of the currents robots heading.
*
* I decided to use Dijkstra’s instead of alternative searching algorithms such as A star due to the size of the maze being relatively small (max 200x200),
* so efficiency isn’t the main focus and the ideal solution is more favorable in the tradeoff for a slower algorithm and A* is not necessary.
*/

public class GrandFinale{

    private int pollRun = 0;
    private RobotData robotData;
    private Dijkstras dijkstras;
    private int explorerMode; // 1 = explore , 0 = backtrack
    private int lastJunction = 0;
    private int lastDirection = -1;
    private boolean updateLastDirection = true;
    private int steps;
    private ArrayList<Integer> shortestPath;
    private int counter;
    private int moves;
    private boolean hasToBackTrack = false;
    private int runs = 0;
    private int startX;
    private int startY;
    private boolean startRepeat = false;
    private boolean ignore = false;


    public void controlRobot(IRobot robot) {

        /*
        * This method sets the direction which the robot faces
        * if the robot is exploring and reaches a been before square it should backtrack and its direction is reversed
        * When a new maze is encountered data is reset so it can be explored and a new graph can be created to explore
        * When the maze has been explored before a shortest path is generated using dijkstras and followed
        * Robot is also faced south so that if starting in a corridor it always exits the same way
        */

        int direction;
        runs = robot.getRuns();

		if((runs == 0) && (pollRun == 0)){
            startX = robot.getLocation().x;
            startY = robot.getLocation().y;
            robotData = new RobotData();
            lastJunction = 0; //reset valuea for new maze
            lastDirection = -1;
            updateLastDirection = true;
            startRepeat = false;
            robot.setHeading(IRobot.SOUTH); // ensure if starting at corridor always goes the same way
            // ensures it always goes the same way
            robotData.addJunction(startX,startY, robot.getHeading());
            explorerMode = 1; //start in explore mode
            steps = 0;
		}else if(pollRun == 0){
            //generate shortest path
            int val = nonWallExits(robot);
            if(val > 2){
                startRepeat = false;
                counter = 0;
            }else{
                counter = 1;
            }
            moves = 0; 
            robot.setHeading(IRobot.SOUTH); // ensure if starting at corridor always goes the same way
            robotData.displayJunctions();
            dijkstras = new Dijkstras(robotData);
            shortestPath = dijkstras.findPath(lastJunction);
            System.out.println("shortest path");
            System.out.println(shortestPath);
		}
        if(runs == 0){
            //first run
            if(hasToBackTrack){
                // return to previous square
                explorerMode = 0;
                hasToBackTrack = false;
                direction = IRobot.BEHIND;
                robot.face(direction);
            }else{
                if(robot.getLocation().x == startX && robot.getLocation().y == startY && pollRun > 0){
                    if(robotData.junctionCount() < 2){
                        //indicates that first junction has been encountered again before meeting a junction
                        //this means it should leave out the other exit of the corridor
                        startRepeat = true;
                    }                   
                }
                if (explorerMode == 1){ //robot explores
                    direction = exploreControl(robot);
                }else{ //robot backtracks
                    direction = backtrackControl(robot);
                }

                robot.face(direction); // set robot direction
                if(explorerMode ==1 && robot.look(IRobot.AHEAD)== IRobot.BEENBEFORE){
                    hasToBackTrack = true; // explore the square in front to connect junctions
                    //then return to previous square and backtrack
                }
                if(updateLastDirection){
                    //store direction change in terms of heading
                    lastDirection = robot.getHeading();
                    updateLastDirection = false;
                }
                steps++;
            }
        }else{
            // follow shortest path
            direction = followPath(robot);
            robot.face(direction);
        }
        pollRun++;

    }

    private int followPath(IRobot robot){
        /*
        * This method follows the shortest path by determing the number of case and exploring normally if corridor or deadend
        * if a junction is encountered the current junction and next junction determiness the direction robot travels
        * so that it can reach the next junction via using the connection
        */
        int exits = nonWallExits(robot); // calculate number of exits from current position
        int direction = -1;
        int jNumCurrent;
        int jNumNext;
        JunctionRecord currentJunction;
        ArrayList<Integer> junctionList;
        ArrayList<Integer> directionList;
        switch(exits){ // determine which case the robot is encountering
			case 2:
                if(moves == 0 && startRepeat){ //leave out of second corridor exit
                    moves ++;
                    for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
                        if (robot.look(i) != IRobot.WALL) { //doesn't matter if behind at the start
                            direction = i;
                        }
                    }
                }else{
                    direction = corridor(robot);
                }
				break;
			case 1:
                //only encountered at the start of the maze
				direction = deadEnd(robot);
				break;
			default:
                if (counter == shortestPath.size()-1){
                    /*
                    * last junction therefore no connections
                    * leave the way that the robot last left that junction
                    * which is to the end
                    */
                    direction = lastDirection;
                    direction = IRobot.AHEAD + (direction - robot.getHeading() + 4) % 4; // convert heading to relative direction
                }else{
                    jNumNext = shortestPath.get(counter+1);
                    jNumCurrent = shortestPath.get(counter);
                    currentJunction = robotData.getJunction(jNumCurrent);
                    junctionList = currentJunction.getConnection();
                    directionList = currentJunction.getDirection();
                    

                    int jNum = -1;
                    int connectNumber = 0;
                    for(int i = 0; i < junctionList.size(); i++){ //determine connection index to retrieve direction
                        jNum = junctionList.get(i);
                        if(jNumNext == jNum){
                            connectNumber = i;
                        }
                    }

                    direction = directionList.get(connectNumber);
                    direction = IRobot.AHEAD + (direction - robot.getHeading() + 4) % 4; // convert heading to relative direction
                    counter ++;
                    
                }               

		}

        return direction;

    }

    private int exploreControl(IRobot robot){
        /*
        * Method calculates the number of exits, which determines which case is encountered
        * respective methods are callled to deal with case
        * Direction is returned to control robot
        * If a junction is encountered connection is initialised between last junction
        * and the current junction.
        */
        int exits = nonWallExits(robot); // calculate number of exits from current position
        int direction;
        int jNum = -1;
        switch(exits){ // determine which case the robot is encountering
			case 2:
				direction = corridor(robot);
				break;
			case 1:
				direction = deadEnd(robot);
				explorerMode = 0;
				break;
			default:
                /*
                * this is when exits are 3/4 (junction/crossroads)
                */
				direction = junction(robot);
				jNum = robotData.findJunction(robot.getLocation().x,robot.getLocation().y);
				if(jNum == -1){ //store junction if not encountered
                    robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
                    jNum = robotData.findJunction(robot.getLocation().x,robot.getLocation().y);
				}				
                robotData.addConnection(lastJunction,lastDirection,jNum,steps,robot);
                steps = 0;
                lastJunction = jNum;
                updateLastDirection = true;
		}
        return direction;
    }
    private int backtrackControl(IRobot robot){
        /*
        * The backtrack method returns the direction which the robot should backtrack to or explore
        * The case is determined by the number of exits if not a junction respective method is called
        * If the junction is unexplored and found while backtracking it is added to the arrayList and explored
        * If a junction it will explore if possible, if not it backtrack to the next junction
        * Connection is also initialised between the current and last junction
        */
        int exits = nonWallExits(robot);
        int direction = -1;
        boolean exploring = false;
        int x = robot.getLocation().x;
        int y = robot.getLocation().y;
        if (exits >2){
            int jNum = robotData.findJunction(x,y);
            if(jNum == -1){ //unexplored junction found while backtracking
                robotData.addJunction(x,y, robot.getHeading());
                exploring = true;
                explorerMode = 1;
            }else{
                if (passageExits(robot) > 0){ // set to explore mode if passage
                    explorerMode = 1;
                    direction = randomPassage(robot);
                }else{
                    // if no passages exit the same way first encountered
                    direction = robotData.searchJunctions(robot.getLocation().x,robot.getLocation().y);
                    direction = IRobot.AHEAD + ((direction - robot.getHeading() + 4) % 4 + 2) % 4; // convert heading to relative direction and reverse it
                }
            }

            jNum = robotData.findJunction(robot.getLocation().x,robot.getLocation().y);
            robotData.addConnection(lastJunction,lastDirection,jNum,steps,robot);
            steps = 0;
            lastJunction = jNum;
            updateLastDirection = true;
        }else if (exits == 2){ // call respective methods
            direction = corridor(robot);
        }else{
            direction = deadEnd(robot);
        }

        if (exploring){
            return exploreControl(robot);
        }else{
            return direction;
        }
        
    }

    private int nonWallExits (IRobot robot) {
        /*
        * iterates through all directions and returns number of walls encountered
        */
		int nonWalls = 0;
		for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) != IRobot.WALL) {
                nonWalls++; //increment counter
            }
        }
        return nonWalls;
    }
    private int randomPassage(IRobot robot){
        /*
        * iterates through all directions and adds to arraylist each passages encountered
        * before randomly selecting a passage
        */
        ArrayList<Integer> passageDir = new ArrayList<Integer>();
        for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) == IRobot.PASSAGE) {
                passageDir.add(i); // add to passage array list
            }
        }
        return (passageDir.get((int) Math.floor(Math.random()*passageDir.size())));

    }
    private int passageExits (IRobot robot) {
        /*
        * iterates through all directions and returns number of passages encountered
        */
		int passages = 0;
		for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) == IRobot.PASSAGE) {
                passages++;
            }
        }
        return passages;
    }
    private int beenBefore(IRobot robot){
        /*
        * iterates through all directions and returns number of been before squares encountered
        */
        int visited = 0;
        for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) == IRobot.BEENBEFORE) {
                visited++;
             }
        }
        return visited;
    }
    private int junction(IRobot robot){
        /*
        * this method returns a direction to a random passage or if not possible an exit
        * two array lists are used with passage having a greater priority
        */
        ArrayList<Integer> bestCase = new ArrayList<Integer>();
        ArrayList<Integer> worstCase = new ArrayList<Integer>();
        // bestCase list has higher priority than worstCase
        for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) != IRobot.WALL) {
                if(robot.look(i) == IRobot.PASSAGE){
                    bestCase.add(i);
                }else{
                    worstCase.add(i);
                }
            }
        }
        int direction;
        int randno;
        int passages = passageExits(robot);
        //return random exit, using bestCase first and worstCase if bestCase is empty
        if (passages == 0){
            direction = worstCase.get((int)Math.floor(Math.random()*worstCase.size()));
        }else{
            direction = bestCase.get((int) Math.floor(Math.random()*bestCase.size()));
        }
        return direction;
    }
    private int corridor(IRobot robot){
        /*
        * returns direction that has an exit and isnt backwards
        */
        for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (i != IRobot.BEHIND && robot.look(i) != IRobot.WALL) {
                return i;
            }
        }
        return IRobot.AHEAD; // default case, should a condition not be achieved
    }
    private int deadEnd(IRobot robot){
        /*
        * returns direction with an exit
        */
        for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
            if (robot.look(i) != IRobot.WALL) {
                return i;
            }
        }
        return IRobot.BEHIND; // default case to compile
    }

    public void reset() { //resets data
        pollRun = 0;
        explorerMode = 1;
    }
}

class RobotData{
    private ArrayList<JunctionRecord> junctions; //array of junction objects

    public RobotData(){ //constructor
        this.junctions = new ArrayList<JunctionRecord>();
    }
    public void addJunction(int x, int y, int arrived){
        /*
        * initialises new junction and adds to junctions list
        */
        JunctionRecord newJunction = new JunctionRecord(x,y,arrived);
        junctions.add(newJunction);
    }
    public int junctionCount(){
        return junctions.size();
    }
    public int searchJunctions(int x, int y){
        /*
        * direction arrived at the junction at x,y coordinates
        */
        for (JunctionRecord j : junctions) {
            if (j.getJunctionX() == x && j.getJunctionY() == y) {
                return j.getJunctionArrived(); //return arrival direction
            }
        }
        return -1; // junction not encountered before
    }
    public int findJunction(int x, int y){
        /*
        * returns true if the junction is in the junctions list and false if not
        */
        int counter = 0;
        for (JunctionRecord j : junctions) {
            if (j.getJunctionX() == x && j.getJunctionY() == y) {
                return counter; //junction found
            }
            counter ++;
        }
        return -1; //junction not found
    }
    public void addConnection(int lastJunction,int lastDirection,int jNum,int steps, IRobot robot){
        /*
        * Adds a undirected connection between two junction if the connection isn't
        * already stored and not a self loop 
        */
        if(lastJunction != jNum){ // prevents junctions being connected to themselves
            JunctionRecord j = junctions.get(lastJunction);
            JunctionRecord i = junctions.get(jNum);
            //prevent duplicate connections
            ArrayList<Integer> jConn = j.getConnection();
            ArrayList<Integer> jDirs = j.getDirection();
            ArrayList<Integer> jSteps = j.getSteps();
            for (int k = 0; k < jConn.size(); k++) {
                if (jConn.get(k) == jNum &&
                    jDirs.get(k) == lastDirection &&
                    jSteps.get(k) == steps) {
                    return; // already have this edge
                }
            }

            j.setConnection(lastDirection,jNum,steps);
            int reverseDirection = IRobot.NORTH+((robot.getHeading()+2)%4); //reverse the current heading direction
            i.setConnection(reverseDirection,lastJunction,steps);

        }
    }
    public void displayJunctions(){
        /*
        * iterates through each junction and displays them indavidually and there connections
        */
        int counter = 0;
        for(JunctionRecord j : junctions){
            j.display(counter);
            counter ++;
        }
    }
    /*
    * getter methods
    */
    public ArrayList<JunctionRecord> getList(){
        return junctions;
    }
    public int getJunctionNumber(JunctionRecord junction) {
        return junctions.indexOf(junction);
    }
    public JunctionRecord getJunction(int index) {
        return junctions.get(index);
    }
}

class JunctionRecord{ // junction class
    private int junctionX;
    private int junctionY;
    private int junctionArrived;
    private int lastLeft;
    private ArrayList<Integer> junctionList;
    private ArrayList<Integer> directionList;
    private ArrayList<Integer> stepsList;
    public JunctionRecord(int x, int y, int arrived){
        this.junctionX = x;
        this.junctionY = y;
        this.junctionArrived = arrived;
        
        this.junctionList = new ArrayList<>();
        this.directionList = new ArrayList<>();
        this.stepsList = new ArrayList<>();
    }
    /*
    * getter methods
    */
    public int getJunctionX() {
        return junctionX;
    }
    public int getJunctionY() {
        return junctionY;
    }
    public int getJunctionArrived() {
        return junctionArrived;
    }
    public int getJunction(int jNum){
        return junctionList.get(jNum);
    }
    public ArrayList<Integer> getConnection(){
        return junctionList;
    }
    public ArrayList<Integer> getDirection(){
        return directionList;
    }
    public ArrayList<Integer> getSteps(){
        return stepsList;
    }

    public void setConnection(int direction, int jNum,int steps){
        /*
        * initialises connections
        */
        junctionList.add(jNum);
        directionList.add(direction);
        stepsList.add(steps);
    }   
    public void display(int jNum) {
        /*
        * Displays junction and connections
        */
        System.out.print("Junction " + jNum + " ");
        
        for(int i = 0; i < junctionList.size(); i++){
            String directionName = getDirectionName(directionList.get(i));
            System.out.print("(connection: " + junctionList.get(i) + "position x :" + junctionX + "position y: " + junctionY + ", steps: " + stepsList.get(i) + ", direction: " + directionName + ") ");
            System.out.println();
        }
    }
    public String getDirectionName(int direction){
        if(direction == IRobot.NORTH) return "NORTH";
        if(direction == IRobot.SOUTH) return "SOUTH";
        if(direction == IRobot.EAST) return "EAST";
        if(direction == IRobot.WEST) return "WEST";
        return "ERROR";
    }
}
class Dijkstras{
    private int startJunction;
    private RobotData robotData;
    private ArrayList<JunctionRecord> junctions;
    private ArrayList<Integer> minSteps;
    private ArrayList<Integer> predecessors;
    private ArrayList<Boolean> visited;
    private ArrayList<JunctionRecord> pQueue; //priority queue

    public Dijkstras(RobotData robotData){
        this.robotData = robotData;
        this.junctions = robotData.getList();

        this.startJunction = 0;

        this.minSteps = new ArrayList<>();
        this.predecessors = new ArrayList<>();
        this.visited = new ArrayList<>();
        this.pQueue = new ArrayList<>();
    }

    public ArrayList<Integer> findPath(int lastJunction){
        /*
        * performs dijkstras algorithm and returns the shortest path
        * from the first to the final junction
        * 
        * Works by using a priority queue, and taking the junction with shortest distance
        * in the queue, selecting this junction adding connected junction to the queue
        * and updating their distance and predeccesor if shorter
        */
        int currentJnum;
        int distance;
        int finalJunction = lastJunction;
        JunctionRecord currentJunction;
        ArrayList<Integer> connections;
        ArrayList<Integer> stepsList;
        ArrayList<Integer> finalPath = new ArrayList<>();
        initialiseLists();

        pQueue.add(junctions.get(startJunction)); 

        while (pQueue.size() != 0){
            //retrieve data required
            currentJunction = getLowestJunction(pQueue);
            currentJnum = robotData.getJunctionNumber(currentJunction);
            pQueue.remove(currentJunction);
            connections = currentJunction.getConnection();
            stepsList = currentJunction.getSteps();

            int counter = 0;
            for(int jNum:connections){
                //iterate through connections

                if(visited.get(jNum) == false){
                    pQueue.add(junctions.get(jNum));
                }
                distance = minSteps.get(currentJnum) + stepsList.get(counter); //get distance to junction
                if(distance < minSteps.get(jNum)){
                    // if distance is less than the stored distance
                    //update distance and predecessors
                    minSteps.set(jNum, distance);
                    predecessors.set(jNum,currentJnum);
                }
                counter ++;
            }

            visited.set(currentJnum,true);
            
        }
        displayLists();
        currentJnum = finalJunction;
        while(currentJnum != startJunction){
            finalPath.add(currentJnum);
            currentJnum = predecessors.get(currentJnum);
        }
        finalPath.add(startJunction);
        Collections.reverse(finalPath);   // reverse list using collections package
        return finalPath;   

    }
    private void displayLists(){
        //display for testing
        System.out.println("visited");
        System.out.println(visited);
        System.out.println("predecessors");
        System.out.println(predecessors);
        System.out.println("distance");
        System.out.println(minSteps);
    }
    public JunctionRecord getLowestJunction(ArrayList<JunctionRecord> pQueue){
        /*
        * returns the junction with the lowest distance to it
        */
        int nextJ;
        JunctionRecord closestJ = pQueue.get(0);
        int distanceToJ = Integer.MAX_VALUE;
        int d;
        for(JunctionRecord j: pQueue){
            nextJ = robotData.getJunctionNumber(j);
            d = minSteps.get(nextJ);
            if(distanceToJ > d){
                closestJ = j;
                distanceToJ = d;
            }
        }
        return closestJ;

    }
    public void initialiseLists(){
        for(int i =0; i < junctions.size(); i ++){
            minSteps.add(Integer.MAX_VALUE);
            predecessors.add(-1);
            visited.add(false);
        }
        visited.set(startJunction,true);
        minSteps.set(startJunction, 0);
    }
}
