import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;

/*
* To solve the exercise I ensured the robot backtracks and reverses direction everytime that it meets a square that it has already visited, this prevents random logic and allows
* the robot to peform DFS like logic on a loopy maze
*
* My previous soloution didn't work as the robot would get stuck in loops as well as if an unexplored junction was encountered while backtracking,
* which isn't possible in normal DFS mazes it would cause looping, so I had to account for this by utilising a findJunction method
* This accounts for more comprehisive logic to prevent repeated junction recordings and prevents junctions not being recorded unlike with the previous program
* as well as preventing loops in the backtrack mode
*/

public class Ex3{

    private int pollRun = 0;
    private RobotData robotData;
    private int explorerMode; // 1 = explore , 0 = backtrack

    public void controlRobot(IRobot robot) {
        /*
        * This method sets the direction which the robot faces
        * if the robot is exploring and reaches a been before square it should backtrack and its direction is reversed
        */
        int direction;

		if((robot.getRuns() == 0) && (pollRun == 0)){
            robotData = new RobotData();
            // prevents issues by backtracking to start if no junction is met e.g. uncommon maze layout such as corridor leading to deadend
            robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
            explorerMode = 1; //start in explore mode
		}
		
        if (explorerMode == 1){ //robot explores
            direction = exploreControl(robot);
        }else{ //robot backtracks
            direction = backtrackControl(robot);
        }

        robot.face(direction); // set robot direction
        if(explorerMode ==1 && robot.look(IRobot.AHEAD)== IRobot.BEENBEFORE){
            explorerMode = 0;
            direction = IRobot.BEHIND;
            robot.face(direction);
            direction = backtrackControl(robot);
            robot.face(direction); // set robot direction
        }
        pollRun++;
    }
    private int exploreControl(IRobot robot){
        /*
        * Method calculates the number of exits, which determines which case is encountered
        * respective methods are callled to deal with case
        * Direction is returned to control robot
        */
        int exits = nonWallExits(robot); // calculate number of exits from current position
        int direction;
        switch(exits){ // determine which case the robot is encountering
			case 2:
				direction = corridor(robot);
				break;
			case 1:
				direction = deadEnd(robot);
				explorerMode = 0;
				break;
			default:
				direction = junction(robot);
				if(robotData.findJunction(robot.getLocation().x,robot.getLocation().y) == false){ //store junction
                    robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
				}
		}

        return direction;
    }
    private int backtrackControl(IRobot robot){
        /*
        * The backtrack method returns the direction which the robot should backtrack to or explore
        * The case is determined by the number of exits if not a junction respective method is called
        * If the junction is unexplored and found while backtracking it is added to the arrayList and explored
        * If a junction it will explore if possible, if not it backtrack to the next junction
        */
        int exits = nonWallExits(robot);
        int direction;
        if (exits >2){
            if(robotData.findJunction(robot.getLocation().x,robot.getLocation().y) == false){ //unexplored junction found while backtracking
                robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
                explorerMode = 1;
                return exploreControl(robot);
            }
            if (passageExits(robot) > 0){ // set to explore mode if passage
                explorerMode = 1;
                direction = randomPassage(robot);
            }else{
                // if no passages exit the same way first encountered
                direction = robotData.searchJunctions(robot.getLocation().x,robot.getLocation().y);
                direction = IRobot.AHEAD + ((direction - robot.getHeading() + 4) % 4 + 2) % 4; // convert heading to relative direction and reverse it
            }
        }else if (exits == 2){ // call respective methods
            direction = corridor(robot);
        }else{
            direction = deadEnd(robot);
        }
        return direction;
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
    public void reset() { //reset all data
        robotData.resetData();
        pollRun = 0;
        explorerMode = 1;
    }
}

class RobotData{
    private ArrayList<JunctionRecord> junctions; //array of junction objects

    public RobotData(){
        this.junctions = new ArrayList<JunctionRecord>();
    }
    public void addJunction(int x, int y, int arrived){
        /*
        * initialises new junction and adds to junctions list
        */
        JunctionRecord newJunction = new JunctionRecord(x,y,arrived);
        junctions.add(newJunction);
    }
    public void resetData(){
        this.junctions = new ArrayList<JunctionRecord>(); //reset list
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
    public boolean findJunction(int x, int y){
        /*
        * returns true if the junction is in the junctions list and false if not
        */
        for (JunctionRecord j : junctions) {
            if (j.getJunctionX() == x && j.getJunctionY() == y) {
                return true; //junction found
            }
        }
        return false; //junction not found
    }

}
class JunctionRecord{ // junction class
    private int junctionX;
    private int junctionY;
    private int junctionArrived;
    public JunctionRecord(int x, int y, int arrived){
        this.junctionX = x;
        this.junctionY = y;
        this.junctionArrived = arrived;
    }
    /*
    * getter method return private variables
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
}
