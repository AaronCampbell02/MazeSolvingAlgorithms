import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;

/*
* This exercise reduces memory usage in comparison to exercise one by utilsing a stack
* and a stack pointer, not requiring the x and y values of each junction.

* Each junction is pushed onto the stack when first encountered and popped once fully explored
* This directly matches how a depth first search is executed therefore can be used
* to match the algorithm and save storage space
*/

public class Ex2{

    private int pollRun = 0;
    private RobotData robotData;
    private int explorerMode; // 1 = explore , 0 = backtrack

    public void controlRobot(IRobot robot) {

        int direction;

		if((robot.getRuns() == 0) && (pollRun == 0)){
            robotData = new RobotData();
            robotData.addJunction(robot.getHeading());
            explorerMode = 1;
		}
        if (explorerMode == 1){
            direction = exploreControl(robot);
        }else{
            direction = backtrackControl(robot);
        }
        robot.face(direction); // set robot direction
        pollRun++;
    }
    private int exploreControl(IRobot robot){
        /*
        * Method calculates the number of exits, which determines which case is encountered
        * respective methods are callled to deal with case
        * Direction is returned to control robot
        */
        int exits = nonWallExits(robot);
        int direction;
        switch(exits){ 
			case 2:
				direction = corridor(robot);
				break;
			case 1:
				direction = deadEnd(robot);
				explorerMode = 0;
				break;
			default:
				direction = junction(robot);
				if (beenBefore(robot) <= 1){ //store junction
                    robotData.addJunction(robot.getHeading());
				}
		}

        return direction;
    }
    private int backtrackControl(IRobot robot){
        /*
        * The backtrack method returns the direction which the robot should backtrack to or explore
        * The case is determined by the number of exits if not a junction respective method is called
        * If a junction it will explore if possible, if not it backtrack to the next junction
        */
        int exits = nonWallExits(robot);
        int direction;
        if (exits >2){
            int passages = passageExits(robot); //determine number of passages
            if (passages > 0){ // set to explore mode if passage
                explorerMode = 1;
                direction = randomPassage(robot);
            }else{
                // if no passages exit the same way first encountered
                direction = robotData.searchJunctions();
                direction = IRobot.AHEAD + ((direction - robot.getHeading() + 4) % 4 + 2) % 4; // convert heading to relative direction and reverse it
                robotData.exploredJunction(); //pop the junction from the stack
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
                passageDir.add(i);
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
    private int junctionPointer = -1;

    public RobotData(){
        this.junctions = new ArrayList<JunctionRecord>();
    }
    public void addJunction(int arrived){ 
        /*
        * initialises new junction and adds to junctions list as well as update the stack pointer
        */
        JunctionRecord newJunction = new JunctionRecord(arrived);
        junctions.add(newJunction);
        junctionPointer = junctions.size()-1;
    }
    public void resetData(){
        this.junctions = new ArrayList<JunctionRecord>(); //reset list
    }
    public int searchJunctions(){
        /*
        * direction arrived of the junction at the top of the stack
        */
        JunctionRecord j = junctions.get(junctionPointer);
        return j.getJunctionArrived();
    }
    public void exploredJunction(){
        /*
        * remove the junction at the top of the stack and decrement pointer
        */
        junctions.remove(junctionPointer);
        junctionPointer --;
    }

}
class JunctionRecord{ // junction class
    private int junctionArrived;
    public JunctionRecord(int arrived){
        this.junctionArrived = arrived;
    }
    /*
    * getter method return private variables
    */
    public int getJunctionArrived() {
        return junctionArrived;
    }
}
