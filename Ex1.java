import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;

/*
The passageExits() method iterates through all adjacent exits and increments a counter when robot.look(direction) == iRobot.PASSAGE. Similar logic is used for the nonWallExits() method.
I also included a randomPassage() method that simplifies code and improves modularity of the program.

The four controller methods determine how to deal with each case, with junction and crossroad being identical, therefore I combined both methods to create a single “junction” method.
This method uses two array lists with different priorities, the higher priority list contains passages, and lower priority contains explored paths.

The higher priority list is randomly selected if it contains elements, otherwise an exit is selected randomly.  Corridor iterates through all directions that aren’t behind and returns the other exit.
The deadEnd() method iterates through all possible moves and returns the direction where no wall is encountered, this prevents issues of starting in a dead end.
Efficiency is improved by using directional variables relationships (e.g. AHEAD, LEFT, RIGHT, BEHIND) and avoiding repeated selection.

Explore control utilises the 3 control methods and adds unencountered junctions to robotData. Backtrack determines the case e.g. (junction/dead end/corridor) if it is a junction
the method determines if there are passages, if so, it explores them and sets the controller to explore mode. Otherwise, it exits the junction in the same direction as when first encountered,
backtracking to the next junction. Corridor and dead end utilise the control methods.

The RobotData class utilises a list of JunctionRecord objects to retrieve junction properties. The searchJunction() method, returns the direction the junction at specified coordinates
was first entered from, using the junctionRecord object. Each junction object store x and y coordinates as well as direction arrived at the junction, which can be accessed within
the RobotData class via the object’s getter methods.

This program utilises a depth first search, this implies the maximum number of moves is 2N-1, when N is the number of edges in the maze.
This is because each path is traversed twice except the final path. Such that every path is explored fully, until the end is reached.

*/

public class Ex1{

    private int pollRun = 0;
    private RobotData robotData;
    private int explorerMode; // 1 = explore , 0 = backtrack

    public void controlRobot(IRobot robot) {

        int direction;

		if((robot.getRuns() == 0) && (pollRun == 0)){
            robotData = new RobotData();
            robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
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
                    robotData.addJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading());
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
                nonWalls++;
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
    public void reset() {
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
    * getter methods return private variables
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
