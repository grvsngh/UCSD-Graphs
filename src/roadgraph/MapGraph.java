/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	
	private HashMap<GeographicPoint, MapNode> nodeMap;// = new HasMap<>();
	private HashSet<MapEdge> edgeList;
	private HashMap<MapNode,Double> distanceTo;
	
	
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		nodeMap = new HashMap<>();
		edgeList = new HashSet<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return nodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> vertices = nodeMap.keySet();
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		
		return edgeList.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(nodeMap.containsKey(location)){
			return false;
		}else{
			nodeMap.put(location, new MapNode(location));
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
			if(length <= 0){
				throw new IllegalArgumentException();
			}
			if(nodeMap.get(from) == null || nodeMap.get(to) == null){
				throw new IllegalArgumentException();
			}
		    nodeMap.get(from).addEdge(to, roadName,roadType,length);
		    edgeList.add(new MapEdge(from,to,roadName,roadType, length));
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		if(start == null || goal == null ){
			throw new IllegalArgumentException();
		}
		HashMap<MapNode,MapNode> map = new HashMap<>();
		PriorityQueue<MapNode> nodesToBeVisited = new PriorityQueue<>();
		HashSet<MapNode> nodesVisited = new HashSet<>();
		MapNode current = null;
		MapNode startNode = nodeMap.get(start);
		MapNode goalNode = nodeMap.get(goal);
		MapNode temp = null;
		if(startNode == null || goalNode == null ){
			System.out.println("No edges exits between start and goal");
			return null;
		}
		
		nodesToBeVisited.add(startNode);
		nodeSearched.accept(startNode.getLocation());
		while(!nodesToBeVisited.isEmpty()){
			current = nodesToBeVisited.remove();
			nodeSearched.accept(current.getLocation());
			if(current.equals(goalNode)){
				break;
			}

				for(MapEdge o : current.getEdgeList()){//turn into function to make less ugly?
					temp = nodeMap.get(o.getEnd());
					if(!nodesVisited.contains(temp)){
						nodesVisited.add(temp);
						nodesToBeVisited.add(temp);
						map.put(temp, current);
					}
				}

		}
		
		if(!current.equals(goalNode)){
			return null;//no path
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		

		return createPath(map,goalNode,startNode);
	}
	
	private List<GeographicPoint> createPath(HashMap<MapNode,MapNode> map, MapNode goal, MapNode start){
		List<GeographicPoint> route = new LinkedList<>();
		MapNode current = goal;
		
		while(!current.equals(start)){
			((LinkedList<GeographicPoint>) route).addFirst(current.getLocation());
			//route.addFirst(map.get(current).getLocation());
			current = map.get(current);
		}
		((LinkedList<GeographicPoint>) route).addFirst(start.getLocation());
		return route;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		int count =0;
		//initialize priority queue(PQ), visited HashSet,
		//parent HashMap, and distances to infinity
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();
		//HashMap<MapNode, Integer> distances = new HashMap<>();
		double distance = Integer.MAX_VALUE;
		//InitializeQueue(queue);
		setLengthInfinite();
		
		MapNode current = null;
		MapNode temp = null;
		MapNode startNode = nodeMap.get(start);
		MapNode goalNode = nodeMap.get(goal);
		startNode.setLength(0);
		queue.add(startNode);
		while(!queue.isEmpty()){
			current = queue.remove();
			count++;
			if(!visited.contains(current)){
				visited.add(current);
				//REACHED GOAL
				if(current.equals(goalNode)){
                  
					 System.out.println(count);
					return createPath(parentMap,goalNode,startNode);
				}
				for(MapEdge n : current.getEdgeList()){
					temp = nodeMap.get(n.getEnd());
					if(!visited.contains(temp)){
						//if path through curr to n is shorter
						if(temp.getLength() > (n.getDistance()+current.getLength())){
							temp.setLength(n.getDistance()+current.getLength());
							parentMap.put(temp,current);
							queue.add(temp);
						}
					}
				}
			}
			
		}//endwhile, if we get here then there's no path
		
		 System.out.println(count);
		return null;
	}
	
	private void setLengthInfinite(){
		MapNode temp = null;
		for(GeographicPoint n: nodeMap.keySet()){
			nodeMap.get(n).setLength(Integer.MAX_VALUE);
			nodeMap.get(n).setPredictedLength(Integer.MAX_VALUE);
		}
	}
	
	private double predictLength(MapNode start, MapNode goal){
		double distance=0;
		GeographicPoint startLoc = start.getLocation();
		GeographicPoint endLoc = goal.getLocation();
		distance = Math.abs(startLoc.distance(endLoc));
		
		return distance;
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		int count =0;
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();
		//HashMap<MapNode, Integer> distances = new HashMap<>();
		double distance = Integer.MAX_VALUE;
		//InitializeQueue(queue);
		setLengthInfinite();
		
		MapNode current = null;
		MapNode temp = null;
		MapNode startNode = nodeMap.get(start);
		MapNode goalNode = nodeMap.get(goal);
		startNode.setLength(0);
		queue.add(startNode);
		while(!queue.isEmpty()){
			current = queue.remove();
			count++;
			if(!visited.contains(current)){
				visited.add(current);
				//REACHED GOAL
				if(current.equals(goalNode)){
                    System.out.println(count);
					return createPath(parentMap,goalNode,startNode);
				}
				for(MapEdge n : current.getEdgeList()){
					temp = nodeMap.get(n.getEnd());
					if(!visited.contains(temp)){
						//if path through curr to n is shorter
						distance = predictLength(temp, goalNode);
						if(temp.getLength() > (n.getDistance()+current.getLength()+distance)){
							temp.setLength(n.getDistance()+current.getLength()+distance);
							parentMap.put(temp,current);
							queue.add(temp);
						}
					}
				}
			}
			
		}//endwhile, if we get here then there's no path
		 System.out.println(count);
		return null;
	}

	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		//firstMap.printGraph();
		
		//System.out.println();
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
		
	}
	
}
