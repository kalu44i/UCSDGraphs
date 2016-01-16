/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 *         <p>
 *         A class which represents a graph of geographic locations
 *         Nodes in the graph are intersections between
 */
public class MapGraph {
    //Add your member variables here in WEEK 2

    //each point has link to MapNode by this Map
    private Map<GeographicPoint, MapNode> nodes;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        //Implement in this constructor in WEEK 2
        nodes = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //Implement this method in WEEK 2
        return nodes.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        //Implement this method in WEEK 2
        return nodes.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //Implement this method in WEEK 2
        int numEdges = 0;
        for (Map.Entry<GeographicPoint, MapNode> entry : nodes.entrySet()) {
            numEdges += entry.getValue().getMapEdges().size();
        }

        return numEdges;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // Implement this method in WEEK 2
        if (location != null && !nodes.containsKey(location)) {
            nodes.put(location, new MapNode(location));
            return true;
        }

        return false;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        //Implement this method in WEEK 2
        MapNode startNode = nodes.get(from);
        MapNode endNode = nodes.get(to);

        startNode.addEdge(startNode, endNode, roadName, roadType, length);
        nodes.put(from, startNode);
    }


    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        //Implement this method in WEEK 2
        boolean isFound = false;
        Queue<GeographicPoint> queue = new LinkedList<>();
        Set<GeographicPoint> visited = new HashSet<>();
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
        //add start point to queue
        queue.add(start);
        while (!queue.isEmpty()) {
            GeographicPoint current = queue.remove();
            if (current.equals(goal)) {
                isFound = true;
                break;
            }
            List<MapNode> neigbours = nodes.get(current).getNeighbours();
            for (MapNode pt : neigbours) {
                if (!visited.contains(pt)) {
                    queue.add(pt.getLocation());
                    parentMap.put(pt.getLocation(), current);
                }
            }
            visited.add(current);
        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
        if (!isFound) {
            System.out.println("No path exists");
            return null;
        }

        //build path in List


        return getPath(start, goal, parentMap);
    }

    /**
     * Returns path from start point to goal point
     *
     * @param start     The starting location
     * @param goal      The goal location
     * @param parentMap Map where point links to another point
     * @return list with point which are path's points
     */
    private List<GeographicPoint> getPath(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
        List<GeographicPoint> path = new LinkedList<>();
        GeographicPoint curr = goal;
        path.add(curr);
        while (curr != start) {
            curr = parentMap.get(curr);
            path.add(curr);
        }
        //reverse path
        Collections.reverse(path);

        return path;
    }


    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        int i = 0;
        boolean isFound = false;
        //lets work with DijkastraPoint objects
        MapPoint sPoint = new DijkastraPoint(nodes.get(start));
        MapPoint gPoint = new DijkastraPoint(nodes.get(goal));

        Queue<MapPoint> priorityQueue = new PriorityQueue<>();
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
        Set<MapPoint> visited = new HashSet<>();

        priorityQueue.add(sPoint);
        while (!priorityQueue.isEmpty()) {
            MapPoint curr = priorityQueue.poll();
            if (curr != null && !checkVisited(curr, visited)) {
//                    System.out.println("Update");
                updateParentMap(curr, parentMap);
            }
            nodeSearched.accept(curr.getMapNode().getLocation());
//            GeographicPoint ps = new GeographicPoint(32.869423, -117.220917);
//            if (curr.getMapNode().getLocation().equals(ps)) {
//                System.out.println("Found in ");
//            }
            i++;
            //check if current object isn't in Visited
            if (!visited.contains(curr)) {
                visited.add(curr);
                //check if current equals goal
                if (curr.equals(gPoint)) {
                    System.out.println("Distance: " + curr.getDistance());
                    isFound = true;
                    updateParentMap(curr, parentMap);
                    break;
                }
                //get neigbours of DijkastraPoint
                List<MapPoint> neigbours = getNeigboursMapPoints(curr, null);
                for (MapPoint mapPoint : neigbours) {
//                    GeographicPoint p = new GeographicPoint(32.869423, -117.220917);
//                    if (mapPoint.getMapNode().getLocation().equals(p)) {
//                        System.out.println("Found in neigbours");
//                    }
                    //check if point was visited before
                    if (!checkVisited(mapPoint, visited)) {
                        //Get MapNode objects
                        MapNode currNode = curr.getMapNode();
                        MapNode node = mapPoint.getMapNode();

                        //get edges of current node
                        List<MapEdge> edges = currNode.getMapEdges();

                        //find appropriate edge
                        MapEdge mapEdge = new MapEdge();
                        for (MapEdge edge : edges) {
                            if (edge.getEndNode().equals(node)) {
                                mapEdge = edge;
                            }
                        }

                        mapPoint.setDistance(curr.getDistance() + mapEdge.getLength());
                        mapPoint.setPreviousNode(currNode);
                        priorityQueue.add(mapPoint);
                    }
                }

                //get DijkastraPoint woth High Priority, with least distance

                MapPoint point = priorityQueue.peek();
            }
        }

        if (!isFound) {
            System.out.println("No path exists");
            return null;
        }

        // Hook for visualization.  See writeup.
//        nodeSearched.accept(next.getLocation());
        System.out.println("Dijkstra - " + i);
        return getPathDijkastra(start, goal, parentMap);
    }

    /**
     * Return true if point has been visted and false if not
     *
     * @param point   DijkastraPoint which checks
     * @param visited Set of visited DijkastraPoint
     * @return boolean
     */
    private boolean checkVisited(MapPoint point, Set<MapPoint> visited) {
        boolean isVisited = false;
        for (MapPoint p : visited) {
            if (point.equals(p)) {
                isVisited = true;
            }
        }

        return isVisited;
    }

    /**
     * Mathod is used for getting path for Dijkastra algorithm
     *
     * @param start     GeographicPoint start
     * @param goal      GeographicPoint goal
     * @param parentMap Map with GeographicPoints
     * @return path
     */
    private List<GeographicPoint> getPathDijkastra(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
        List<GeographicPoint> result = new ArrayList<>();
        result.add(goal);
        GeographicPoint curr = parentMap.get(goal);
        while (!curr.equals(start)) {
            result.add(curr);
            curr = parentMap.get(curr);
        }
        result.add(curr);

        Collections.reverse(result);

        return result;
    }

    /**
     * Method updates parent map which saves ling between GeographicPoints
     *
     * @param point
     * @param parentMap
     */
    private void updateParentMap(MapPoint point, Map<GeographicPoint, GeographicPoint> parentMap) {
        try {
            if (point.getPreviousNode() != null) {
                GeographicPoint previousPoint = point.getPreviousNode().getLocation();
                GeographicPoint currPoint = point.getMapNode().getLocation();
                parentMap.put(currPoint, previousPoint);
            }
        } catch (Exception e) {
            System.out.println(point.getDistance());
        }

    }


    /**
     * Returns list of DijkastraPoint which are neigbours to DijkastraPoint curr
     *
     * @param curr DijkastraPoint for which method returns neigbours
     * @return List of Neigbours
     */
    private List<MapPoint> getNeigboursMapPoints(MapPoint curr, MapNode goalNode) {
        List<MapPoint> mapPoints = new ArrayList<>();
        List<MapNode> neig = curr.getMapNode().getNeighbours();
        for (MapNode gp : neig) {
            MapPoint point;
            if (curr instanceof DijkastraPoint) {
                point = new DijkastraPoint(gp);
            } else {
                point = new AStarPoint(gp, goalNode);
            }
            point.setPreviousNode(curr.getPreviousNode());
            mapPoints.add(point);
        }
        return mapPoints;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        int i = 0;
        boolean isFound = false;
        //lets work with DijkastraPoint objects
        MapPoint startNode = new AStarPoint(nodes.get(start), nodes.get(goal));
        MapPoint goalNode = new AStarPoint(nodes.get(goal), nodes.get(goal));

        Queue<MapPoint> priorityQueue = new PriorityQueue<>();
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
        Set<MapPoint> visited = new HashSet<>();

        priorityQueue.add(startNode);
        while (!priorityQueue.isEmpty()) {
            MapPoint curr = priorityQueue.poll();
            if (curr != null && !checkVisited(curr, visited)) {
                updateParentMap(curr, parentMap);
            }
            nodeSearched.accept(curr.getMapNode().getLocation());
            i++;
            //check if current object isn't in Visited
            if (!visited.contains(curr)) {
                visited.add(curr);
                //check if current equals goal
                if (curr.equals(goalNode)) {
                    System.out.println("Distance: " + curr.getDistance());
                    isFound = true;
                    updateParentMap(curr, parentMap);
                    break;
                }
                //get neigbours of DijkastraPoint
                List<MapPoint> neigbours = getNeigboursMapPoints(curr, nodes.get(goal));
                for (MapPoint mapPoint : neigbours) {
                    //check if point was visited before
                    if (!checkVisited(mapPoint, visited)) {
                        //Get MapNode objects
                        MapNode currNode = curr.getMapNode();
                        MapNode node = mapPoint.getMapNode();

                        //get edges of current node
                        List<MapEdge> edges = currNode.getMapEdges();

                        //find appropriate edge
                        MapEdge mapEdge = new MapEdge();
                        for (MapEdge edge : edges) {
                            if (edge.getEndNode().equals(node)) {
                                mapEdge = edge;
                            }
                        }

                        mapPoint.setDistance(curr.getDistance() + mapEdge.getLength());
                        mapPoint.setPreviousNode(currNode);
                        priorityQueue.add(mapPoint);
                    }
                }

                //get DijkastraPoint woth High Priority, with least distance
                MapPoint point = priorityQueue.peek();

                //check if point is not visited before.
            }
        }

        if (!isFound) {
            System.out.println("No path exists");
            return null;
        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
        System.out.println("A star search - " + i);
        return getPathDijkastra(start, goal, parentMap);
        // Hook for visualization.  See writeup.

    }


    public Map<GeographicPoint, MapNode> getNodes() {
        return nodes;
    }

    public static void main(String[] args) {
//        System.out.print("Making a new map...");
//        MapGraph theMap = new MapGraph();
//        System.out.print("DONE. \nLoading the map...");
//        System.out.println("DONE.");
//        for (Map.Entry<GeographicPoint, MapNode> entry : theMap.getNodes().entrySet()) {
//            System.out.println(entry.getValue().getNeighbours().size());
//        }

        // You can use this method for testing.

//		 Use this code in Week 3 End of Week Quiz
//		MapGraph theMap = new MapGraph();
//        System.out.print("DONE. \nLoading the map...");
//        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//        System.out.println("DONE.");
//
//        GeographicPoint start = new GeographicPoint(1, 1);
//        GeographicPoint end = new GeographicPoint(8, -1);
//
//        List<GeographicPoint> route = theMap.dijkstra(start, end);
//        for (GeographicPoint gp : route) {
//            System.out.println(gp.toString());
//        }
//        List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
//        for (GeographicPoint gp : route2) {
//            System.out.println(gp.toString());
//        }

        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route = theMap.dijkstra(start, end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

    }

}
