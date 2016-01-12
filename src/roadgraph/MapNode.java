package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * The class represents mapEdges for a GeographicPoint.
 * Each MapNode based on one GeographicPoint. (a GeographicPoint of a MapNode)
 *
 * @author NickVeremeichyk
 * @since 2016-01-03.
 */
public class MapNode {
    //list of mapEdges
    private List<MapEdge> mapEdges;
    private GeographicPoint location;

    public GeographicPoint getLocation() {
        return location;
    }


    public MapNode(GeographicPoint location) {
        this.location = location;
        mapEdges = new ArrayList<>();
    }

    /**
     * Method adds new edge to MapNode with start in GeographicPoint of the MapNode
     *
     * @param startPoint start of edge and it equals to GeographicPoint of MapNode.
     * @param endPoint   end of edge
     * @param streetName name of street
     * @param length     lehgth of street
     */
    public void addEdge(MapNode startPoint, MapNode endPoint, String streetName, String roadType, Double length) {
        mapEdges.add(new MapEdge(startPoint, endPoint, streetName, roadType, length));
    }

    /**
     * @return list of mapEdges
     */
    public List<MapEdge> getMapEdges() {
        return mapEdges;
    }

    /**
     * Returns Neighbours of the MapNode
     *
     * @return list of geographic points
     */
    public List<MapNode> getNeighbours() {
        List<MapNode> neigbours = new LinkedList<>();
        for (MapEdge mapEdge : mapEdges) {
            neigbours.add(mapEdge.getEndNode());
        }
        return neigbours;
    }

    /**
     *
     * @return
     */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + (int)((location != null) ? location.getX() : 0);
        result = prime * result + (int)((location != null) ? location.getY() : 0);
        return result;
    }

    /**
     *
     * @param obj
     * @return
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        MapNode other = (MapNode) obj;
        if (location == null) {
            if (other.location != null)
                return false;
        } else if (!location.equals(other.location))
            return false;
        return true;
    }
}
