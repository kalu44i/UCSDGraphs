package roadgraph;

import geography.GeographicPoint;

/**
 * This class represents edge which has start and end points of street, name and length.
 *
 * @author NickVeremeichyk
 * @since 2016-01-03.
 */
public class MapEdge {
    private MapNode startNode;
    private MapNode endNode;
    private String roadName;
    private String roadType;
    private Double length;

    public MapEdge() {
    }

    public MapEdge(MapNode startNode, MapNode endNode, String streetName, String roadType, Double length) {
        this.startNode = startNode;
        this.endNode = endNode;
        this.roadName = streetName;
        this.length = length;
    }

    public void setStartNode(MapNode startNode) {
        this.startNode = startNode;
    }

    public void setEndNode(MapNode endNode) {
        this.endNode = endNode;
    }

    public void setRoadName(String roadName) {
        this.roadName = roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public void setRoadType(String roadType) {
        this.roadType = roadType;
    }

    public void setLength(Double length) {
        this.length = length;
    }

    public MapNode getStartNode() {
        return startNode;
    }

    public MapNode getEndNode() {
        return endNode;
    }

    public String getRoadName() {
        return roadName;
    }

    public Double getLength() {
        return length;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + (int)((startNode != null) ? startNode.hashCode() : 0);
        result = prime * result + (int)((startNode != null) ? endNode.hashCode() : 0);
        result = prime * result + ((roadName != null) ? roadName.hashCode() : 0);
        result = prime * result + ((roadType != null) ? roadType.hashCode() : 0);
        long temp;
        temp = Double.doubleToLongBits(length);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        MapEdge other = (MapEdge) obj;
        if (endNode == null) {
            if (other.endNode != null)
                return false;
        } else if (!endNode.equals(other.endNode))
            return false;
        if (Double.doubleToLongBits(length) != Double.doubleToLongBits(other.length))
            return false;
        if (roadName == null) {
            if (other.roadName != null)
                return false;
        } else if (!roadName.equals(other.roadName))
            return false;
        if (roadType == null) {
            if (other.roadType != null)
                return false;
        } else if (!roadType.equals(other.roadType))
            return false;
        if (startNode == null) {
            if (other.startNode != null)
                return false;
        } else if (!startNode.equals(other.startNode))
            return false;
        return true;

    }


}
