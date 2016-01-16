package roadgraph;

import geography.GeographicPoint;

import java.util.Objects;

/**
 * @author NickVeremeichyk
 * @since 2016-01-11.
 */
public abstract class MapPoint implements Comparable {
    private MapNode mapNode;
    private MapNode previousNode;
    private Double distanceSum;
    private Double distance;
    private Double distToGoal;

    public MapPoint(MapNode mapNode) {
        this.mapNode = mapNode;
        this.distance = 0.0;
        this.previousNode = null;
    }


    public MapNode getMapNode() {
        return mapNode;
    }

    public void setMapNode(MapNode mapNode) {
        this.mapNode = mapNode;
    }

    public MapNode getPreviousNode() {
        return previousNode;
    }

    public void setPreviousNode(MapNode previousNode) {
        this.previousNode = previousNode;
    }

    public Double getDistanceSum() {
        return distanceSum;
    }

    public void setDistanceSum(Double distanceSum) {
        this.distanceSum = distanceSum;
    }

    public Double getDistance() {
        return distance;
    }

    public void setDistance(Double distance) {
        this.distance = distance;
    }

    public Double getDistToGoal() {
        return distToGoal;
    }

    public void setDistToGoal(Double distToGoal) {
        this.distToGoal = distToGoal;
    }

    /**
     *
     * @param currentNode
     * @param goalNode
     * @return
     */
    public Double getDistToGoal(MapNode currentNode, MapNode goalNode) {
        GeographicPoint currPoint = currentNode.getLocation();
        GeographicPoint goalPoint = goalNode.getLocation();
        return currPoint.distance(goalPoint);
    }


    @Override
    public boolean equals(Object obj) {
        return Objects.equals(mapNode, ((MapPoint) obj).getMapNode());
    }

    @Override
    public int hashCode() {
        int prime = 31;
        int result = 1;
        result = result * prime + mapNode.hashCode();
        result = (int) (result * prime + +distance);
        result = result * prime + +((previousNode != null) ? previousNode.hashCode() : 0);
        return result;
    }

    @Override
    public abstract int compareTo(Object o);
}
