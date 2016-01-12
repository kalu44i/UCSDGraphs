package roadgraph;

/**
 * @author NickVeremeichyk
 * @since 2016-01-11.
 */
public class AStarPoint extends MapPoint {

    public AStarPoint(MapNode mapNode, MapNode goalNode) {
        super(mapNode);
        this.setDistToGoal(getDistToGoal(mapNode, goalNode));
        this.setDistanceSum(getDistance() + getDistToGoal());
    }

    /**
     * @param o
     * @return
     */
    @Override
    public int compareTo(Object o) {
        return Double.compare(this.getDistanceSum(), ((AStarPoint) o).getDistanceSum());
    }
}
