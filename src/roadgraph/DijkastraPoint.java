package roadgraph;

/**
 * Container for current MapNode, previos MapNode and distance.
 *
 * @author NickVeremeichyk
 * @since 2016-01-10.
 */
public class DijkastraPoint extends MapPoint {


    public DijkastraPoint(MapNode mapNode) {
        super(mapNode);
    }

    /**
     * @param o
     * @return
     */
    @Override
    public int compareTo(Object o) {
        return Double.compare(this.getDistance(), ((DijkastraPoint) o).getDistance());
    }
}
