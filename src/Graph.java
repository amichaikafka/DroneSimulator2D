import com.mxgraph.layout.mxCircleLayout;
import com.mxgraph.layout.mxIGraphLayout;
import com.mxgraph.swing.mxGraphComponent;
import org.jgrapht.*;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.spanning.*;
import org.jgrapht.ext.JGraphXAdapter;
import org.jgrapht.graph.*;
import org.jgrapht.graph.DefaultEdge;

import javax.swing.*;
import java.util.Set;

public class Graph {

	DefaultDirectedGraph<Point, DefaultEdge> g;
    private DefaultWeightedEdge e1;
    
    public Graph() {
    	g = new DefaultDirectedGraph<Point, DefaultEdge>(DefaultEdge.class);
    }
    public void addVertex(Point name) {
        Point last_vertex = null;
        Set<Point> all = g.vertexSet();
        
        if(all.size() > 0) {
        	last_vertex = getLastElement(all);
        }
        g.addVertex(name);
        if(last_vertex != null) 
        	g.addEdge(name,last_vertex);
        //graph.addVertex(name);
    }
    
    public Point getLastElement(Set<Point> c) {
    	Point last = null;
    	if(c.size() > 0) {
    		for(Point x : c) {
    			last = x;
    		}
    	}
        return last;
    }
    public void addEdge(Point v1,Point v2) {
        g.addEdge(v1, v2);
    }


    public DefaultDirectedGraph<Point, DefaultEdge> getGraph() {
        return g;
    }
    
    public String getOutput() {
    	return g.toString();
    }
    
    public void drawGraph() {
    	JFrame new_window = new JFrame();
    	new_window.setSize(500,500);
    	new_window.setTitle("Graph Viewer");
    	JGraphXAdapter<Point, DefaultEdge> graphAdapter = new JGraphXAdapter<Point, DefaultEdge>(g);

        mxIGraphLayout layout = new mxCircleLayout(graphAdapter);
        layout.execute(graphAdapter.getDefaultParent());

        new_window.add(new mxGraphComponent(graphAdapter));

        new_window.pack();
        new_window.setLocationByPlatform(true);
        new_window.setVisible(true);
    }

    /*public SimpleWeightedGraph<String,DefaultWeightedEdge> getGraph() {
        return graph;
    }*/

    public void getSpanningTree() {
        KruskalMinimumSpanningTree k=new KruskalMinimumSpanningTree(g);
        System.out.println(k.getSpanningTree().toString());
        //KruskalMinimumSpanningTree k1=new KruskalMinimumSpanningTree(graph);
        //System.out.println(k1.getEdgeSet().toString());   
    }
    public void shortestPath(Point s,Point d){
//        System.out.println(g.vertexSet());
//        System.out.println(g.edgeSet());
//        System.out.println(g);
//        System.out.println(s);
//        System.out.println(d);
        var path = new BFSShortestPath(g);
        var p =path.getPath(s,d);
//        System.out.println(p);
//        System.out.println(p.getClass());
//        var path2=new DijkstraShortestPath(g);
//        var p2=path2.getPath(s, d);
//        System.out.println(p2);
//        System.out.println(BFSShortestPath.findPathBetween(g, s, d));
    }

    /*public void getSpanningTreeCost() {
        KruskalMinimumSpanningTree k=new KruskalMinimumSpanningTree(graph);
        System.out.println(k.getSpanningTreeCost());
    }*/ 
}