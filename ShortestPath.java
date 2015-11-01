package ShortestPathAlgorithm;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

/**
 * @author Sanjay Gandhi 
 *
 */

/*
 * Class to represent Vertex of a graph
 * 
 * @variable name: Vertex name adjList: Adjacency List of the vertex visited:
 * boolean to notify if the vertex is visited
 * 
 */
class Vertex implements Comparable<Vertex>{
	int name;
	LinkedList<Edge> adjList = new LinkedList<Edge>();
	Vertex parent = null;
	int distance = 0;
	int indegree = 0;

	/**
	 * Constructor
	 */
	public Vertex(int name) {
		this.name = name;
	}

	/**
	 * @return the name
	 */
	public int getName() {
		return name;
	}

	@Override
	public int compareTo(Vertex v) {
		return Integer.compare(distance, v.distance);
	}
}

/*
 * Class to represent the edges in the Graph 
 * @variable head   : Head of an edge
 *           tail   : Tail of an edge
 *           weight : Weight of an edge(distance)
 */
class Edge {

	Vertex head;
	Vertex tail;
	int weight;

	/**
	 * Constructor for Edge
	 *
	 * @param u
	 *            : Vertex - The head of the edge
	 * @param v
	 *            : Vertex - The tail of the edge
	 * @param w
	 *            : int - The weight associated with the edge
	 */
	Edge(Vertex u, Vertex v, int w) {
		head = u;
		tail = v;
		weight = w;
	}	
}

/*
 * Class to represent the Graph containing array of Vertices
 * 
 * @variables: V : Array of vertices size: Number of vertices
 */
class Graph {
	/*
	 * Array of Vertices 
	 */
	Vertex V[];

	/*
	 * Size : Number of Vertices in the Graph
	 */
	int size;

	/**
	 * Constructor
	 * 
	 * @param v
	 *            : number of vertices
	 */
	public Graph(int numberOfVertices) {
		size = numberOfVertices;
		V = new Vertex[size];
		for (int i = 0; i < size; i++)
			V[i] = new Vertex(i);
	}


	/**
	 * Method to add an edge to the Graph 
	 * 
	 * @param head 
	 *            : Vertex - head vertex of an edge
	 * @param tail 
	 *            : Vertex - tail vertex of an edge
	 * @param weight
	 *            : int - weight of an edge  
	 */
	public void addEdge(int head, int tail, int weight) {
		Edge e = new Edge(V[head], V[tail], weight);
		V[head].adjList.add(e);
		V[tail].indegree++;
	}

	/**
	 * Method to initialize a graph 
	 * 1) Sets the parent of every vertex as null
	 * 2) Sets the distance of every vertex as infinity
	 */
	void initialize() {
		for (Vertex u : V) {
			u.parent = null;
			u.distance = Integer.MAX_VALUE;
		}
	}

	/**
	 * Method to print the adjacency list of the graph
	 */
	public void printAdjList() {
		System.out.println("ADJACENCY LIST OF THE GRAPH ");
		for (Vertex u : V) {
			System.out.print("Vertex " + u.name + " : ");
			for (Edge e : u.adjList)
				System.out.print(e.tail.name + " ");
			System.out.println();
		}
		System.out.println();
	}

	/**
	 * Method to find the Shortest Path in the directed graph with uniform weights 
	 * by BREADTH-FIRST SEARCH ALGORITHM
	 * @param source :
	 *                 int - Source vertex name           
	 */
	public void bfs(int source) {
		//Initialize the graph
		initialize();

		//Create a boolean array to store the visited status of the vertices of the graph
		boolean[] visited = new boolean[size];

		//Queue to add the vertices as they are visited
		Queue<Vertex> q = new LinkedList<Vertex>();

		//Set the source distance to 0 and visited to true and add to the queue
		V[source].distance = 0;
		visited[source] = true;
		q.add(V[source]);

		//Process the queue
		while (!q.isEmpty()) {
			//Remove the head of the queue
			Vertex u = q.remove();

			//Visit the adjacent vertices and update its parameters and add to the queue if it is not visited before
			for (Edge e : u.adjList) {
				Vertex v = e.tail;
				if (!visited[v.name]) {
					V[v.name].parent = V[u.name];
					V[v.name].distance = V[u.name].distance + e.weight;
					visited[v.name] = true;
					q.add(V[v.name]);					
				}
			}
		}
	}

	/**
	 * Method to calculate Shortest Path in the graph with non-negative weights
	 *  by using DIKSTRA's Algorithm
	 * @param source
	 *              : int - Source Vertex name 
	 */
	public void dijkstra(int source){
		//Initialize the graph
		initialize();

		//Create a Queue
		PriorityQueue<Vertex> Q = new PriorityQueue<Vertex>();

		//Add the source vertex to the queue
		Q.add(V[source]);

		//Set source distance to 0
		V[source].distance = 0;

		//Perform BFS
		while(!Q.isEmpty()){
			//Get the minimum distance vertex
			Vertex u = Q.poll();			

			//Iterate through u's adjList
			for(Edge e : u.adjList){
				//Tail of the edge
				Vertex v = e.tail;

				//If vertex v is not null
				if(v != null){
					// If distance of parent and edge combined is less than
					// distance to node
					if (V[u.name].distance + e.weight < V[v.name].distance) {
						// Remove this node
						Q.remove(V[v.name]);

						// Set the distance of this node
						V[v.name].distance = V[u.name].distance + e.weight;

						// Set parent
						V[v.name].parent = V[u.name];

						// Add this node with updated distance
						Q.add(V[v.name]);
					}
				}
			}
		}
	}

	/**
	 * Method to calculate Shortest Path in the graph with negative edges 
	 *  by using ADAPTIVE Bellman Ford Algorithm
	 * @param source : 
	 *                 int - Source vertex name
	 */
	public void bellmanFord(int source) throws Exception{

		//A int Array to record the number of times the vertices are relaxed 
		int[] count = new int[size];

		//Initialize the graph
		initialize();

		//Set source distance to 0
		V[source].distance = 0;

		//Queue storing the vertex whose distance is changed/updated
		Queue<Vertex> Q = new LinkedList<Vertex>();

		//Add the source to the queue
		Q.add(V[source]);

		//Process till the queue is empty
		while(!Q.isEmpty()){

			//Delete the head of the queue and increment the process count
			Vertex u = Q.remove();			
			count[u.name]++;

			if(count[u.name] == size)
				throw new Exception(" OOH !! Graph has NEGATIVE CYCLE !!! ");

			//Outgoing edges of Vertex u
			for( Edge e : u.adjList ) {
				//Tail of the edge
				Vertex v = e.tail;
				//Relax the edge and update the boolean to true if relaxed
				if(V[v.name].distance > V[u.name].distance + e.weight) {
					V[v.name].distance = V[u.name].distance + e.weight;
					V[v.name].parent = V[u.name];
					if(!Q.contains(V[v.name]))
						Q.add(V[v.name]);
				}
			}
		}		
	}

	/**
	 * Method to calculate Shortest Path Distance for directed Acyclic Graph
	 * @param sorted : Array of Topologically sorted order of vertices
	 * @param source : int - Source vertex name
	 * 
	 */
	public void DAG(ArrayList<Vertex> sorted, int source){
		//Initialize the graph
		initialize();	

		//Set the source vertex distance to be 0
		V[source].distance = 0;

		//Process the list in order
		while(!sorted.isEmpty()){
			Vertex u = sorted.remove(0);
			//Relax the edges
			for(Edge e : u.adjList){
				Vertex v = e.tail;				
				if(V[v.name].distance > V[u.name].distance + e.weight && V[v.name].parent != null){
					V[v.name].distance = V[u.name].distance + e.weight;
					V[v.name].parent = V[u.name];
				}				
			}
		}
	}

	/**
	 * Method to sort the vertices of the graph topologically
	 */
	public ArrayList<Vertex> top_sort(){
		//Create an array storing the indegree of the vertices
		int[] indegree = new int[size];
		for(int i=0;i<size;i++)
			indegree[i] = V[i].indegree;

		//Queue to store the vertices whose indegree is zero
		Queue<Vertex> Q = new LinkedList<Vertex>();

		//ArrayList to store the vertices in topologically sorted order
		ArrayList<Vertex> top_sort = new ArrayList<Vertex>();

		//Add vertices whose indegree is zero
		for(Vertex u : V)
			if(V[u.name].indegree == 0 && u!=null)
				Q.add(V[u.name]);

		//Process the queue
		while(!Q.isEmpty()){
			//Remove the head of the queue and add it to the array
			Vertex u = Q.remove();
			top_sort.add(V[u.name]);

			//Reduce the indegree of the adjacent nodes and add to the queue if the indegree is zero
			for(Edge e : V[u.name].adjList){
				Vertex v = e.tail;
				V[v.name].indegree--;
				if(V[v.name].indegree == 0)
					Q.add(V[v.name]);
			}
		}

		//Return null if the array doesn't contain all the vertices implying CYCLE in the graph
		// else return the sorted array
		if(top_sort.size() != size)
			return null;
		return top_sort;

	}

	/**
	 * Method to print the edges involved in the shortest path 
	 * from source vertex u to the destination vertex v
	 * 
	 * @param source
	 *               : int - name of the source Vertex
	 *               
	 * @param dest
	 *               : Vertex - Destination vertex
	 */
	public void printEdgesInvolved(Vertex dest, int source){
		/*
		 * Recursively call until it reaches the source vertex 
		 * or is reachable form the source vertex
		 */
		if(dest.name != source && dest.parent != null){
			printEdgesInvolved(dest.parent, source);
			System.out.print(" ("+dest.parent.name+","+dest.name+")");
		}

	}

	/**
	 * Method to print the shortest distance of each vertex from the source vertex
	 * @param source
	 *               : int - name of the source vertex
	 */
	public void printResult(int source){

		System.out.println("SHORTEST PATH DISTANCE FROM SOURCE VERTEX "+ V[source].name+" TO : ");
		for(Vertex u : V ){
			/*
			 * If the vertex is not reachable from the source vertex,
			 * print Distance as - and Edges Involved : -
			 * 
			 */
			if(u.distance == Integer.MAX_VALUE)
				System.out.println("Vertex "+u.name+" : - \t Edges Involved : -");
			else
			{
				System.out.print("Vertex "+u.name+" : "+ u.distance+"\t");
				System.out.print(" Edges Involved :");
				printEdgesInvolved(u, source);
				System.out.println();
			}			
		}
	}

	/**
	 * Method to print the shortest path distance and parent of all vertices from source if size of the graph is less than 100
	 */
	public void printAll(){
		for(Vertex u : V){
			if(u.distance != Integer.MAX_VALUE || u.parent != null)
				System.out.println(u.distance + " " + u.parent.name);
			else
				System.out.println("INF  - ");
		}
	}
}

public class ShortestPath {
	public static void main (String[] args) throws FileNotFoundException {

		Scanner in;
		if(args.length > 0) {
			File inputFile = new File(args[0]);
			in = new Scanner(inputFile);
		}
		else
			in = new Scanner(System.in);

		//Get the number of vertices in the graph
		int numberOfVertices = in.nextInt();
		Graph G = new Graph(numberOfVertices);
		//Get the number of edges in the graph
		int numberOfEdges = in.nextInt();

		//Get the source Vertex 
		int source = in.nextInt();

		//Get the destination vertex
		int destination = in.nextInt();

		//Declare the boolean variables
		boolean uniformWeights = true;
		boolean nonnegativeWeights = false;

		//Get the first edge 
		int head = in.nextInt();
		int tail = in.nextInt();
		int weight = in.nextInt();
		//Add the first edge to the graph
		G.addEdge(head, tail, weight);
		int sample = weight;

		//Check for negative edges
		if(weight<0)
			nonnegativeWeights = true;

		//Get the remaining edges
		for(int i =1;i<numberOfEdges;i++){
			head = in.nextInt();
			tail = in.nextInt();
			weight = in.nextInt();

			//Add the edges to the graph
			G.addEdge(head, tail, weight);

			//Check for uniform weights
			if(weight != sample)
				uniformWeights = false;

			//Check for negative edges
			if(weight<0)
				nonnegativeWeights = true;		
		}

		in.close();

		//		//Print the Adjacency List of the Graph
		//		G.printAdjList();

		long start = System.currentTimeMillis();

		/*
		 * SHORTEST PATH ALGORITHM SELECTION AND DISPLAYING THE OUTPUT
		 */
		if(uniformWeights){
			G.bfs(source);
			long end = System.currentTimeMillis();
			if(G.V[destination].distance != Integer.MAX_VALUE)
				System.out.println("BFS "+ G.V[destination].distance + " " + G.V[destination].parent + " " + (end-start));
			else
				System.out.println("BFS INF - " + (end-start));
			if(numberOfVertices < 100)
				G.printAll();				
		}
		else{
			//Topologically sort the graph vertices
			ArrayList<Vertex> sorted = G.top_sort();
			if(sorted != null){
				G.DAG(sorted, source);
				long end = System.currentTimeMillis();
				if(G.V[destination].distance != Integer.MAX_VALUE)
					System.out.println("DAG "+ G.V[destination].distance + " " + G.V[destination].parent + " " + (end-start));
				else
					System.out.println("DAG INF - " + (end-start));
				if(numberOfVertices < 100)
					G.printAll();
			}
			else if(nonnegativeWeights){
				G.dijkstra(source);
				long end = System.currentTimeMillis();
				if(G.V[destination].distance != Integer.MAX_VALUE)
					System.out.println("DIJ "+ G.V[destination].distance + " " + G.V[destination].parent + " " + (end-start));
				else
					System.out.println("DIJ INF - " + (end-start));
				if(numberOfVertices < 100)
					G.printAll();
			}
			else {
				try{
					G.bellmanFord(source);
					long end = System.currentTimeMillis();
					if(G.V[destination].distance != Integer.MAX_VALUE)
						System.out.println("B-F "+ G.V[destination].distance + " " + G.V[destination].parent + " " + (end-start));
					else
						System.out.println("B-F INF - " + (end-start));
					if(numberOfVertices < 100)
						G.printAll();
				}catch(Exception e){
					System.out.println(e.getMessage());
				}
			}
		}
	}
}
