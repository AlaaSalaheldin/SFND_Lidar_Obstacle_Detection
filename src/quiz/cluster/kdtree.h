/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node<PointT>(PointT p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree<PointT>()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
	{
		// tree is empty
		if(*node==NULL)
			*node = new Node<PointT>(point,id);
		else
		{
			// calculate current dim
			uint cd = depth%2;

			if(cd == 0)	// use x
			{
				if(point.x < ((*node)->point.x))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
					insertHelper(&((*node)->right), depth+1, point, id);
			}
			else	// use y
			{
				if(point.y < ((*node)->point.y))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
					insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			//TODO: add z
			float nx = node->point.x;
			float ny = node->point.y;
			float tx = target.x;
			float ty = target.y;
			if ((nx >= (tx - distanceTol)) && (nx <= (tx + distanceTol)) &&
			    (ny >= (ty - distanceTol)) && (ny <= (ty + distanceTol)))
			{
				// ensure node is inside the circle
				float distance = sqrt((nx-tx)*(nx-tx) + (ny-ty)*(ny-ty));
				if(distance <= distanceTol)
					ids.push_back(node->id);
            }

			// continue with next node
			uint cd = depth % 2;
			if(cd == 0)	// use x dimension
			{
				if((target.x-distanceTol)<node->point.x)
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				if((target.x+distanceTol)>node->point.x)
					searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			else
			{
				if((target.y-distanceTol)<node->point.y)
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				if((target.y+distanceTol)>node->point.y)
					searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

};

template<typename PointT>
inline void proximity(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.push_back(idx);

	std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			proximity(id, cloud, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
inline std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size())
	{
		if(!processed[i])
		{
			std::vector<int> cluster;
			proximity<PointT>(i, cloud, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
		
		i++;
	}

	return clusters;
}




