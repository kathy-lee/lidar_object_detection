/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <pcl/common/common.h>


// Structure to represent node of kd tree
//template<typename PointT>
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

//template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(&root, 0, point, id);
	}

    void insertHelper(Node** node, int depth, pcl::PointXYZI point, int id)
	{
		//Tree is empty
		if(*node == NULL)
        	*node = new Node(point, id);
      	else
      	{
			//calculate current dim: 0 or 1
			int d = depth % 2;
            if(d == 0){
                if(point.x < (*node)->point.x) insertHelper(&(*node)->left, depth + 1, point, id);
                else insertHelper(&(*node)->right, depth + 1, point, id);
            }
            else{
                if(point.y < (*node)->point.y) insertHelper(&(*node)->left, depth + 1, point, id);
                else insertHelper(&(*node)->right, depth + 1, point, id);
            }
      	}
	    return;	
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
 
		searchHelper(root, 0, target, distanceTol,ids);

		return ids;
	}
	
	void searchHelper(Node* node, int depth, pcl::PointXYZI target, float distanceTol, std::vector<int>& ids)
    {

        
		if(node != NULL)
		{
			// check if node is within target boundary box
			if((node->point.x >= (target.x-distanceTol)&&(node->point.x <= (target.x+distanceTol)))&&(node->point.y >= (target.y-distanceTol)&&(node->point.y <= (target.y+distanceTol))))
            {
                float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y));
                if(distance <= distanceTol) ids.push_back(node->id);
            }
            if(depth%2 == 0){
                if((target.x - distanceTol) < node->point.x) searchHelper(node->left, depth+1, target, distanceTol, ids);
                if((target.x + distanceTol) > node->point.x) searchHelper(node->right, depth+1, target, distanceTol, ids);
            }
            else{
                if((target.y - distanceTol) < node->point.y) searchHelper(node->left, depth+1, target, distanceTol, ids);
                if((target.y + distanceTol) > node->point.y) searchHelper(node->right, depth+1, target, distanceTol, ids);
            }
		}

	}
};




