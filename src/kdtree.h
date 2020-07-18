/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// Structure to represent node of kd tree
template<typename PointT>

struct Node
{
  PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

  void insertHelper(Node<PointT>** node, int depth, PointT point, int id) {
    if (*node == NULL) {
      *node = new Node<PointT>(point, id);
    } else if ( point.data[depth % 3] < ((*node)->point.data[depth % 3])) {
      insertHelper(&((*node)->left), depth + 1, point, id);
    } else {
      insertHelper(&((*node)->right), depth + 1, point, id);
    }
  }

  void insert(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    for(uint id = 0; id < cloud->points.size(); id++)
    {
      insertHelper(&root,0,cloud->points[id], id);
    }
  }

  void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids) {
    if (node != NULL) {

      if ((target.x - distanceTol <= node->point.x) &&
          (target.x + distanceTol >= node->point.x) &&
          (target.y - distanceTol <= node->point.y) &&
          (target.y + distanceTol >= node->point.y) &&
          (target.z - distanceTol <= node->point.z) &&
          (target.z + distanceTol >= node->point.z)) {


        float dis=sqrt((node->point.x-target.x)*(node->point.x-target.x)+
                      (node->point.y-target.y)*(node->point.y-target.y)+
                      (node->point.z-target.z)*(node->point.z-target.z));

        if(dis<distanceTol) {
          ids.push_back(node->id);
        }

      }

      if (target.data[depth % 3] - distanceTol <= node->point.data[depth % 3]) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }

      if (target.data[depth % 3] + distanceTol >= node->point.data[depth % 3]) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
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




