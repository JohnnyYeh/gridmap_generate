//write to test 202097
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

#define inf 1>>20


struct GridNode
{     
   int id;        // 1--> open set, -1 --> closed set
   Eigen::Vector3d coord;
   Eigen::Vector3i index;
   
   uint8_t * occupancy; 

   GridNode(Eigen::Vector3i _index)
   {  
      id = 0;
      index = _index;
      
   }

   GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
   {  
      id = 0;
      index = _index;
      coord = _coord;
   }

   GridNode(){};
   
   ~GridNode(){};
};

typedef GridNode* GridNodePtr;
GridNodePtr *** GridNodeMap;

uint8_t * data;

//关于地图的一些参数
int GLX_SIZE;
int GLY_SIZE;
int GLZ_SIZE;
int GLYZ_SIZE;
int GLXYZ_SIZE;
int tmp_id_x, tmp_id_y, tmp_id_z;

double _x_size = 20.0; //要生成的地图大小 20*20*3 单位为m
double _y_size = 20.0;
double _z_size = 3.0 ;
double  _resolution =  0.15 ;
double  _cloud_margin = 0.3;
double _inv_resolution;
double gl_xl, gl_yl, gl_zl;
double gl_xu, gl_yu, gl_zu;

bool _has_map = false;

ros::Publisher _vis_grid_path_pub, _vis_inf_map_pub;
Eigen::Vector3d _map_lower,_map_upper;

Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d pt) 
{
    Eigen::Vector3i idx;
    idx <<  std::min( std::max( int( (pt(0) - gl_xl) * _inv_resolution), 0), GLX_SIZE - 1),
            std::min( std::max( int( (pt(1) - gl_yl) * _inv_resolution), 0), GLY_SIZE - 1),
            std::min( std::max( int( (pt(2) - gl_zl) * _inv_resolution), 0), GLZ_SIZE - 1);      		    
  
    return idx;
};

Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i index) 
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * _resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + gl_zl;

    return pt;
};

void initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    double resolution = _resolution;
    double inv_resolution = 1.0 / _resolution;    

    data    = new uint8_t[GLXYZ_SIZE];

    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++)
            {
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
                GridNodeMap[i][j][k]->occupancy = & data[i * GLYZ_SIZE + j * GLZ_SIZE + k];
            }
        }
    }
}

bool setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return false;

    tmp_id_x = static_cast<int>( (coord_x - gl_xl) * _inv_resolution);
    tmp_id_y = static_cast<int>( (coord_y - gl_yl) * _inv_resolution);
    tmp_id_z = static_cast<int>( (coord_z - gl_zl) * _inv_resolution);      

    if(data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] == 0){
        data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] = 1;
        return true;
    }
    else{
        return false;
    }
}


void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if( _has_map ) return;
    ROS_WARN("Get map message");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_inf;
    pcl::fromROSMsg(pointcloud_map, cloud);
    sensor_msgs::PointCloud2 map_inflation;
    
    if( (int)cloud.points.size() == 0)
        return;
   
    pcl::PointXYZ pt, pt_inf;
    
    int inf_step   = round(_cloud_margin * _inv_resolution); //这里涉及膨胀地图的大小，resolution 为一个栅格大小，
							    //单位为m,我自己是设置为[0.15 0.30]之间，越大地图越稀疏，_inv_resolution,为resolution的倒数
                                                          
    int inf_step_z = max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;

                    if(isnan(inf_x) || isnan(inf_y) || isnan(inf_z)) continue;

                    Vector3d vec_inf(inf_x, inf_y, inf_z);
                    Vector3i idx_inf = coord2gridIndex(vec_inf);     

                    // set in obstacle points
                    bool flag_newly_occupied = setObs(inf_x, inf_y, inf_z); 
                    
                    // rounding for visualizing the grid map
                    if(flag_newly_occupied) 
                    {
                        Vector3d coor_round = gridIndex2coord( idx_inf ); 
                        pt_inf.x = coor_round(0);
                        pt_inf.y = coor_round(1);
                        pt_inf.z = coor_round(2);
                        cloud_inf.points.push_back(pt_inf);
                    }
                }
            }
        }
    }

    cloud_inf.width    = cloud_inf.points.size();
    cloud_inf.height   = 1;
    cloud_inf.is_dense = true;

    pcl::toROSMsg(cloud_inf, map_inflation);
    map_inflation.header.frame_id = "/map";
    _vis_inf_map_pub.publish(map_inflation);
    _has_map = true;
}


////放在main
int main(int argc,char ** argv)
{
	ros::init(argc,argv,"gridmap_node");
	ros::NodeHandle nh("~");
	
	_vis_inf_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("inflation_map", 10);
	ros::Subscriber _map_sub = nh.subscribe("/cloud_pcd", 2, rcvPointCloudCallBack);
	
	_map_lower << -5.0, -_y_size/2.0, 0.0;
	_map_upper << _x_size - 5.0, +_y_size/2.0, _z_size;
	
	_inv_resolution = 1.0 / _resolution;
	int _max_x_id = (int)(_x_size * _inv_resolution);
	int _max_y_id = (int)(_y_size * _inv_resolution);
	int _max_z_id = (int)(_z_size * _inv_resolution);
							   
	GLX_SIZE = _max_x_id;
	GLY_SIZE = _max_y_id;
	GLZ_SIZE = _max_z_id;
	GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
	GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
    initGridMap(_resolution, _map_lower, _map_upper);//要初始化地图 

	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}






















