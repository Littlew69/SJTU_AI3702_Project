#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Astar.h"
#include <queue>
#include <map>
#include <algorithm>

using namespace cv;
using namespace std;
MapParamNode MapParam;
Mat Maptest;
void World2MapGrid(MapParamNode& MapParam, Point2d& src_point, Point& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    Mat P_dst = MapParam.Rotation.inv() * (P_src - MapParam.Translation);

    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = MapParam.height - 1 - round(P_dst.at<double>(1, 0));
}
void MapGrid2world(MapParamNode& MapParam, Point& src_point, Point2d& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, MapParam.height - 1 - src_point.y), CV_64FC1);

    Mat P_dst = MapParam.Rotation * P_src + MapParam.Translation;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void MapCallback(const nav_msgs::OccupancyGrid& msg)
{

    // Get the parameters of map
    MapParam.resolution = msg.info.resolution;
    MapParam.height = msg.info.height;
    MapParam.width = msg.info.width;
    // The origin of the MapGrid is on the bottom left corner of the map
    MapParam.x = msg.info.origin.position.x;
    MapParam.y = msg.info.origin.position.y;


    // Calculate the pose of map with respect to the world of rviz
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta = yaw;

    //从rviz上所给定的起点和终点坐标是真实世界坐标系下的位置，需要转化为地图坐标下的表示
    //MapParam.Rotation MapParam.Translation 用于该变换
    MapParam.Rotation = Mat::zeros(2,2, CV_64FC1);
    MapParam.Rotation.at<double>(0, 0) = MapParam.resolution * cos(theta);
    MapParam.Rotation.at<double>(0, 1) = MapParam.resolution * sin(-theta);
    MapParam.Rotation.at<double>(1, 0) = MapParam.resolution * sin(theta);
    MapParam.Rotation.at<double>(1, 1) = MapParam.resolution * cos(theta);
    MapParam.Translation = Mat(Vec2d(MapParam.x, MapParam.y), CV_64FC1);

    cout<<"Map:"<<endl;
    cout<<"MapParam.height:"<<MapParam.height<<endl;
    cout<<"MapParam.width:"<<MapParam.width<<endl;

    Mat Map(MapParam.height, MapParam.width, CV_8UC1);
    Maptest= Map;
    int GridFlag;
    for(int i = 0; i < MapParam.height; i++)
    {
        for(int j = 0; j < MapParam.width; j++)
        {
            GridFlag = msg.data[i * MapParam.width + j];
            GridFlag = (GridFlag < 0) ? 100 : GridFlag; // set Unknown to 0
            Map.at<uchar>(j,MapParam.height-i-1) = 255 - round(GridFlag * 255.0 / 100.0);
        }
    }
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.StartPoint);
    cout<<"StartPoint:"<<MapParam.StartPoint<<endl;
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.TargetPoint);
    int p =Maptest.at<uchar>(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
    cout<<"flag:"<<p<<endl;
    MapGrid2world(MapParam,MapParam.TargetPoint,src_point);
    cout<<"TargetPoint world:"<<src_point<<endl;
    cout<<"TargetPoint:"<<MapParam.TargetPoint<<endl;
}
void PathGrid2world(MapParamNode& MapParam, vector<Point>& PathList, nav_msgs::Path& plan_path)
{
    plan_path.header.stamp = ros::Time::now();
    plan_path.header.frame_id = "map";
    plan_path.poses.clear();
    for(int i=0;i<PathList.size();i++)
    {
        Point2d dst_point;
        MapGrid2world(MapParam,PathList[i], dst_point);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        plan_path.poses.push_back(pose_stamped);
    }
}


struct AStarPoint {
    int x;
    int y;
    AStarPoint(int _x, int _y) : x(_x), y(_y) {}
};

// A*算法中的节点
struct AStarNode {
    Point point;
    AStarNode* parent;
    int f, g, h;

    AStarNode(Point _point, AStarNode* _parent = nullptr)
        : point(_point), parent(_parent), f(0), g(0), h(0) {}

    int distanceTo(AStarNode* other) {
        int dx = abs(point.x - other->point.x);
        int dy = abs(point.y - other->point.y);
        return dx + dy;
    }
};

vector<Point> findPath(Point start, Point end, Mat& map) {
    vector<Point> path;

    // 节点是否在开放列表或关闭列表中
    auto isInList = [](AStarNode* node, vector<AStarNode*>& list) {
        return find_if(list.begin(), list.end(), [=](AStarNode* n) { return n->point.x == node->point.x && n->point.y == node->point.y; }) != list.end();
    };

    auto removeNodeFromList = [](AStarNode* node, vector<AStarNode*>& list) {
        list.erase(remove_if(list.begin(), list.end(), [=](AStarNode* n) { return n->point.x == node->point.x && n->point.y == node->point.y; }), list.end());
    };

    // 获取地图中指定坐标的值
    auto getValueAt = [&](int x, int y) {
        return map.at<uchar>(y, x);
    };

    // 边界检查
    auto isValidPoint = [&](int x, int y) {
        return x >= 0 && x < map.cols && y >= 0 && y < map.rows;
    };

    // 获取相邻节点
    auto getNeighbours = [&](AStarNode* node) {
        vector<AStarNode*> neighbours;
        int x = node->point.x;
        int y = node->point.y;

        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (isValidPoint(nx, ny) && getValueAt(nx, ny) == 0) {
                neighbours.push_back(new AStarNode(Point(nx, ny), node));
            }
        }

        return neighbours;
    };

    // 启发式函数，估计从当前节点到目标节点的距离
    auto heuristic = [&](AStarNode* a, AStarNode* b) {
        return a->distanceTo(b);
    };

    // 开始A*算法
    vector<AStarNode*> openList;
    vector<AStarNode*> closedList;

    AStarNode* startNode = new AStarNode(start);
    AStarNode* endNode = new AStarNode(end);

    openList.push_back(startNode);

    while (!openList.empty()) {
        AStarNode* currentNode = openList[0];
        int currentIndex = 0;

        for (int i = 1; i < openList.size(); ++i) {
            if (openList[i]->f < currentNode->f) {
                currentNode = openList[i];
                currentIndex = i;
            }
        }

        openList.erase(openList.begin() + currentIndex);
        closedList.push_back(currentNode);

        // 找到路径
        if (currentNode->point.x == endNode->point.x && currentNode->point.y == endNode->point.y) {
            while (currentNode != nullptr) {
                path.push_back(currentNode->point);
                currentNode = currentNode->parent;
            }
            break;
        }

        // 获取当前节点的邻居节点
        auto neighbours = getNeighbours(currentNode);
        for (auto neighbour : neighbours) {
            if (!isInList(neighbour, closedList)) {
                int tentativeG = currentNode->g + 1; // 这里简化了距离的计算，实际中可能需要考虑斜线移动的距离

                if (!isInList(neighbour, openList) || tentativeG < neighbour->g) {
                    neighbour->g = tentativeG;
                    neighbour->h = heuristic(neighbour, endNode);
                    neighbour->f = neighbour->g + neighbour->h;
                    neighbour->parent = currentNode;

                    if (!isInList(neighbour, openList)) {
                        openList.push_back(neighbour);
                    }
                }
            }
        }
    }

    // 清理内存
    for (auto node : openList) {
        delete node;
    }
    for (auto node : closedList) {
        delete node;
    }

    reverse(path.begin(), path.end()); // 逆转路径，使其从起点到终点
    
    vector<Point> pathlist;
    for (auto node : path){
        
    }
    return path;
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;

 
    geometry_msgs::PointStamped astar_step;

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Rate loop_rate(20);
    nav_msgs::Path plan_path;
    
    vector<Point> PathList;
    
    while(ros::ok())
    {
        AStarPoint start = AStarPoint(MapParam.StartPoint.x, MapParam.StartPoint.y);
        AStarPoint end = AStarPoint(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
        PathList = findPath(MapParam.StartPoint, MapParam.TargetPoint, Maptest);
        //cout << PathList << endl;

        PathGrid2world(MapParam, PathList,plan_path);
        path_pub.publish(plan_path);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
