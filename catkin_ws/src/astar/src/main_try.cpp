#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Astar.h"

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

// 定义节点结构体
struct AStarNode {
    Point pos; // 节点的坐标
    int f, g, h; // f = g + h，分别表示总代价、实际代价、启发式代价
    AStarNode* parent; // 父节点指针

    AStarNode(Point pos) : pos(pos), f(0), g(0), h(0), parent(nullptr) {}

    // 计算启发式代价（曼哈顿距离）
    int calculateH(AStarNode* goal) {
        return abs(pos.x - goal->pos.x) + abs(pos.y - goal->pos.y);
    }

    // 计算总代价
    void calculateF() {
        f = g + h;
    }
};

// 定义比较函数用于优先队列
struct CompareNode {
    bool operator()(const AStarNode* a, const AStarNode* b) {
        return a->f > b->f;
    }
};

// A*算法
void AStar(AStarNode* start, AStarNode* goal, const Mat& grid, vector<Point>& PathList, int expansionRange=1) {
    priority_queue<AStarNode*, vector<AStarNode*>, CompareNode> openList;
    vector<vector<bool>> closedList(grid.cols, vector<bool>(grid.rows, false));

    openList.push(start);
    Mat paddedGrid = Mat::zeros(grid.rows + 2, grid.cols + 2, CV_8UC1);
    grid.copyTo(paddedGrid(Rect(1, 1, grid.cols, grid.rows)));

    while (!openList.empty()) {
        AStarNode* current = openList.top();
        openList.pop();

        if (current->pos == goal->pos) {
            // 找到了目标节点，回溯路径
            while (current != nullptr) {
                PathList.push_back(current->pos);
                current = current->parent;
            }
            reverse(PathList.begin(), PathList.end());
            break;
        }

        // 标记当前节点为已访问
        closedList[current->pos.x][current->pos.y] = true;

        // 遍历当前节点的邻居节点
        int dx[] = {-1, 0, 1, 0};
        int dy[] = {0, -1, 0, 1};
        for (int i = 0; i < 4; ++i) {
            int newX = current->pos.x + dx[i];
            int newY = current->pos.y + dy[i];

            // 检查邻居节点是否有效
            if (newX >= 0 && newX < grid.cols && newY >= 0 && newY < grid.rows &&
                grid.at<uchar>(newY, newX) == 0 && !closedList[newX][newY]) {
                // 检查是否需要膨胀
                bool canMove = true;
                for (int j = -expansionRange; j <= expansionRange; ++j) {
                    for (int k = -expansionRange; k <= expansionRange; ++k) {
                        int expandedX = newX + j;
                        int expandedY = newY + k;
                        if (expandedX >= 0 && expandedX < grid.cols && expandedY >= 0 && expandedY < grid.rows &&
                            grid.at<uchar>(expandedY, expandedX) != 0) {
                            canMove = false;
                            break;
                        }
                    }
                }

                if (canMove) {
                    AStarNode* neighbor = new AStarNode(Point(newX, newY));
                    neighbor->g = current->g + 1; // 更新实际代价
                    neighbor->h = neighbor->calculateH(goal); // 计算启发式代价
                    neighbor->calculateF(); // 计算总代价
                    neighbor->parent = current;

                    openList.push(neighbor);
                    closedList[newX][newY] = true;
                }
            }
        }
    }
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
        AStarNode* start = new AStarNode(MapParam.StartPoint);
        AStarNode* goal = new AStarNode(MapParam.TargetPoint);
        AStar(start, goal, Maptest, PathList);
        //cout << PathList << endl;

        PathGrid2world(MapParam, PathList,plan_path);
        path_pub.publish(plan_path);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
