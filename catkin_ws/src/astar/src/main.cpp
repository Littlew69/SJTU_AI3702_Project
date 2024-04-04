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

double calc_d(Point x, Point Target){
    return sqrt((x.x - Target.x)*(x.x-Target.x) + (x.y - Target.y)*(x.y - Target.y));
}

bool unreachable(Point y){
    int inflating = 1;
    for (int i =-inflating;i<=inflating;i++){
        for (int j=-inflating;j<=inflating;j++){
            if (i* i + j* j >inflating*inflating) continue;
            if (y.x+i<0 || y.y+j<0 || y.x+i>=MapParam.width || y.y+j >= MapParam.height) continue;
            if (y.x==MapParam.StartPoint.x && y.y==MapParam.StartPoint.y) continue;
            if (Maptest.at<uchar>(y.x+i, y.y+j)< 100)
                return true;
        }
    }    
    return false;
}
const int tx[8] = {1,1,-1,-1, 0, 0, 1, -1};
const int ty[8] = {1,-1,1,-1, 1,-1, 0, 0};

struct cmp{
    bool operator()(const Point &a, const Point &b)const{
        return a.x<b.x || (a.x==b.x & a.y<b.y);
    }
};

map<Point, int, cmp> mp;
vector<Point> PathList;
priority_queue<pair<int, int> > Q;
vector<node> node_list;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;

 
    geometry_msgs::PointStamped astar_step;

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("move_base/NavfnROS/Astar/initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base/NavfnROS/Astar/target", 10, TargetPointtCallback);

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Rate loop_rate(20);
    nav_msgs::Path plan_path;


    Point Last_Start;
    Point Last_Target;

    while(ros::ok())
    {
        if (MapParam.StartPoint != Last_Start && MapParam.TargetPoint == Last_Target){
            bool flag = 0;
            for (int i=0;i<PathList.size();i++)
                if (PathList[i].x == MapParam.StartPoint.x && PathList[i].y == MapParam.StartPoint.y)
                    {
                        ROS_INFO("%d\n", PathList.size());
                        PathList.erase(PathList.begin(), PathList.begin()+i+1);
                        ROS_INFO("i:%d, %d\n", i, PathList.size());
                        PathGrid2world(MapParam, PathList, plan_path);
                        path_pub.publish(plan_path);
                        ROS_INFO("CONTINUE PREVIOUS PATH!!");
                        flag = 1;
                        Last_Start = MapParam.StartPoint;
                        break;
                    }
            if (flag) continue;
        }
        if (MapParam.StartPoint != Last_Start || MapParam.TargetPoint != Last_Target){
            ROS_INFO("Start Planning!");
            Last_Start = MapParam.StartPoint;
            Last_Target = MapParam.TargetPoint;
            double dis = calc_d(Last_Start, Last_Target);
            bool flag = 0;
            if (dis > 200) flag = 1;
            
            PathList.clear();
            // map<Point, int, cmp > empty_map1;
            // mp.swap(empty_map1);
            mp.clear();

            mp[MapParam.StartPoint] = 0;
            int tot = 1;
            node_list.clear();
            node_list.push_back(node(0,calc_d(MapParam.StartPoint, MapParam.TargetPoint), -1, MapParam.StartPoint));

            while (! Q.empty()) Q.pop();
            Q.push(make_pair(calc_d(MapParam.StartPoint, MapParam.TargetPoint), 0));
            while (!Q.empty()){
                bool flag_b = 0;
                pair<int, int> p = Q.top();
                Q.pop();
                int idx = p.second;
                Point x = node_list[idx].p;
                if (x==MapParam.TargetPoint) break;
                for (int i=0;i<8;i++){
                    Point y;
                    y.x = x.x + tx[i];
                    y.y = x.y + ty[i];
                    if (y.x<0 || y.y<0 || y.x>=MapParam.width || y.y>=MapParam.height)
                        continue;
                    if (unreachable(y) || (i<4 && unreachable(Point(x.x,y.y)) && unreachable(Point(x.y,y.x))))
                        continue;
                    double g = node_list[idx].g + calc_d(x,y);
                    // double g = node_list[idx].g + 1;
                    double h = calc_d(y, MapParam.TargetPoint);
                    if (mp.find(y)!=mp.end()){
                        int idy = mp[y];
                        if (g+h < node_list[idy].g + node_list[idy].h){
                            node_list[idy].g = g;
                            node_list[idy].h = h;
                            if (!flag)
                                Q.push(make_pair(-g, idy));
                            else Q.push(make_pair(-g - h, idy));     
                        }
                    } else{
                        mp[y] = tot++;
                        node_list.push_back(node(g, h, idx, y));
                        if (!flag)
                            Q.push(make_pair(-g, tot-1));
                        else Q.push(make_pair(-g-h, tot-1));
                    } 
                    if (y==MapParam.TargetPoint) {
                        flag_b = 1;
                    }
                }
                if (flag_b) break;
            }
            if (mp.find(MapParam.TargetPoint)==mp.end()){
                ROS_ERROR("COULD NOT FIND A FEASIBLE PATH UNDER THIS OCCUPY THRESHOLD AND THIS INFLATING RADIUS!, S(%d, %d), T(%d, %d)\n", MapParam.StartPoint.x, MapParam.StartPoint.y, MapParam.TargetPoint.x, MapParam.TargetPoint.y);
            }else{
                PathList.clear();
                for (int id = mp[MapParam.TargetPoint]; id >0; ){
                    PathList.push_back(node_list[id].p);
                    id = node_list[id].pre;
                }
                reverse(PathList.begin(), PathList.end());
                PathGrid2world(MapParam, PathList, plan_path);
                path_pub.publish(plan_path);
                ROS_INFO("Find a valid path successfully! Euclid Distance: %.3lf, Total Steps: %d\n", node_list[mp[MapParam.TargetPoint]].g, (int) (PathList.size()-1 ));
            }
        } else{
            // PathList.erase(PathList.begin(), PathList.begin()+1);
            PathGrid2world(MapParam, PathList, plan_path);
            path_pub.publish(plan_path);
            //ROS_INFO("NO CHANGE! REPUB THE PATH!");
        }
    //    PathGrid2world(MapParam, PathList,plan_path);
    //    path_pub.publish(plan_path);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
