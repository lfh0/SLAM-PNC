/**************************************************************************
 * ASatrSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.03.16
 * 
 * @Description:
 *  本程序为A*路径搜索算法实现
 *  
 * 参考程序:
 * https://blog.csdn.net/yuxuan20062007/article/details/86767355
 * https://blog.csdn.net/qq_37937847/article/details/122501731?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_ecpm_v1~rank_v31_ecpm-3-122501731.pc_agg_new_rank&utm_term=c%2B%2B+%E5%8F%AF%E8%87%AA%E5%8A%A8%E6%8E%92%E5%BA%8F%E7%9A%84%E5%AE%B9%E5%99%A8&spm=1000.2123.3001.4430
 *  ****************************************************/

#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "tic_toc.h"
#include <mutex>
#include <map>
#include <set> 
#include <deque>
#include <queue>
#include <vector>
#include <algorithm>

using namespace std;

// f(n) = g(n) + h(n)
typedef struct Node
{
    int x;
    int y;
    float g;
    float h;
    float f;
    Node* father_node;
    Node(){

    }
    Node(int x, int y){
        this->x = x;
        this->y = y;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father_node = NULL;
    }
    Node(int x, int y, Node* father){
        this->x = x;
        this->y = y;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father_node = father;
    }

    // //重载 ==
    // bool operator == (const Node & n) const{
    //     if (x == n.x && y == n.y){
    //         return true;
    //     }else
    //     {
    //         return false;
    //     }
    // }
    // // 重载 < 
    // bool operator < (const Node &n) const {
    //     if (f != n.f)
    //     {
    //         return f < n.f;
    //     }else 
    //     {
    //         return h < n.h;
    //     }

    // }

    // 重载输出函数
    friend ostream& operator << (ostream& ostr, Node &n){

        if (n.father_node == NULL)
        {
            cout << "[ " << n.x << " , " << n.y << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- {  NULL  } " << std::endl;
        }else
        {
            cout << "[ " << n.x << " , " << n.y << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- { " << (n.father_node)->x<< " , " << (n.father_node)->y << " } ";
        }

        return cout;
    } 

}Node;

// // 重载()运算符
// class NodeCompare
// {
//     public:
//         bool operator()(Node n1, Node n2){
//             if (n1.f < n2.f)
//             {
//                 return n1.f < n2.f;
//             }else if (n1.f = n2.f)
//             {
//                 return n1.h < n2.h;
//             }

//         }
// };

typedef std::vector<Node> NodeSet;
typedef std::vector<Node>::iterator NodeSetIter;

// ------------------------------ class --------------
class AStarSearch
{
private:
    int heuristic_func_;

    int iter_;
    int iter_step_;
    bool map_updated_;
    bool search_finished_;

    nav_msgs::OccupancyGrid map_;

    NodeSet open_set_;
    NodeSet close_set_;

    // for vis
    std::mutex vis_mut_;
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::deque<geometry_msgs::Point> p_final_que_;


private:

    void realCoordi2map(geometry_msgs::Point p_real, geometry_msgs::Point &p_map, nav_msgs::OccupancyGrid map_data);
    void map2realCoordi(geometry_msgs::Point &p_real, geometry_msgs::Point p_map, nav_msgs::OccupancyGrid map_data);
    void checkNeighbor(Node node, std::vector<Node> &neiNodes);
    bool checkNode(Node this_node, 
                    Node end_node, 
                    NodeSet &open_set,
                    NodeSet close_set,
                    nav_msgs::OccupancyGrid map);
    int CollisionFree(Node node, nav_msgs::OccupancyGrid map);
    void countCost(Node &node, Node end_node, int heuristic_func_type);
    int inSet(Node node, NodeSet node_set);
    void printSet(NodeSet node_set);
    void backSetNode(Node node);
public:
    AStarSearch(int heuristic_func, int iter_step);
    
    ~AStarSearch();

    void updateMap(nav_msgs::OccupancyGrid map_data);
    bool getSearchState(); 
    void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                             std::queue<geometry_msgs::Point> &p_final_q,
                             int iter);
    void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    static bool nodeCompare(Node &n1, Node &n2); // 特别注意这里的nodeCompare需要是静态函数
};

//******************************************************
// 【输入】 - heuristic_func : 选择启发函数类型 
//            0 - Manhattan Distance 
//            1 - Diagonal distance 
//            2 - Euclidean distance
// 【输入】 - iter_step : 为了更好的显示，每隔iter_step步，需要键入值以继续 这里设为最大值即可
AStarSearch::AStarSearch(int heuristic_func, int iter_step) :
heuristic_func_(heuristic_func),
iter_step_(iter_step)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [A star] search method!! .\033[0m");
}

AStarSearch::~AStarSearch()
{
}

void AStarSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_updated_ = true;
    map_ = map_data;
}

bool AStarSearch::getSearchState(){
    return search_finished_;
}

void AStarSearch::printSet(NodeSet node_set){

    std::cout << ">>>>>>>>>> set ------ " << node_set.size() << std::endl;
    for(NodeSetIter it = node_set.begin(); it != node_set.end(); it++){
        if (it->father_node == NULL)
        {
            std::cout << "[ " << it->x << " , " << it->y << " ] -- " << "GHF: " << it->g << " " << it->h << " " << it->f 
            << " -- {  NULL  } " << std::endl;
        }else
        {
            std::cout << "[ " << it->x << " , " << it->y << " ] -- " << "GHF: " << it->g << " " << it->h << " " << it->f 
            << " -- { " << it->father_node->x<< " , " << it->father_node->y << " } " << std::endl;
        }
    }
    std::cout << "-------- set <<<<<<<<" <<std::endl;

}


bool AStarSearch::nodeCompare(Node &n1, Node &n2){
    if (n1.f == n2.f)
    {
        return n1.h < n2.h;
    }else
    {
        return n1.f < n2.f;
    }
}


// **************************************
// 将点的真实坐标转换为地图坐标
// 这里的地图坐标是 A_star搜索中构建的地图，其以地图左下角(map_data.info.origin.position)为坐标原点， 地图的宽(map_data.info.width)为x方向 ，
//                 高(map_data.info.height)为y方向， 地图的分辨率(map_data.info.resolution)为大小构建的方格地图
//  ^ y
//  |
//  |
//  -----------> xstd::cout << "in close set " << std::endl;
void AStarSearch::realCoordi2map(geometry_msgs::Point p_real, geometry_msgs::Point & p_map, nav_msgs::OccupancyGrid map_data){
    p_map.x = floor((p_real.x - map_data.info.origin.position.x)/map_data.info.resolution);
    p_map.y = floor((p_real.y - map_data.info.origin.position.y)/map_data.info.resolution);

}

// **************************************
// 将地图坐标点转换为真实坐标
void AStarSearch::map2realCoordi(geometry_msgs::Point &p_real, geometry_msgs::Point p_map, nav_msgs::OccupancyGrid map_data){
    p_real.x = p_map.x * map_data.info.resolution + map_data.info.origin.position.x + 0.5 * map_data.info.resolution;
    p_real.y = p_map.y * map_data.info.resolution + map_data.info.origin.position.y + 0.5 * map_data.info.resolution;
}

//*******************************************
// 返回map坐标系中 node坐标的占有状态
// 占据状态　1:占据　0:未占据　-1:未知
int AStarSearch::CollisionFree(Node node, nav_msgs::OccupancyGrid map){

    std::vector<signed char> Data = map.data;
    int index = node.y * map.info.width + node.x;

    int state, out;
    out = Data[index];

    if (out == 100) // 被占据
    {
        state = 1;
    }

    if (out == -1) //未知
    {
        state = -1;
    }

    if (out == 0)
    {
        state = 0;
    }
    
    return state;

}


//*******************************
// 搜索当前节点的临近节点
// 这里采用八连通的搜索方式，　既是一个节点可以衍生出八个临近节点
//【输入】node 当前节点
//【输入】map地图信息
//【输出】neiNodes 当前节点的临近节点 --这些节点的父节点是当前节点
void AStarSearch::checkNeighbor(Node node, std::vector<Node> &neiNodes){

    Node node1(node.x, node.y + 1);
    Node node2(node.x, node.y - 1);
    
    Node node3(node.x + 1, node.y + 1);
    Node node4(node.x + 1, node.y - 1);
    Node node5(node.x + 1, node.y    );

    Node node6(node.x - 1, node.y + 1);
    Node node7(node.x - 1, node.y - 1);
    Node node8(node.x - 1, node.y    );

    neiNodes.push_back(node1);
    neiNodes.push_back(node2);
    neiNodes.push_back(node3);
    neiNodes.push_back(node4);
    neiNodes.push_back(node5);
    neiNodes.push_back(node6);
    neiNodes.push_back(node7); 
    neiNodes.push_back(node8);

}


//********************************
// 检测该临近节点this_node是否加入open_set
//
//【输入】　this_node 当前临近节点
//【输入】　end_node 终点
//【输出】　open_set　该参数会被修改
//【输入】　close_set　
//【输入】　map 地图信息
//【返回值】表示this_node是否加入open_set
bool AStarSearch::checkNode(Node this_node, 
                            Node end_node, 
                            NodeSet &open_set,
                            NodeSet close_set,
                            nav_msgs::OccupancyGrid map){
    
    std::cout << "--[check node] --------[ " <<  this_node.x << " , " << this_node.y << " ]"<< std::endl;
    // 检测 node 是否在map范围内
    if (this_node.x < 0 || this_node.x > map.info.width || this_node.y < 0 || this_node.y > map.info.height)
    {
        std::cout << "--[check node] outside map " << std::endl;
        return false;
    }
    
    // 检测 node 的占有状态
    int checking = CollisionFree(this_node, map);
    if (checking == 1)
    {
        std::cout << "--[check node] collision 1 " << std::endl;
        return false;
    }

    if (checking == -1)
    {
        std::cout << "--[check node] collision -1 " << std::endl;
        return false;
    }
    
    if (checking == 0) //free状态
    {
        std::cout << "--[check node] collision free " << std::endl;
        // 检测是否在close_set中
        if (inSet(this_node, close_set) != -1) // this_node  在 close_set中
        {
            std::cout << "--[check node] in close set " << std::endl;
            return false;
        }
        
        // 计算 this_node 代价
        countCost(this_node, end_node, heuristic_func_);
        std::cout << "--[check node] this_node-- " << this_node << std::endl;
        // 检测是否在open_set中

        int index = inSet(this_node, open_set);
        if (index != -1)
        {
            std::cout << "--[check node] in open set " << std::endl;
            if (this_node.f < open_set[index].f)
            {
                std::cout << "--[check node] rewrite node " << std::endl;
                open_set[index].father_node = this_node.father_node;
                open_set[index].f = this_node.f;
                open_set[index].h = this_node.h;
                open_set[index].g = this_node.g;
            }
            return false;
            
        }else
        {
            std::cout << "--[check node] add new node  " << std::endl;
            open_set.push_back(this_node);
            return true;
        }
        
    }

    return false;
}


//*********************************
int AStarSearch::inSet(Node node, NodeSet node_set){

    for (int ind = 0; ind < node_set.size(); ind++)
    {
        if (node.x == node_set[ind].x && node.y == node_set[ind].y){
            return ind;
        }
    }

    return -1;
    
}


void AStarSearch::backSetNode(Node node){
    if (node.father_node != NULL)
    {
        std::cout << "[back set: ] " << node << std::endl;
        geometry_msgs::Point p, p_map;
        p_map.x = node.x;
        p_map.y = node.y;
        map2realCoordi(p, p_map, map_);
        p_final_que_.push_back(p);

        backSetNode(*(node.father_node));
    }else
    {
        std::cout << "[back set: ] " << node << std::endl;
        geometry_msgs::Point p, p_map;
        p_map.x = node.x;
        p_map.y = node.y;
        map2realCoordi(p, p_map, map_);
        p_final_que_.push_back(p);
        return;
    }

}


//**********************************
// 计算当前节点的代价
//【输入】 node 当前节点  node 值中的 f, g, h会被修改
//【输入】 end_node 终点
//【输入】 heuristic_func_type 启发函数类型， 用什么方法计算当前节点与终点的启发值
// 0 - Manhattan Distance 曼哈顿距离
// 1 - Diagonal distance 对角距离
// 2 - Euclidean distance 欧几里得距离(直线距离)
//【输出】 node 赋值f,g,h的当前节点值
// 参考： https://zhuanlan.zhihu.com/p/54510444
void AStarSearch::countCost(Node& node, Node end_node, int heuristic_func_type){
    float f, g, h;
    float dx = abs(node.x - end_node.x);
    float dy = abs(node.y - end_node.y);

    // 计算 h
    if (heuristic_func_type == 0) 
    {
        h = dx + dy;
    }else if (heuristic_func_type == 1)
    {
        h = dx + dy + (sqrt(2) -2 ) * min(dx, dy);
    }else if (heuristic_func_type == 2)
    {
        h = pow((dx, 2) + pow(dy, 2), 0.5);
    }
    
    // 计算 g = father.g + node 到 father_node的距离
    g = node.father_node->g + sqrt(pow(node.x - node.father_node->x,2) + pow(node.y - node.father_node->y,2));

    f = g + h;

    node.f = f;
    node.g = g;
    node.h = h;

}

//**********************************************
// 主函数
void AStarSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
    
    //
    while (!map_updated_)
    {
        ROS_INFO("no map info !! use updateMap function first");
    }
    
    int a;
    geometry_msgs::Point p_map_start, p_map_end;

    realCoordi2map(p_start, p_map_start, map_);
    Node star_node(int(p_map_start.x), int(p_map_start.y));

    realCoordi2map(p_end, p_map_end, map_);
    Node end_node(int(p_map_end.x), int(p_map_end.y));

    //>>>> 清空参数  多次调用search()时需要
    open_set_.clear();
    close_set_.clear();
    iter_ = 0;

    //<<<<

    open_set_.push_back(star_node);

    std::cout << " --- " << open_set_.size() << std::endl;

    

    Node current_node;

    std::queue<geometry_msgs::Point> vis_point_que;

    while (open_set_.size() > 0)
    {

        std::cout << "************************** " << iter_ << std::endl;
        while (!vis_point_que.empty())
        {
            vis_point_que.pop();
        }
        
        // printSet(open_set_);

        iter_++;

        if(iter_ % iter_step_ == 0){
            std::cout << "input any to go on!!" << std::endl;
            a = cin.get();
        }

        
        // current_node.x = open_set_.begin()->x; //返回open_set中的最小值
        // current_node.y = open_set_.begin()->y;
        // current_node.g = open_set_.begin()->g;
        // current_node.h = open_set_.begin()->h;
        // current_node.f = open_set_.begin()->f;
        // current_node.father_node = open_set_.begin()->father_node;

        current_node = open_set_[0];
        

        std::cout << "cu node: " << current_node << std::endl;

        //printSet(open_set_);

        if (current_node.x == end_node.x && current_node.y == end_node.y)
        {
            ROS_INFO("find the path!!!");
            
            search_finished_ = true;

            vis_mut_.lock();
            backSetNode(current_node);
            vis_mut_.unlock();
            ROS_INFO("back path over!!!");
            break;
        }
        
        // 检测邻域 
        std::vector<Node> neighborNodes;
        checkNeighbor(current_node, neighborNodes);
        std::cout << " neighborNodes size: " << neighborNodes.size() << std::endl;
        
        //　检测current_node的邻域点
        geometry_msgs::Point p_new, p_map_new;
        Node* father = new Node(current_node.x, current_node.y, current_node.father_node);
        father->g = current_node.g;
        father->h = current_node.h;
        father->f = current_node.f;
        
        for (int i=0; i < neighborNodes.size(); i++)
        {
            neighborNodes[i].father_node = father;

            if(checkNode(neighborNodes[i], end_node, open_set_, close_set_, map_)){
                
                p_map_new.x = neighborNodes[i].x;
                p_map_new.y = neighborNodes[i].y;

                map2realCoordi(p_new, p_map_new, map_);
                vis_point_que.push(p_new);
            }
        }
        
        std::cout << "-----erase------" << std::endl;
        // 从open_set中删除current_node
        for (NodeSetIter it = open_set_.begin(); it != open_set_.end(); it++)
        {
            if (it->x == current_node.x && it->y == current_node.y)
            {
                open_set_.erase(it);
            }
            
        }

        //　在close_set添加current_node
        close_set_.push_back(current_node);

        //排序open_set
        sort(open_set_.begin(), open_set_.end(), nodeCompare);

        vis_mut_.lock();
        iter_vis_set_.insert(make_pair(iter_, vis_point_que));
        vis_mut_.unlock();

    }
    
}

//****************************************
//// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的新加点, 
void AStarSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                             std::queue<geometry_msgs::Point> &p_final_q,
                             int iter){
    
    vis_mut_.lock();
    if (!iter_vis_set_.empty())
    {
        copy(iter_vis_set_.begin(), iter_vis_set_.end(), inserter(iter_vis_info, iter_vis_info.begin()));
        iter_vis_set_.clear();
    }

    if (search_finished_)
    {
        while (!p_final_que_.empty())
        {
            p_final_q.push(p_final_que_.back()); //因为p_final_que_是从终点往起点储存的，所以这里要从后往前读，p_final_q才是从起点往终点储存
            p_final_que_.pop_back();
        }
    }

    vis_mut_.unlock();

    iter = iter_;
}





