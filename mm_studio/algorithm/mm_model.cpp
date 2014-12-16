#include "mm_model.h"
#include "scale_model.h"
#include <fstream>
#include <ctime>

#define INITIAL_LENGTH 1

MM::MM() {
}

MM::~MM() {
}

double Distance2(double x1, double y1, double x2, double y2) {
    double dist_x = x1 - x2;
    double dist_y = y1 - y2;
    
    double dist = dist_x * dist_x + dist_y * dist_y;
    
    return std::sqrt(dist);
}

void MM::Clear() {
    action_to_index_.clear();
    index_to_action_.clear();
    state_to_index_.clear();
    index_to_state_.clear();
    suitable_actions_for_state_.clear();
    parsing_result_.clear();
    action_list_.clear();
}

bool MM::InitModel(boost::shared_ptr<Route> route, std::vector<std::vector<wxPoint2DDouble> >& candidate_points, std::vector<ShortestPath>& shortest_paths) {
    // clear previous state
    Clear();
    
    route_ = route;
    
    // 计算候选点到gps点的最大距离
    max_candidate_distance_ = 0;
    min_candidate_distance_ = 1e6;
    
    for (int i = 0; i < candidate_points.size(); ++i) {
        std::vector<wxPoint2DDouble>& candidates = candidate_points[i];
        double x1, y1;
        route_->getRouteVertex(i, x1, y1);
        
        for (int j = 0; j < candidates.size(); ++j) {
            double distance = Distance2(x1, y1, candidates[j].m_x, candidates[j].m_y);
            
            if (max_candidate_distance_ < distance)
                max_candidate_distance_ = distance;
            if (min_candidate_distance_ > distance)
                min_candidate_distance_ = distance;
        }
    }
    
#if PRINT_INFO
    std::cout << "min candidate distance: " << min_candidate_distance_ << std::endl;
    std::cout << "max candidate distance: " << max_candidate_distance_ << std::endl;
#endif
    
    // 计算两两gps点之间的最长路径
    max_route_length_ = 0;
    min_route_length_ = 1e6;
    
    for (int i = 0; i < shortest_paths.size(); ++i) {
        if (shortest_paths[i].path_.size() == 0)
            continue;
        
        if (max_route_length_ < shortest_paths[i].length_)
            max_route_length_ = shortest_paths[i].length_;
        
        if (min_route_length_ > shortest_paths[i].length_)
            min_route_length_ = shortest_paths[i].length_;
    }
    
#if PRINT_INFO
    std::cout << "max route length: " << max_route_length_ << std::endl;
    std::cout << "min route length: " << min_route_length_ << std::endl;
#endif
    
    // 添加虚拟的初始点
    initial_state_size_ = candidate_points[0].size();
    for (int i = 0; i < initial_state_size_; ++i) {
        ShortestPath sp;
        sp.from_vertex_gps_id_ = -1;
        sp.from_vertex_candidate_id_ = -1;
        sp.from_vertex_id_ = -1;
        sp.from_vertex_location_x_ = -1;
        sp.from_vertex_location_y_ = -1;
        sp.length_ = min_route_length_;
        sp.nodes_.resize(1);
        sp.to_vertex_gps_id_ = 0;
        sp.to_vertex_candidate_id_ = i;
        sp.to_vertex_location_x_ = candidate_points[0][i].m_x;
        sp.to_vertex_location_y_ = candidate_points[0][i].m_y;
        sp.path_.resize(1);
        shortest_paths.push_back(sp);
    }
    
    //初始化起始点
    current_state_.gps_id_ = -1;
    current_state_.candidate_id_ = -1;

    // 状态到int索引的映射
    states_number_  = InitStateMap(candidate_points);

    // 决策到int索引的映射
    actions_number_ = InitActionMap(shortest_paths);
    
    // 计算某个状态下的决策集
    bool result = InitSymbolSuitableActions();

    // 初始化强化学习的Q表
    RLAlgorithm::Init(states_number_, actions_number_);

    // 设置学习参数
    epsilon_ = 0.9;
    gamma_ = 1.0;
    learning_rate_ = 0.1;
    episodes_ = 50000;
    
    return result;
}

int MM::InitActionMap(std::vector<ShortestPath>& shortest_paths) {
#if PRINT_INFO
    std::cout << "计算决策到int索引的映射 ...\n";
#endif
    int count = 0;
    for (int i = 0; i < shortest_paths.size(); ++i) {
        action_to_index_.insert(std::make_pair(shortest_paths[i], count));
        index_to_action_.insert(std::make_pair(count, shortest_paths[i]));
        ++count;
    }
#if PRINT_INFO
    std::cout << "总共有 " << count << " 个决策。\n";
#endif
    return count;
}

int MM::InitStateMap(std::vector<std::vector<wxPoint2DDouble> >& candidate_points) {
#if PRINT_INFO
    std::cout << "计算状态到int索引的映射 ...\n";
#endif
    // 终止状态设置为最后一个gps点的ID
    terminal_state_ = candidate_points.size() - 1;
    
    // 虚拟初始点
    State state;
    state.gps_id_ = -1;
    state.candidate_id_ = -1;
    state_to_index_.insert(std::make_pair(state, 0));
    index_to_state_.insert(std::make_pair(0, state));
    
    // 其它候选点
    int count = 1;
    for (int i = 0; i < candidate_points.size(); ++i) {
        std::vector<wxPoint2DDouble>& candidates = candidate_points[i];
        for (int j = 0; j < candidates.size(); ++j) {
            State state;
            state.gps_id_ = i;
            state.candidate_id_ = j;
            state_to_index_.insert(std::make_pair(state, count));
            index_to_state_.insert(std::make_pair(count, state));
            
            ++count;
        }
    }
#if PRINT_INFO
    std::cout << "总共有 " << count << " 个状态。\n";
#endif
    return count;
 }

bool MM::InitSymbolSuitableActions() {
#if PRINT_INFO
    std::cout << "计算某个状态下的决策集 ...\n";
#endif
    for (int i = 0; i < index_to_state_.size(); ++i) {
        State state = index_to_state_.at(i);
        std::vector<int> suitable_actions;
        
        for (int j = 0; j < index_to_action_.size(); ++j) {
            Action action = index_to_action_.at(j);
            
            if (state.gps_id_ == action.from_vertex_gps_id_ &&
                state.candidate_id_ == action.from_vertex_candidate_id_ &&
                // action.length_ != 0 &&
                action.path_.size() > 0
                ) {
                suitable_actions.push_back(j);
            }
        }
        
        suitable_actions_for_state_.insert(std::make_pair(i, suitable_actions));
     
#if PRINT_INFO
        std::cout << "状态 " << i << " 对应 " << suitable_actions.size() << " 个决策.\n";
#endif
    }
    
#if PRINT_INFO
    // 确保每个非终止状态都有对应的对策
    for (int i = 0; i < terminal_state_; ++i) {
        bool has_actions = false;
        for (int j = 0; j < suitable_actions_for_state_.size(); ++j) {
            State state = index_to_state_.at(j);
            std::vector<int> suitable_actions = suitable_actions_for_state_.at(j);
            if (state.gps_id_ == i && suitable_actions.size() > 0) {
                has_actions = true;
                break;
            }
        }
        if (!has_actions) {
            std::cout << "gps id = " << i << " has no suitable actions!\n";
            return false;
        }
    }
#endif
    
    return true;
}


double MM::RunParsingAlgorithm(int episodes, int greedy_episodes, double learning_rate) {
    parsing_result_.clear();
    action_list_.clear();
    
#if PRINT_INFO
    std::cout << "启动强化学习算法...\n";
#endif
    
    episodes_ = episodes;
//    unsigned int rand_seed = static_cast<unsigned int>(std::time(0));
//    std::srand(rand_seed);
    
    learning_ = true;
    record_ = false;
    learning_rate_ = learning_rate;
    epsilon_ = 0.1;

    // 总共学习@episodes_次
    for(int i = 0; i < episodes_; ++i) {
        // 动态调整学习参数
        double seed = (double) i / (double) episodes_;
        epsilon_ = ScaleModel::ExpTrans(seed, 0.001, 0.999);
        gamma_ = 1 - epsilon_;
        // learning_rate_ = 1 - epsilon_;

        // 重新初始化学习
        Reset();
        
        // 从初始状态开始学习
        double score = RunTask(current_state_);
        
#if PRINT_INFO
        // 没一千次学习输出当前学习成果（得分）
        if((i + 1) % 1000 == 0) {
            std::cout << "\n场景 " << i << ". 得分 = " << score <<
                      " epsilon: " << epsilon_ << " learning_rate_: " << learning_rate_ /*<< " Seed: " << rand_seed*/ <<
                      std::endl;
        }
        
        // 每一万次学习计算一次贪婪算法得分
        if((i + 1) % 10000 == 0) {
            //double back_learning_rate = learning_rate_;
            double back_epsilon = epsilon_;

            //learning_rate_ = 0.01;
            epsilon_ = 1.0;
            learning_ = false;

            Reset();
            double score = RunTask(current_state_);
            std::cout << "\n\n贪婪得分: " << score << std::endl;

            //learning_rate_ = back_learning_rate;
            epsilon_ = back_epsilon;
            learning_ = true;
        }
#endif
        
    }

    // 使用更大的贪婪性去学习
    epsilon_ = 0.9;
    gamma_ = 0.1;

    for(int i = 0; i < greedy_episodes; ++i) {
        double score = 0.0;
        // Reset the environment(recover the parsing original)
        Reset();
        
        // Parsing from root state
        score = RunTask(current_state_);
        
#if PRINT_INFO
        if(i % 10000 == 0) {
            std::cout << "\场景 " << i << " 得分: " << score <<
                      " epsilon: " << epsilon_ << " gamma: " << gamma_ /*<< " Seed: " << rand_seed*/ << std::endl;
        }
#endif
    }

    // 根据学习的结果使用贪婪算法得到最优解
    Reset();
    epsilon_ = 1.0;
    learning_ = false;
    record_ = true;
    double score = RunTask(current_state_);
#if PRINT_INFO
    std::cout << "\贪婪得分: " << score << std::endl;
#endif
    return (score);
}


double MM::RunTask(State& state) {
    action_list_.clear();
    
    // 场景得分
    double episode_score = 0.0;
    int action_index, next_state_index;

    // 一直执行直到场景结束为止
    while(state.gps_id_ != terminal_state_) {
        // 场景中某一步的得分
        double score = 0.0;
        
        // 用于更新Q表
        bool end_of_task = false;

        // 当前状态的索引
        int state_index = state_to_index_.at(state);
        
        // 当前状态对应的决策集，不应该是空的
        std::vector<int> suitable_actions = suitable_actions_for_state_.at(state_index);
        if (suitable_actions.size() == 0)
            return -10.0;

        // 探索一个决策
        Explore(state_index, action_index, epsilon_, suitable_actions);
        Action action = index_to_action_.at(action_index);
        
        // 该决策导致状态转移
        State next_state;
        next_state.gps_id_ = action.to_vertex_gps_id_;
        next_state.candidate_id_ = action.to_vertex_candidate_id_;
        next_state_index = state_to_index_.at(next_state);
        
        // 记录该状态
        //if (record_) {
        //    parsing_result_.push_back(next_state);
            action_list_.push_back(action);
        //}
        
        // 计算得分
        double x1, y1;
        route_->getRouteVertex(action.to_vertex_gps_id_, x1, y1);
        double candidate_score = (Distance2(action.to_vertex_location_x_, action.to_vertex_location_y_, x1, y1) - min_candidate_distance_) / (max_candidate_distance_ - min_candidate_distance_);
        
        double route_score = (action.length_ - min_route_length_) / (max_route_length_ - min_route_length_);
        
       // std::cout << 1.0 - candidate_score << std::endl;
       // std::cout << 1.0 - route_score << std::endl;
        
        episode_score = episode_score + 0.4 * (1.0 - candidate_score) + 0.2 * (1.0 - route_score);
        
        // 改状态是否是终止状态？
        if (next_state.gps_id_ == terminal_state_) {
            end_of_task = true;

            double similarity_score = 0.4 * CalculateSimilarity();
            
            std::cout << "除去相似性得得分: " << episode_score << std::endl;
            std::cout << "相似性得分：" << similarity_score << std::endl;
            
            // 评价整条路径的相似度
            episode_score += similarity_score;
            
            
            score = episode_score;
#if PRINT_INFO
            if (score < 0)
                std::cout << "Warning: score is smaller than zero!\n~\n~\n~\n";
#endif
        } else {
            score = 0.0;
        }

        // 更新Q值表
        if(learning_) {
            // update state-action value pair
            // When @end_of_task == true, @next_state_ is not used.
            Update(state_index, action_index, score, next_state_index, end_of_task, learning_rate_, gamma_);
        }

        // 转移状态
        state = next_state;
    }

    return episode_score;
}

void MM::Reset() {
    // 重新设置当前的状态
    current_state_.gps_id_ = -1;
    current_state_.candidate_id_ = -1;
    
    parsing_result_.clear();
}

bool MM::parsing_result_valid() {
    if (action_list_.size() == 0)
        return false;
    
    for (int i = 1; i < action_list_.size(); ++i) {
#if PRINT_INFO
        std::cout << "action " << i << ": length = " << action_list_[i].length_ << ", size = " << action_list_[i].path_.size() << std::endl;
#endif
        if (//action_list_[i].length_ == 0 ||
            action_list_[i].path_.size() == 0)
            return false;
    }
    
    return true;
}

double MM::CalculateSimilarity() {
    // 首先找到所有候选点到gps点的距离
    std::vector<double> distances;
    for (int i = 0; i < action_list_.size(); ++i) {
        assert(i == action_list_[i].to_vertex_gps_id_);
        
        double gps_x, gps_y;
        route_->getRouteVertex(action_list_[i].to_vertex_gps_id_, gps_x, gps_y);
        double dis = Distance2(gps_x, gps_y, action_list_[i].to_vertex_location_x_, action_list_[i].to_vertex_location_y_);
        distances.push_back(dis);
    }
    
    // 计算距离的平均值
    double ave_dis = 0;
    for (int i = 0; i < distances.size(); ++i) {
        ave_dis += distances[i];
    }
    ave_dis = ave_dis / distances.size();
    
    // 距离平均值的最大最小值
    double max_to_ave_dis = (distances[0] - ave_dis) * (distances[0] - ave_dis);
    double min_to_ave_dis = max_to_ave_dis;
    for (int i = 1; i < distances.size(); ++i) {
        double to_ave_dis = (distances[i] - ave_dis) * (distances[i] - ave_dis);
        if (max_to_ave_dis < to_ave_dis)
            max_to_ave_dis = to_ave_dis;
        if (min_to_ave_dis > to_ave_dis)
            min_to_ave_dis = to_ave_dis;
    }
    
    // 计算归一化标准差
    double S= 0;
    for (int i = 0; i < distances.size(); ++i) {
        double to_ave_dis = (distances[i] - ave_dis) * (distances[i] - ave_dis);
        double offset_persent = (to_ave_dis - min_to_ave_dis) / (max_to_ave_dis - min_to_ave_dis);
        S = S + (1.0 - offset_persent);
    }

    return S;
}