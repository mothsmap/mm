#include <boost/lexical_cast.hpp>
#include "debug.h"
#include "mm_sparse_solver.h"
#include "scale_model.h"
#include <ctime>
#include <fstream>

#define INITIAL_LENGTH 1

MM::MM() {
}

MM::~MM() {
}

void MM::Clear() {
    action_to_index_.clear();
    index_to_action_.clear();
    state_to_index_.clear();
    index_to_state_.clear();
    suitable_actions_for_state_.clear();
    action_list_.clear();
}

bool MM::InitModel(boost::shared_ptr<RTree> tree, boost::shared_ptr<Route> route) {
    // clear previous state
    Clear();
    
    tree_ = tree;
    route_ = route;
    
    // 添加虚拟的初始点
    std::vector<CandidatePointSet>&  candidate_set = route_->getCandidateSet();
    std::vector<CandidateTrajectory>& candidate_trajectory = route_->getCandidateTrajectories();
    
    initial_state_size_ = candidate_set[0].size();
    std::vector<int> initial_traj;
    
    for (int i = 0; i < initial_state_size_; ++i) {
        CandidateTrajectory ct = {
            -1, 0, // from gps id, to gps id
            -1, i, // from candidate id, to candidate id
            initial_traj // trajectory
        };

        candidate_trajectory.push_back(ct);
    }
    
    //初始化起始点
    current_state_.gps_id_ = -1;
    current_state_.candidate_id_ = -1;

    // 状态到int索引的映射
    states_number_  = InitStateMap(candidate_set);

    // 决策到int索引的映射
    actions_number_ = InitActionMap(candidate_trajectory);
    
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

int MM::InitActionMap(std::vector<CandidateTrajectory>& shortest_paths) {
    DebugUtility::Print(DebugUtility::Normal, "计算决策到int索引的映射 ...");
    
    int count = 0;
    for (int i = 0; i < shortest_paths.size(); ++i) {
        action_to_index_.insert(std::make_pair(shortest_paths[i], count));
        index_to_action_.insert(std::make_pair(count, shortest_paths[i]));
        ++count;
    }
    DebugUtility::Print(DebugUtility::Normal, "总共有 " + boost::lexical_cast<std::string>(count) + " 个决策。\n");

    return count;
}

int MM::InitStateMap(std::vector<CandidatePointSet>& candidate_points) {
    DebugUtility::Print(DebugUtility::Normal, "计算状态到int索引的映射 ...");

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
        CandidatePointSet candidates = candidate_points[i];
        for (int j = 0; j < candidates.size(); ++j) {
            State state;
            state.gps_id_ = i;
            state.candidate_id_ = j;
            state_to_index_.insert(std::make_pair(state, count));
            index_to_state_.insert(std::make_pair(count, state));
            
            ++count;
        }
    }
    DebugUtility::Print(DebugUtility::Normal, "总共有 " + boost::lexical_cast<std::string>(count) + " 个状态。\n");
    return count;
 }

bool MM::InitSymbolSuitableActions() {
    DebugUtility::Print(DebugUtility::Normal, "计算某个状态下的决策集 ...");
    for (int i = 0; i < index_to_state_.size(); ++i) {
        State state = index_to_state_.at(i);
        std::vector<int> suitable_actions;
        
        for (int j = 0; j < index_to_action_.size(); ++j) {
            Action action = index_to_action_.at(j);
            
            if (state.gps_id_ == action.from_gps_point_ &&
                state.candidate_id_ == action.from_candidate_point_ //&&
                // action.length_ != 0 &&
                //action.trajectory_.size() > 0
                ) {
                suitable_actions.push_back(j);
            }
        }
        
        suitable_actions_for_state_.insert(std::make_pair(i, suitable_actions));
     
//        DebugUtility::Print(DebugUtility::Normal, "状态 " + boost::lexical_cast<std::string>(i) +
//                            " 对应 " + boost::lexical_cast<std::string>(suitable_actions.size()) + " 个决策。\n");
    }
    
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
            DebugUtility::Print(DebugUtility::Error, "gps id =  " + boost::lexical_cast<std::string>(i) + " has no suitable actions!\n");
            return false;
        }
    }
    
    return true;
}


double MM::RunParsingAlgorithm(int episodes, int greedy_episodes, double learning_rate) {
    episodes_ = episodes;
    learning_rate_ = learning_rate;
    epsilon_ = 0.1;
    learning_ = true;
    record_ = false;
    
    DebugUtility::Print(DebugUtility::Normal, "启动强化学习算法... episodes = " + boost::lexical_cast<std::string>(episodes_) + ", learning rate = " + boost::lexical_cast<std::string>(learning_rate_) + ", epison = " + boost::lexical_cast<std::string>(epsilon_));
    
    // 总共学习@episodes_次
    for(int i = 0; i < episodes_; ++i) {
        //unsigned int rand_seed = static_cast<unsigned int>(std::time(0));
        //std::srand(rand_seed);
//#ifdef WIN32
//        srand(time(0));
//#else
//        srand48(clock());
//#endif
        
        // 动态调整学习参数
        double seed = (double) i / (double) episodes_;
        epsilon_ = ScaleModel::ExpTrans(seed, 0.001, 0.999);
        gamma_ = 1 - epsilon_;
        // learning_rate_ = 1 - epsilon_;

        // 重新初始化学习
        Reset();
        
        // 从初始状态开始学习
        double score = RunTask(current_state_);
#if 1
        // 每一千次学习输出当前学习成果（得分）及相应参数
        if((i + 1) % 100 == 0) {
            DebugUtility::Print(DebugUtility::Verbose, "场景 " + boost::lexical_cast<std::string>(i) +
                                ": score = " + boost::lexical_cast<std::string>(score) +
                                ", epsilon " + boost::lexical_cast<std::string>(epsilon_) +
                                ", learning rate = " + boost::lexical_cast<std::string>(learning_rate_) +
                                ", gamma = " + boost::lexical_cast<std::string>(gamma_));
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
#if 1
        if((i + 1) % 100 == 0) {
            DebugUtility::Print(DebugUtility::Verbose, "场景 " + boost::lexical_cast<std::string>(i) +
                                ": score = " + boost::lexical_cast<std::string>(score) +
                                ", epsilon " + boost::lexical_cast<std::string>(epsilon_) +
                                ", learning rate = " + boost::lexical_cast<std::string>(learning_rate_) +
                                ", gamma = " + boost::lexical_cast<std::string>(gamma_));
        }
#endif
    }

    // 根据学习的结果使用贪婪算法得到最优解
    Reset();
    epsilon_ = 1.0;
    learning_ = false;
    record_ = true;
    double score = RunTask(current_state_);
    
    DebugUtility::Print(DebugUtility::Normal, "贪婪得分: " + boost::lexical_cast<std::string>(score));
    
    return (score);
}


double MM::RunTask(State& state) {
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
        
        State next_state;
        
        // 当前状态对应的决策集，不应该是空的
        std::vector<int> suitable_actions = suitable_actions_for_state_.at(state_index);
        if (suitable_actions.size() == 0) {
            // DebugUtility::Print(DebugUtility::Warning, "empty actions for state " + boost::lexical_cast<std::string>(state_index));
            score = -1000000000.0;
            next_state.gps_id_ = terminal_state_;
        } else {
            // 探索一个决策
            Explore(state_index, action_index, epsilon_, suitable_actions);
            Action action = index_to_action_.at(action_index);
            
            // 该决策导致状态转移
            next_state.gps_id_ = action.to_gps_point_;
            next_state.candidate_id_ = action.to_candiate_point_;
            next_state_index = state_to_index_.at(next_state);
            
            // 记录该状态
            if (record_) {
                action_list_.push_back(action);
            }
            
            // 计算得分
#if 1
            // 候选点评分
            GPSPoint point = route_->getRouteVertex(action.to_gps_point_);
            CandidatePoint cp = route_->getCandidatePoint(action.to_gps_point_, action.to_candiate_point_);
            if (action.to_gps_point_ != cp.gps_point_id) {
                DebugUtility::Print(DebugUtility::Error, "Error! Candidate point GPS point mismatch!");
            }
            double dist_score = GeometryUtility::Distance(point.x_, point.y_, cp.proj_x_, cp.proj_y_);
            dist_score = 10000 - pow(dist_score, 2);
            if (dist_score < 0)
                dist_score = 0;
            
            // 候选路径评分
            double route_score = 0;
            
            if (state_index != 0) {
                for (int i = 0; i < action.trajectory_.size(); ++i) {
                    int travel_count = tree_->GetRoadInfo(action.trajectory_[i]).travel_counts_;
                    route_score += travel_count;
                }
                
                if (action.trajectory_.size() == 0) {
               //     std::cout << action.from_gps_point_ << "\t" << action.from_candidate_point_ << "\t" << action.to_gps_point_ << "\t" << action.to_candiate_point_ << std::endl;
                    
                    route_score = -10000000;
                } else {
                    route_score /= action.trajectory_.size();
                }
            }
            
            // 连通性评分
            double connecty = 0;
            if (next_state.gps_id_ != terminal_state_) {
                std::vector<int> suitable_actions = suitable_actions_for_state_.at(next_state_index);
                if (suitable_actions.size() == 0) {
                    connecty = -10000000;
                    
                    next_state.gps_id_ = terminal_state_;
                }
            }
            
            score = 666.666 * route_score + dist_score + connecty;
            
          //  std::cout << "route score: " << route_score << "\t" << "distance score: " << dist_score << std::endl;
#else
            // 候选路径评分
            double route_score = 0;
            for (int i = 0; i < action.trajectory_.size(); ++i) {
                int travel_count = tree_->GetRoadInfo(action.trajectory_[i]).travel_counts_;
                route_score += travel_count;
            }
            route_score /= action.trajectory_.size();
            score = route_score;
#endif
        }
        
 //       std::cout << "score: " << score << std::endl;
        // 该状态是否是终止状态？
        if (next_state.gps_id_ == terminal_state_) {
            end_of_task = true;
            episode_score += score;
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

    action_list_.clear();
}

bool MM::parsing_result_valid() {
    if (action_list_.size() == 0)
        return false;
    
    for (int i = 1; i < action_list_.size(); ++i) {
        DebugUtility::Print(DebugUtility::Normal, "action " + boost::lexical_cast<std::string>(i) +
                            ", size = " + boost::lexical_cast<std::string>(action_list_[i].trajectory_.size()));
        if (//action_list_[i].length_ == 0 ||
            action_list_[i].trajectory_.size() == 0)
            return false;
    }
    
    return true;
}

std::vector<int> MM::get_learning_result() {
    std::vector<int> result_edges;
    
    for (int i = 1; i < action_list_.size(); ++i) {
        std::vector<int>& edges = action_list_[i].trajectory_;
        for (int j = 0; j < edges.size(); ++j) {
            result_edges.push_back(edges[j]);
        }
    }
    
    return result_edges;
}

void MM::SaveLearningResultAsGeojson(std::string filename) {
    std::string geojson = "{\"type\": \"FeatureCollection\",\"crs\": { \"type\": \"name\", \"properties\": { \"name\":\"urn:ogc:def:crs:EPSG::3857\" } },\"features\": [";
    
    for (int i = 1; i < action_list_.size(); ++i) {
        std::vector<int>& edges = action_list_[i].trajectory_;
        
        for (int j = 0; j < edges.size(); ++j) {
            BoostLineString line = tree_->GetEdge(edges[j]);
            
            double x1 = line.at(0).get<0>();
            double y1 = line.at(0).get<1>();
            double x2 = line.at(1).get<0>();
            double y2 = line.at(1).get<1>();
            
            geojson += "{\"type\": \"Feature\", \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[" + boost::lexical_cast<std::string>(x1) + "," + boost::lexical_cast<std::string>(y1) + + "], [" + boost::lexical_cast<std::string>(x2) + "," + boost::lexical_cast<std::string>(y2) + + "]]}}";
            
            if (i != action_list_.size() - 1 || j != edges.size() - 1)
                geojson += ",";
        }
    }
    geojson += "]}";
    
    std::ofstream ofs(filename);
    ofs << geojson << std::endl;
    ofs.close();
}