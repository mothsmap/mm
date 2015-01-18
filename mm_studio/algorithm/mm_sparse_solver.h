#ifndef __mm_sparse__solver__hh
#define __mm_sparse__solver__hh


#include "mm_graph.h"
#include "mm_route.h"
#include "mm_tree.h"
#include "rl.h"

#include <map>
#include <stack>

typedef ShortestPath Action;
struct State {
//public:
//    State() {}
//    State(int gps_id, candidate_id) { gps_id_ = gps_id; candidate_id_ = candidate_id_; }
//    
    int gps_id_;
    int candidate_id_;

    State& operator= (const State& state) {
        gps_id_ = state.gps_id_;
        candidate_id_ = state.candidate_id_;

        return *this;
    }
    
    bool operator== (const State& state) const {
        return (gps_id_ == state.gps_id_ && candidate_id_ == state.candidate_id_);
    }
    
    bool operator!= (const State& state) const {
        return (gps_id_ != state.gps_id_ || candidate_id_ != state.candidate_id_);
    }
    
    bool operator> (const State& state) const{
        if(gps_id_ == state.gps_id_)
            return (candidate_id_ > state.candidate_id_);
        else
            return (gps_id_ > state.gps_id_);
    }
    
    bool operator< (const State& state) const {
        if(gps_id_ == state.gps_id_)
            return (candidate_id_ < state.candidate_id_);
        else
            return (gps_id_ < state.gps_id_);
    }
};

// This is the agent in RL
class MM : public RLAlgorithm {
public:
    MM();
    bool InitModel(boost::shared_ptr<RTree> tree, boost::shared_ptr<Route> route, std::vector<std::vector<wxPoint2DDouble> >& candidate_points, std::vector<ShortestPath>& shortest_paths);

    ~MM();

    // Run the parsing algorithm.
    double RunParsingAlgorithm(int episodes, int greedy_episodes, double learning_rate);
    
    void SaveLearningResult(std::string path, double score);
    
    inline int get_states_number() { return states_number_; }
    
    inline int get_actions_number() { return actions_number_;}
    
    inline int get_parsing_action_size() { return action_list_.size(); }
    inline std::vector<int>& get_parsing_action(int index) {
        return action_list_[index].path_; }
    
    inline std::vector<Action>& get_parsing_action() { return action_list_; }
    //inline std::vector<State>& get_parsing_result() { return parsing_result_; }
    bool parsing_result_valid();
    
    void Clear();
protected:
    //! Reset the model.
    void Reset();

    //! Run a learning sub-task.
    double RunTask(State& state);

    // Initialize the state-index and index-state map.
    int InitStateMap(std::vector<std::vector<wxPoint2DDouble> >& candidate_points);

    // Initialize the action-index and index-action map.
    int InitActionMap(std::vector<ShortestPath>& shortest_paths);
    
    // Initialize the index-suitable actions map.
    bool InitSymbolSuitableActions();

private:
    boost::shared_ptr<RTree> tree_;
    boost::shared_ptr<Route> route_;
    double max_route_length_, min_route_length_,  max_candidate_distance_, min_candidate_distance_;
    
    int terminal_state_;
    State current_state_;                           /* current state. */
    int initial_state_size_;
    int states_number_, actions_number_;            /* Number of state and action. */

    int episodes_;                                  /* Number of episodes. */
    bool learning_, record_;                        /* Whether to learning, record result. */
    double epsilon_, learning_rate_, gamma_;        /* learing parameters. */

    std::map<Action, int> action_to_index_;         /* action-index map. */
    std::map<int, Action> index_to_action_;         /* index-action map. */
    std::map<State, int> state_to_index_;           /* state-index map. */
    std::map<int, State> index_to_state_;           /* index-state map. */
    std::map<int, std::vector<int> > suitable_actions_for_state_;

  //  std::vector<State> parsing_result_;
    std::vector<Action> action_list_;
};

#endif // FACADE_MODEL_H
