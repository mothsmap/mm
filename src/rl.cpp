#include "rl.h"
#include "rl_utility.h"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>


RLAlgorithm::RLAlgorithm(int states_number, int actions_number) {
    Init(states_number, actions_number);
}

void RLAlgorithm::Init(int states_number, int actions_number) {
#ifdef WIN32
	srand(time(0));
#else
	srand48(clock());
#endif
	
#ifdef _TRACE_LOG
    std::cout << "Init q table: " << states_number << " * " << actions_number << std::endl;
#endif

    state_number_ =  states_number;
    action_number_ = actions_number;

    q_.clear();
    q_.allocate(state_number_, action_number_, CNCSparseMatrix<double>::ROWS, false);

    //	policy_ = new double[action_number_];
}

RLAlgorithm::~RLAlgorithm() {
}

double RLAlgorithm::GetQValue(int state, int action) {
    const CNCSparseRowColumn<double>& row = q_.row(state);

    // Search for a_{index}
    for(long ii = 0; ii < row.nb_coeffs(); ii++)
    {
        const CNCCoeff<double>& coeff = row.coeff(ii);

        if(coeff.index == action)
        {
            return coeff.a;
        }
    }

    return 0;

}

void RLAlgorithm::Explore(const int state, int& action, const double exploration_rate, const std::vector<int>& action_index)
{
    egreedy(state, action, exploration_rate, action_index);
}

//void RLAlgorithm::MaxExplore(const int state, int& action, const std::vector<int>& action_index) {
//	GetMaxActionRandom(state, action, action_index);
//}

void RLAlgorithm::egreedy(const int state, int& action, const double epsilon, const std::vector<int>& action_index)
{
#ifdef WIN32
   if(RLUtility::RandUnit() < epsilon)
#else
   if (drand48() < epsilon)
#endif
    {
        GetMaxAction(state, action, action_index);
    }
    else
    {
        GetRandomAction(state, action, action_index) ;
    }
}
void RLAlgorithm::GetMaxAction(const int state, int& action, const std::vector<int>& action_index)
{
    GetMaxActionRandom(state, action, action_index);
}

void RLAlgorithm::GetMaxActionRandom(const int state, int& action, const std::vector<int>& action_index)
{
    const CNCSparseRowColumn<double>& row = q_.row(state);

    if(row.nb_coeffs() > 0)
    {
        std::vector<int> max_all = RLUtility::ArgMaxAll(row);
        action = max_all[RLUtility::Rand0A(max_all.size())];
    }
    else
    {
        GetRandomAction(state, action, action_index);
    }
}

void RLAlgorithm::GetMaxActionFirst(const int state, int& action, const std::vector<int>& action_index)
{
    const CNCSparseRowColumn<double>& row = q_.row(state);

    if(row.nb_coeffs() > 0)
        action = RLUtility::ArgMax(row);
    else
        GetRandomAction(state, action, action_index);
}

void RLAlgorithm::GetRandomAction(const int state, int& action, const std::vector<int>& action_index)
{
#ifdef WIN32
    int index = RLUtility::Rand0A(action_index.size());
#else
    int index = drand48() * (action_index.size() - 1);
#endif
    action = action_index[index];
}

void RLAlgorithm::Update(const int state, const int action, const double reward,
                         const int next_state, const bool end_of_episode,
                         const double learning_rate, const double gamma)
{

    double old_qvalue = this->GetQValue(state, action);
    double add_value = 0;

    if(end_of_episode)
    {
        add_value = learning_rate * (reward - old_qvalue);
    }
    else
    {
        const CNCSparseRowColumn<double>& row = q_.row(next_state);
        double max_q_value = RLUtility::Max(row) ;
        add_value = learning_rate * (reward + gamma * max_q_value - old_qvalue);
    }

    q_.add(state, action, add_value);
}

void RLAlgorithm::SaveQTable(const char* filename)
{
    std::ofstream ofs(filename, std::ofstream::out);

    ofs << q_.m() << std::endl;

    for(int i = 0; i < q_.m(); ++i)
    {
        const CNCSparseRowColumn<double>& row = q_.row(i);
        ofs << "\t" << row.nb_coeffs() << std::endl;

        for(int j = 0; j < row.nb_coeffs(); ++j)
        {
            const CNCCoeff<double>& coeff = row.coeff(j);
            ofs << "\t\t" << coeff.index << "\t" << coeff.a << std::endl;
        }
    }

    ofs.close();
}

void RLAlgorithm::LoadQTable()
{
    std::ifstream ifs(qtable_path.c_str(), std::ifstream::in);

    int m;
    ifs >> m;
    assert(m == q_.m());

    for(int i = 0; i < q_.m(); ++i)
    {
        int items;
        ifs >> items;

        for(int j = 0; j < items; ++j)
        {
            double index, a;
            ifs >> index >> a;
            q_.add(i, index, a);
        }
    }

    ifs.close();
}
