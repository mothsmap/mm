#ifndef __mm__hh__
#define __mm__hh__

#include <string>

class MMSolver {
public:
    MMSolver();
    ~MMSolver();
    
    bool Prepare(std::string node,
		 std::string edge,
		 std::string history,
		 std::string out_dir,
		 double xmin,
		 double ymin,
		 double xmax,
		 double ymax) ;
    
    bool Match(std::string preprocess,
               std::string in,
               std::string out,
               std::string ground_truth = "");
};

#endif
