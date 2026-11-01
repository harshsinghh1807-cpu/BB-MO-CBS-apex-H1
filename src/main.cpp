#include <iostream>

#include <memory>
#include <time.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <dirent.h>
#include <string>

#include "LowLevel/Utils/Logger.h"
#include "HighLevel/HighLevelSolver.h"
#include "HighLevel/epsSolver.h"
#include "HighLevel/pexSolver.h"
#include "HighLevel/kSolver.h"
#include "HighLevel/pexHSolver.h"
#include <cmath>
#include <iomanip>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

// using namespace std;

std::unordered_map<size_t, std::vector<int>> id2coord;  // node_id to coordinate
std::string map_name;
std::ofstream output;

int main(int argc, char** argv)
{
try {

namespace po = boost::program_options;
// Declare the supported options.
po::options_description desc("Allowed options");
desc.add_options()
    ("help", "produce help message")
    ("map_file,m", po::value<std::string>()->required(), "Map File")
    ("agent_num,n", po::value<int>()->default_value(-1), "number of agents")
    ("dim,d", po::value<int>()->required(), "cost dimension")
    ("scenario_file,s", po::value<std::string>()->required(), "Scenario file")
    ("eps,e", po::value<double>()->default_value(0), "Approximation factor")
    // ("eps_ratio,r", po::value<double>()->default_value(0.8), "eps_HighLevelMerge divided by eps")
    ("algorithm,a", po::value<std::string>()->default_value("BBMOCBS-pex-H"), "algorithm")
    ("c1", po::value<std::string>()->required(), "cost map 1")
    ("c2", po::value<std::string>()->default_value(""), "cost map 2")
    ("c3", po::value<std::string>()->default_value(""), "cost map 3")
    ("CB", po::value<std::string>()->default_value("false"), "if conflict-based")
    ("eager", po::value<std::string>()->default_value("false"), "eager solutions (legacy solvers only)")
    ("matching_limit,K", po::value<int>()->default_value(1), "maximum disjoint conflict pairs for pex-H (K >= 1)")
    ("scalar_optimizations", po::value<std::string>()->default_value("true"), "exact A-star and scalar memoization for H (false for validation) ")
    ("low_level_cache", po::value<std::string>()->default_value("true"), "memoize identical low-level searches when CB=false")
    ("paths_file", po::value<std::string>()->default_value(""), "JSON solution paths, costs, apexes and completion status (shared pex solvers)")

    ("turn_dim", po::value<int>()->default_value(-1), "dimension with turn cost")
    ("turn_cost", po::value<int>()->default_value(5), "extra cost for turn action")
    
    ("solution_num,k", po::value<int>()->default_value(INT_MAX), "number of solution")
    ("time_limit,t", po::value<int>()->default_value(120), "cutoff time (seconds)")
    
    ("output_file,o", po::value<std::string>()->required(), "Name of the output file")
    ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
    ;
po::variables_map vm;
po::store(po::parse_command_line(argc, argv, desc), vm);

if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
}

po::notify(vm);
if (vm["agent_num"].as<int>() < 1 || vm["dim"].as<int>() < 2 || vm["dim"].as<int>() > 3 ||
    vm["time_limit"].as<int>() < 1 || vm["matching_limit"].as<int>() < 1 ||
    !std::isfinite(vm["eps"].as<double>()) || vm["eps"].as<double>() < 0 ||
    vm["turn_dim"].as<int>() < -1 || vm["turn_dim"].as<int>() >= vm["dim"].as<int>() || vm["turn_cost"].as<int>() < 0)
    throw std::invalid_argument("require n >= 1, d = 2 or 3, t >= 1, K >= 1, finite eps >= 0, and valid turn settings");
for (const std::string option : {"CB", "eager", "scalar_optimizations", "low_level_cache"})
    if (vm[option].as<std::string>() != "true" && vm[option].as<std::string>() != "false")
        throw std::invalid_argument(option + " must be true or false");
const auto algorithm = vm["algorithm"].as<std::string>();
const bool is_h = algorithm == "BBMOCBS-pex-H" || algorithm == "BBMOCBS-pex-H1" || algorithm == "BBMOCBS-apex-H1";
const bool is_shared = is_h || algorithm == "BBMOCBS-pex";
if (is_shared && vm["eager"].as<std::string>() == "true")
    throw std::invalid_argument("shared pex solvers follow the supplied non-eager algorithm; use --eager false (or select BBMOCBS-pex-legacy)");
if (!is_shared && !vm["paths_file"].as<std::string>().empty())
    throw std::invalid_argument("--paths_file is supported by the shared pex solvers only");

std::unique_ptr<Logger> logger_owner;
LoggerPtr logger = nullptr;
if (vm["logging_file"].as<std::string>() != ""){
    logger_owner = std::make_unique<Logger>(vm["logging_file"].as<std::string>());
    logger = logger_owner.get();
}

output.open(vm["output_file"].as<std::string>(), std::ios::app);
if (!output) throw std::runtime_error("cannot open output file");
output.seekp(0, std::ios::end);
/********************  INPUT CONFIG  *********************/
// init preprocessor
PreProcessor p;

// import configures
int dim = vm["dim"].as<int>();
double eps = vm["eps"].as<double>();
MergingStrategy ms = vm["CB"].as<std::string>() == "true" ? MergingStrategy::CONFLICT_BASED : MergingStrategy::MORE_SLACK;
bool if_eager = vm["eager"].as<std::string>() == "true" ? true : false;
int turn_dim = vm["turn_dim"].as<int>();
int turn_cost = vm["turn_cost"].as<int>();

// import map
Map map;
map_name = vm["map_file"].as<std::string>();
p.read_map(map_name, map, id2coord);

// import scenario file
std::vector<std::pair<size_t, size_t>>  start_goal;
p.read_scenario(vm["scenario_file"].as<std::string>(), map , vm["agent_num"].as<int>(), start_goal);

// import cost file
std::vector<Edge>  edges;
p.cost_init(map, edges, dim);
for(int i = 0; i < dim; i ++){
    std::string temp = "c" + std::to_string(i+1);
    p.read_cost(vm[temp].as<std::string>(), map, edges, i);
}

// initialize solver
std::unique_ptr<HighLevelSolver> h_solver;
if (is_shared) {
    auto solver = std::make_unique<pexHSolver>(map.graph_size, vm["agent_num"].as<int>(), dim, turn_dim, turn_cost, vm["time_limit"].as<int>());
    solver->set_eps(eps);
    solver->set_use_heuristic(is_h);
    solver->set_scalar_optimizations(vm["scalar_optimizations"].as<std::string>() == "true");
    solver->set_low_level_cache(vm["low_level_cache"].as<std::string>() == "true");
    solver->set_matching_limit(vm["matching_limit"].as<int>());
    solver->set_merging_strategy(ms);
    h_solver = std::move(solver);
} else if(vm["algorithm"].as<std::string>() == "BBMOCBS-eps"){
    h_solver = std::make_unique<epsSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_EPS, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((epsSolver*)h_solver.get())->set_eps(eps);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-pex-legacy"){
    h_solver = std::make_unique<pexSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_PEX, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((pexSolver*)h_solver.get())->set_eps(eps);
    ((pexSolver*)h_solver.get())->set_merging_strategy(ms);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-k"){
    h_solver = std::make_unique<kSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_K, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((kSolver*)h_solver.get())->set_merging_strategy(ms);
    ((kSolver*)h_solver.get())->set_solution_num(vm["solution_num"].as<int>());
}else{
    output << std::endl << std::endl << vm["algorithm"].as<std::string>() + " is not an allowed algorithm";
    exit(1);
}



HSolutionID        hsolution_ids;
std::vector<CostVector>    hsolution_costs;

auto result = h_solver->run(edges, start_goal, hsolution_ids, hsolution_costs, logger);

double HLMergingTime = 0, LowLevelTime = 0, TotalTime = 0;
int ConflictSolvingNum = 0, SolutionNum = 0;

// success_num ++;
HLMergingTime = std::get<0>(result);
LowLevelTime = std::get<1>(result);
TotalTime = std::get<2>(result);
ConflictSolvingNum = std::get<3>(result);
SolutionNum = std::get<4>(result);

// output << std::endl;
output << std::endl;
output << "HLMergingTime = " << HLMergingTime << std::endl;
output << "LowLevelTime = " << LowLevelTime << std::endl;
output << "Total Time = " << TotalTime << std::endl;
output << "ConflictSolvingNum = " << ConflictSolvingNum << std::endl;
output << "SolutionNum = " << SolutionNum << std::endl;

std::cout << "FINISH ONCE" << std::endl;

output.close();
if (is_shared) {
    const auto& solver = *static_cast<pexHSolver*>(h_solver.get());
    const auto filename = vm["paths_file"].as<std::string>();
    if (!filename.empty()) {
        std::ofstream json(filename);
        if (!json) throw std::runtime_error("cannot open paths JSON file");
        json << std::setprecision(std::numeric_limits<double>::max_digits10);
        json << "{\"algorithm\":\"" << solver.algorithm_name() << "\",\"completed\":" << (solver.completed() ? "true" : "false")
             << ",\"epsilon\":" << eps << ",\"objectives\":" << dim << ",\"matching_limit\":" << vm["matching_limit"].as<int>()
             << ",\"statistics\":{\"total_seconds\":" << TotalTime << ",\"conflict_splits\":" << ConflictSolvingNum
             << ",\"low_level_seconds\":" << LowLevelTime << ",\"merge_seconds\":" << HLMergingTime
             << ",\"analysis_seconds\":" << solver.statistics().analysis_seconds
             << ",\"scalar_seconds\":" << solver.statistics().scalar_seconds
             << ",\"scalar_searches\":" << solver.statistics().scalar_searches
             << ",\"scalar_cache_hits\":" << solver.statistics().scalar_cache_hits
             << ",\"scalar_expansions\":" << solver.statistics().scalar_expansions
             << ",\"low_level_searches\":" << solver.statistics().low_level_searches
             << ",\"low_level_queries\":" << solver.statistics().low_level_queries
             << ",\"low_level_cache_hits\":" << solver.statistics().low_level_cache_hits
             << ",\"analyses\":" << solver.statistics().analyses
             << ",\"children_pruned\":" << solver.statistics().children_pruned
             << ",\"nodes_pruned\":" << solver.statistics().nodes_pruned << "},\"solutions\":[";
        bool first = true;
        for (const auto& solution : solver.solutions()) {
            if (!first) json << ',';
            first = false;
            json << "{\"apex\":" << solution.apex << ",\"cost\":" << solution.cost << ",\"paths\":[";
            for (size_t x = 0; x < solution.paths.size(); ++x) {
                if (x) json << ',';
                json << solution.paths[x];
            }
            json << "]}";
        }
        json << "]}\n";
    }
    if (!solver.completed()) return 2;
}
return 0;
} catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
}
}
