#include "HighLevel/pexHSolver.h"
#include "HighLevel/pexSolver.h"
#include "LowLevel/ApexSearch.h"
#include <algorithm>
#include <fstream>
#include <queue>
#include <random>
#include <set>
#include <sstream>

std::unordered_map<size_t, std::vector<int>> id2coord;
std::string map_name;
std::ofstream output;
using namespace pexh;
static size_t checks = 0, comparisons = 0, cache_hits = 0, reinsertions = 0, representative_changes = 0, pruned = 0;
void require(bool value, const std::string& message) {
    ++checks;
    if (!value) throw std::runtime_error(message);
}
Deadline limit() { return Clock::now() + std::chrono::seconds(20); }
bool dominates(const CostVector& a, const CostVector& b) {
    for (size_t i=0; i<a.size(); ++i) if (a[i]>b[i]) return false;
    return true;
}
bool insert_front(std::vector<CostVector>& front, const CostVector& cost) {
    for (const auto& c : front) if (dominates(c,cost)) return false;
    front.erase(std::remove_if(front.begin(),front.end(),[&](const CostVector& c){return dominates(cost,c);}),front.end());
    front.push_back(cost);
    return true;
}
// Independent exhaustive multiobjective search in joint state space. Each agent
// may commit to staying at its goal, after which its cost stops increasing.
// No CBS, heuristic, approximate merging, or bound helpers are used here.
std::vector<CostVector> oracle(const std::vector<Edge>& edges, size_t vertices,
                              const std::vector<std::pair<size_t,size_t>>& endpoints) {
    const size_t dim=edges.front().cost.size(), n=endpoints.size();
    using State=std::pair<std::vector<size_t>, unsigned>;
    using Entry=std::pair<State,CostVector>;
    std::map<State,std::vector<CostVector>> labels;
    std::queue<Entry> open;
    std::vector<std::vector<Edge>> graph(vertices);
    for (const auto& edge:edges) graph[edge.source].push_back(edge);
    std::vector<size_t> starts;
    for (const auto& e:endpoints) starts.push_back(e.first);
    for (unsigned mask=0; mask<(1u<<n); ++mask) {
        bool valid=true;
        for (size_t x=0;x<n;++x) if ((mask&(1u<<x)) && starts[x]!=endpoints[x].second) valid=false;
        if (valid) { State s{starts,mask}; labels[s].push_back(CostVector(dim,0));open.push({s,CostVector(dim,0)}); }
    }
    std::vector<CostVector> result;
    while (!open.empty()) {
        auto entry=open.front();open.pop();
        auto& front=labels[entry.first];
        if (std::find(front.begin(),front.end(),entry.second)==front.end()) continue;
        if (entry.first.second==(1u<<n)-1) {insert_front(result,entry.second);continue;}
        std::vector<size_t> next(n);
        std::function<void(size_t,unsigned,CostVector)> visit;
        visit=[&](size_t x,unsigned mask,CostVector cost) {
            if (x==n) {
                State state{next,mask};
                if (insert_front(labels[state],cost)) open.push({state,cost});
                return;
            }
            const size_t from=entry.first.first[x];
            std::vector<Edge> moves;
            if (entry.first.second&(1u<<x)) moves.emplace_back(from,from,CostVector(dim,0));
            else moves=graph[from];
            for (const auto& edge:moves) {
                bool collision=false;
                for (size_t y=0;y<x;++y)
                    if (next[y]==edge.target || (entry.first.first[y]==edge.target && next[y]==from)) collision=true;
                if (collision) continue;
                next[x]=edge.target;
                auto total=cost;
                for(size_t o=0;o<dim;++o) total[o]+=edge.cost[o];
                visit(x+1,mask,total);
                if (!(mask&(1u<<x)) && edge.target==endpoints[x].second) visit(x+1,mask|(1u<<x),total);
            }
        };
        visit(0,entry.first.second,entry.second);
    }
    std::sort(result.begin(),result.end());
    return result;
}
std::vector<Edge> grid(size_t width,size_t height,size_t dim,unsigned seed=0) {
    std::mt19937 rng(seed);
    std::vector<Edge> edges;
    for(size_t y=0;y<height;++y)for(size_t x=0;x<width;++x) {
        const size_t v=y*width+x;
        id2coord[v]={static_cast<int>(y),static_cast<int>(x)};
        std::vector<size_t> next{v};
        if(x)next.push_back(v-1);if(x+1<width)next.push_back(v+1);
        if(y)next.push_back(v-width);if(y+1<height)next.push_back(v+width);
        for(size_t to:next) {
            CostVector c(dim,1);
            if(seed)for(auto& value:c)value=1+rng()%5;
            edges.emplace_back(v,to,c);
        }
    }
    return edges;
}
void verify_paths(const std::vector<Edge>& edges, const std::vector<std::pair<size_t,size_t>>& endpoints,
                  const std::vector<Solution>& solutions,double eps) {
    for(const auto& solution:solutions) {
        require(solution.paths.size()==endpoints.size(),"returned agent count");
        CostVector actual(solution.cost.size(),0);
        size_t length=0;
        for(size_t x=0;x<endpoints.size();++x) {
            const auto& path=solution.paths[x];
            require(!path.empty()&&path.front()==endpoints[x].first&&path.back()==endpoints[x].second,"path endpoints");
            length=std::max(length,path.size());
            for(size_t t=1;t<path.size();++t) {
                auto edge=std::find_if(edges.begin(),edges.end(),[&](const Edge& e){return e.source==path[t-1]&&e.target==path[t];});
                require(edge!=edges.end(),"path uses actual graph edge");
                for(size_t o=0;o<actual.size();++o)actual[o]+=edge->cost[o];
            }
        }
        require(actual==solution.cost,"returned cost equals edge-cost sum");
        for(size_t o=0;o<actual.size();++o)
            require(actual[o]<=(1.0L+eps)*solution.apex[o],"representative epsilon bound");
        auto at=[&](size_t x,size_t t){return solution.paths[x][std::min(t,solution.paths[x].size()-1)];};
        for(size_t t=0;t<length;++t)for(size_t x=0;x<endpoints.size();++x)for(size_t y=x+1;y<endpoints.size();++y) {
            require(at(x,t)!=at(y,t),"vertex collision including occupied goals");
            if(t+1<length)require(!(at(x,t)==at(y,t+1)&&at(y,t)==at(x,t+1)),"edge swap");
        }
    }
}
void check_front(std::vector<Edge> edges,size_t vertices,std::vector<std::pair<size_t,size_t>> endpoints,
                 double eps,size_t k,bool cb) {
    ++comparisons;
    auto exact=oracle(edges,vertices,endpoints);
    pexHSolver solver(vertices,endpoints.size(),edges.front().cost.size(),-1,0,20);
    solver.set_eps(eps);solver.set_matching_limit(k);
    solver.set_merging_strategy(cb?MergingStrategy::CONFLICT_BASED:MergingStrategy::MORE_SLACK);
    HSolutionID paths;std::vector<CostVector> costs;LoggerPtr logger=nullptr;
    solver.run(edges,endpoints,paths,costs,logger);
    // Both algorithms share every search/merge rule except the proposed H steps.
    pexHSolver baseline(vertices,endpoints.size(),edges.front().cost.size(),-1,0,20);
    baseline.set_use_heuristic(false);baseline.set_eps(eps);
    baseline.set_merging_strategy(cb?MergingStrategy::CONFLICT_BASED:MergingStrategy::MORE_SLACK);
    HSolutionID baseline_paths;std::vector<CostVector> baseline_costs;
    baseline.run(edges,endpoints,baseline_paths,baseline_costs,logger);
    require(baseline.completed(),"shared baseline completes");
    verify_paths(edges,endpoints,baseline.solutions(),eps);
    require(baseline.statistics().analyses==0&&baseline.statistics().scalar_searches==0,"baseline does not run H analysis");
    for(const auto& optimum:exact) {
        bool covered=false;
        for(const auto& c:baseline_costs) {
            bool yes=true;
            for(size_t o=0;o<c.size();++o)if(c[o]>(1.0L+eps)*optimum[o])yes=false;
            covered|=yes;
        }
        require(covered,"baseline epsilon-covers oracle");
    }
    if(eps==0) {
        std::vector<CostVector> actual;
        for(const auto& c:baseline_costs)insert_front(actual,c);
        std::sort(actual.begin(),actual.end());
        require(actual==exact,"baseline exact Pareto front equals oracle");
    }
    // Optimization changes how exact scalar bounds are computed, not their values.
    pexHSolver uncached(vertices,endpoints.size(),edges.front().cost.size(),-1,0,20);
    uncached.set_eps(eps);uncached.set_matching_limit(k);uncached.set_scalar_optimizations(false);
    uncached.set_low_level_cache(false);
    uncached.set_merging_strategy(cb?MergingStrategy::CONFLICT_BASED:MergingStrategy::MORE_SLACK);
    HSolutionID uncached_paths;std::vector<CostVector> uncached_costs;
    uncached.run(edges,endpoints,uncached_paths,uncached_costs,logger);
    require(uncached.completed()&&uncached_costs==costs&&uncached_paths==paths,"optimized exact bounds preserve H costs and paths");
    require(solver.statistics().low_level_queries == solver.statistics().low_level_searches + solver.statistics().low_level_cache_hits,
            "low-level cache accounts for every query");
    if (cb) require(solver.statistics().low_level_cache_hits == 0,"CAT-dependent path sets are not cached by constraints alone");
    cache_hits += solver.statistics().analysis_cache_hits;
    reinsertions += solver.statistics().lazy_reinsertions;
    representative_changes += solver.statistics().representative_changes;
    pruned += solver.statistics().nodes_pruned + solver.statistics().children_pruned;
    require(solver.completed(),"small oracle case completes");
    verify_paths(edges,endpoints,solver.solutions(),eps);
    require(exact.empty()==costs.empty(),"oracle feasibility agrees");
    for(const auto& optimum:exact) {
        bool covered=false;
        for(const auto& cost:costs) {
            bool yes=true;
            for(size_t o=0;o<cost.size();++o)if(cost[o]>(1.0L+eps)*optimum[o])yes=false;
            covered|=yes;
        }
        require(covered,"every exact Pareto vector is epsilon-covered");
    }
    if(eps==0) {
        std::vector<CostVector> actual;
        for(const auto& c:costs)insert_front(actual,c);
        std::sort(actual.begin(),actual.end());
        require(actual==exact,"epsilon zero matches exhaustive Pareto front");
    }
}
void bound_checks() {
    struct LegacyAccess : pexSolver {
        LegacyAccess() : pexSolver(3,1,BBMOCBS_PEX,false,2,-1,0,10) { set_eps(0); }
        using pexSolver::post_process;
    } legacy;
    std::vector<CostVector> legacy_apex{{10,10},{11,11},{5,20}}, legacy_cost=legacy_apex;
    HSolutionID legacy_paths{{{0}},{{1}},{{2}}};
    legacy.post_process(legacy_apex, legacy_cost, legacy_paths);
    require(legacy_cost==std::vector<CostVector>{{5,20},{10,10}},"legacy filtered costs retain original indices");
    require(legacy_paths==HSolutionID{{{2}},{{0}}},"legacy output paths stay aligned with sorted costs");
    PairBound p;p.delta={CostVector{1,1},CostVector{2,2}};
    auto full=full_bounds({0,0},{p});
    require(nondominated({full[0].cost,full[1].cost})==std::vector<CostVector>{{1,1}},"global dominance fixture");
    require(child_bounds(full,1)==std::vector<CostVector>{{2,2}},"child must use full labeled set");
    std::vector<Solution> s{{{12,12},{10,10},{}},{{5,20},{5,18},{}}};
    require(!bound_covered({{10,10},{1,1}},s,0),"partial coverage is false");
    require(s[0].apex==CostVector({12,12}),"failed coverage leaves apex unchanged");
    require(bound_covered({{10,10},{5,18}},s,0),"different witnesses may cover different bounds");
    require(s[0].apex==CostVector({10,10})&&s[1].apex==CostVector({5,18}),"coverage updates both apexes");
    require(bound_covered({},s,0),"empty infeasible side is covered");
    p.delta[1]={MAX_COST,MAX_COST};
    require(child_bounds(full_bounds({0,0},{p}),1).empty(),"infeasible side removed without overflow");
    std::vector<Solution> merged{{{10,20},{10,20},{{0,1}}}};
    add_solution({{12,18},{12,18},{{0,2}}},merged,0.2);
    require(merged.size()==1&&merged[0].apex==CostVector({10,18}),"AddSolution epsilon merge");
    bool overflow=false;try {checked_add(MAX_COST-1,1);}catch(const std::overflow_error&){overflow=true;}
    require(overflow,"cost overflow fails explicitly");
}
void constrained_checks() {
    auto edges=grid(3,1,2);AdjacencyMatrix graph(3,edges);
    VertexConstraint v;EdgeConstraint e;
    v[2]={1};
    require(exact_shortest_path(graph,0,1,0,v,e,-1,0,limit())==3,"goal forbidden after earliest arrival");
    e[0][0]={1};v.clear();
    require(exact_shortest_path(graph,0,2,0,v,e,-1,0,limit())==3,"directed departure-time edge constraint");
    v[0]={0};
    require(exact_shortest_path(graph,0,2,0,v,e,-1,0,limit())==MAX_COST,"time-zero infeasibility");
    bool timeout=false;try {exact_shortest_path(graph,0,1,0,{}, {},-1,0,Clock::now());}catch(const SearchTimeout&){timeout=true;}
    require(timeout,"deadline is distinct from infeasibility");
    auto tiny=grid(2,1,2);AdjacencyMatrix small(2,tiny);
    v.clear();v[2]={0};e.clear();
    require(exact_shortest_path(small,0,0,0,v,e,-1,0,limit())==3,"start-at-goal may leave and return");
    auto dist=reverse_distances(small,0,limit());
    Heuristic h=[&](size_t x,size_t,bool){return dist[x];};
    ApexSearch apex(small,::EPS(2,0),::EPS(2,0),-1,0);
    apex.set_merge_strategy(MORE_SLACK);apex.set_constraint_safe_search(limit());
    PathSet paths;CostSet ac,rc;VertexCAT vc;EdgeCAT ec;std::unordered_map<int,int> cm;
    apex(paths,ac,rc,0,0,h,v,e,20,vc,ec,cm);
    require(!paths.empty(),"approx search returns a constrained goal path");
    for(const auto& p:paths)require(p.second.size()>3&&p.second[2]!=0,"approx search respects future goal constraint");
    require(conflicts({{0},{1,0}}).front().a==0,"goal collision keeps both agent ids");
    require(conflicts({{0,1},{1,0}}).front().vertices.size()==2,"swap is edge conflict");
    require(conflicts({{0,1,2},{1,0,2}}).front().time==0,"first pair conflict is chronological");
    e.clear();e[0][0]={0};v.clear();
    require(exact_shortest_path(small,0,0,0,v,e,-1,0,limit())==2,"future implicit goal wait edge checked");
    require(exact_shortest_path(graph,0,2,0,{}, {},0,5,limit())==7,"exact search uses upstream first-move turn convention");
}
void matching_checks() {
    auto edges=grid(2,2,2);auto other=edges;
    for(auto edge:other){edge.source+=4;edge.target+=4;edges.push_back(edge);}
    AdjacencyMatrix graph(8,edges);
    HighLevelNode node(4);
    std::vector<std::pair<size_t,size_t>> endpoints{{0,1},{1,0},{4,5},{5,4}};
    std::vector<std::vector<size_t>> rep{{0,1},{1,0},{4,5},{5,4}};
    for(size_t x=0;x<4;++x)node.sop_apex[x][0]={1,1};
    size_t searches=0;
    auto a=analyze(node,rep,graph,endpoints,2,-1,0,limit(),searches);
    require(a.matching.size()==2&&a.full.size()==4,"K=2 disjoint cardinal pairs generate four bounds");
    require(a.base==CostVector({4,4}),"base is sum of per-agent component minima");
    require(a.matching[0].delta[0]==CostVector({1,1}),"exact cardinal delta");
    require(searches==8,"one scalar search per pair side objective");
    auto b=analyze(node,rep,graph,endpoints,1,-1,0,limit(),searches);
    require(b.matching.size()==1,"K=1 matching limit");
    // Independently varying objective minima must not use a lexicographic path minimum.
    node.sop_apex[0][0]={1,5};node.sop_apex[0][1]={5,1};
    auto c=analyze(node,rep,graph,endpoints,2,-1,0,limit(),searches);
    require(c.base==CostVector({4,4}),"componentwise ideal uses all paths");
}
void edge_cases() {
    auto run = [&](std::vector<Edge> edges, size_t vertices, std::vector<std::pair<size_t,size_t>> endpoints) {
        pexHSolver solver(vertices,endpoints.size(),edges.front().cost.size(),-1,0,5);
        HSolutionID paths;std::vector<CostVector> costs;LoggerPtr logger=nullptr;
        solver.run(edges,endpoints,paths,costs,logger);
        require(solver.completed(),"infeasible/single-agent case terminates");
        return solver.solutions();
    };
    require(run({Edge(0,0,{1,1}),Edge(1,1,{1,1})},2,{{0,1}}).empty(),"disconnected root is infeasible");
    require(run(grid(2,2,2),4,{{0,2},{1,2}}).empty(),"shared persistent goal is infeasible");
    require(run(grid(2,2,2),4,{{0,1},{0,2}}).empty(),"shared initial vertex is infeasible");
    auto stationary=run(grid(2,2,2),4,{{0,0}});
    require(stationary.size()==1&&stationary[0].cost==CostVector({0,0}),"start at goal costs zero");
    // Compare exact scalar search with a separate finite-time dynamic program.
    // On these four-vertex graphs the 18-step enumeration includes a simple
    // orientation-state suffix after all restrictions (latest restriction t=2).
    for(unsigned seed=1;seed<=6;++seed) {
        auto edges=grid(2,2,3,seed);AdjacencyMatrix graph(4,edges);
        VertexConstraint vertices;vertices[2]={3};
        EdgeConstraint restrictions;restrictions[0][0]={1};
        for(size_t objective=0;objective<3;++objective) {
            using State=std::pair<size_t,size_t>;
            std::map<State,size_t> level{{{0,MAX_COST},0}};
            size_t best=MAX_COST;
            for(size_t time=0;time<=18;++time) {
                std::map<State,size_t> next;
                for(const auto& item:level) {
                    auto vertex=item.first.first, previous=item.first.second;
                    if(vertex==3&&time>2)best=std::min(best,item.second);
                    for(const auto& edge:edges)if(edge.source==vertex) {
                        if(time+1==2&&edge.target==3)continue;
                        if(time==0&&vertex==0&&edge.target==1)continue;
                        size_t cost=item.second+edge.cost[objective];
                        if(objective==1) {
                            auto from=id2coord.at(vertex),to=id2coord.at(edge.target);
                            bool turn=previous==MAX_COST?vertex!=edge.target:
                                (from[0]-id2coord.at(previous)[0]!=to[0]-from[0] ||
                                 from[1]-id2coord.at(previous)[1]!=to[1]-from[1]);
                            if(turn)cost+=2;
                        }
                        State state{edge.target,vertex};
                        auto it=next.find(state);
                        if(it==next.end()||cost<it->second)next[state]=cost;
                    }
                }
                level=std::move(next);
            }
            auto exact=exact_shortest_path(graph,0,3,objective,vertices,restrictions,1,2,limit());
            require(exact==best,"scalar Dijkstra matches independent timed DP with turn costs");
            AdjacencyMatrix inverse(4,edges,true);
            auto lower=reverse_distances(inverse,3,limit());
            auto astar=exact_shortest_path(graph,0,3,objective,vertices,restrictions,1,2,limit(),&lower);
            require(astar==best,"exact scalar A-star matches independent timed DP with turn costs");
        }
    }
}
int main() {
    try {
        // Silence upstream expansion progress while retaining actionable failure output.
        std::ostringstream quiet;
        auto* original=std::cout.rdbuf(quiet.rdbuf());
        try {
            bound_checks();constrained_checks();matching_checks();edge_cases();
            for(size_t dim:{2u,3u})for(unsigned seed=1;seed<=8;++seed)
                for(double eps:{0.0,0.1,0.5})for(bool cb:{false,true})
                    check_front(grid(2,2,dim,seed),4,{{0,1},{1,0}},eps,1+(seed%2),cb);
            check_front(grid(3,2,2,13),6,{{0,2},{2,0}},0,2,false);
            check_front(grid(2,2,2,3),4,{{0,0},{1,2}},0,1,false);
            check_front(grid(2,2,2),4,{{0,1},{1,0}},0,1,false);
            auto disconnected=grid(2,2,2);auto copy=disconnected;
            for(auto edge:copy){edge.source+=4;edge.target+=4;disconnected.push_back(edge);}
            check_front(disconnected,8,{{0,1},{1,0},{4,5},{5,4}},0,2,false);
            check_front(disconnected,8,{{0,1},{1,0},{4,5},{5,4}},0.5,2,true);
            for (unsigned seed : {3u, 17u}) {
                auto travel=grid(2,2,3,seed);
                for (auto& edge : travel) edge.cost[2]=1;
                check_front(travel,4,{{0,1},{1,0}},0,1,false);
                check_front(travel,4,{{0,1},{1,0}},0.05,1,false);
            }
            auto zero=grid(2,2,2);
            for(auto& edge:zero)edge.cost={0,0};
            check_front(zero,4,{{0,1},{1,0}},0,1,false);
            require(cache_hits>0&&cache_hits<=reinsertions,"priority-only reinsertions reuse cached analyses");
            require(representative_changes>0,"representative changes exercise cache invalidation");
            require(pruned>0,"bound pruning exercised by integration cases");
        } catch (...) {std::cout.rdbuf(original);throw;}
        std::cout.rdbuf(original);
        std::cout<<"PASS: "<<checks<<" assertions, including "<<comparisons<<" exhaustive-oracle comparisons\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';
        return 1;
    }
}
