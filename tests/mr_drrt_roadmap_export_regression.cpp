// Regression coverage for the viewer roadmap-visualization feature's
// simulation-side pieces: MRdRRT::numRoadmaps()/roadmap() (public accessor
// added so apps can reach the per-robot PRM* roadmaps after solve()) and
// benchmark_apps::common::appendRoadmaps's undirected-edge deduplication
// (Roadmap::adjacency lists both directions of each edge; a naive dump
// would double every edge in the exported "edges" array).
#include "benchmark_app_common.hpp"
#include "comotion/planning/MRdRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/PlannerStatus.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace common = comotion::benchmark_apps::common;
using json = nlohmann::json;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphere() {
    return std::make_shared<comotion::FlyingSphere>(
        0.1, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

// Same tiny 2-robot direct-crossing setup as
// mr_drrt_metrics_regression.cpp's makeDirectProblem().
std::shared_ptr<comotion::MultiRobotProblem> makeDirectProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    problem->addRobot(makeSphere(), {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphere(), {0.0, 5.0, 0.0}, {0.0, 6.0, 0.0});
    return problem;
}

bool expectTrue(const std::string &label, bool value) {
    if (!value)
        std::cerr << "mr_drrt_roadmap_export_regression: " << label << "\n";
    return value;
}

bool runAccessorCase() {
    comotion::seedOmplGlobalFromUserPlanningSeed(201);
    auto problem = makeDirectProblem();

    comotion::MRdRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(201);
    planner.setRoadmapSize(8);
    planner.setIterationsPerBatch(1);

    if (!expectTrue("numRoadmaps is zero before solve()",
                    planner.numRoadmaps() == 0))
        return false;

    // Exact tensor-search success isn't required for this test; Phase 1
    // (roadmap construction) always runs regardless of Phase 2's outcome.
    planner.solve(1.0);

    if (!expectTrue("numRoadmaps equals robot count after solve()",
                    planner.numRoadmaps() ==
                        static_cast<std::size_t>(problem->numRobots())))
        return false;

    for (std::size_t r = 0; r < planner.numRoadmaps(); ++r) {
        const auto view = planner.roadmap(r);
        if (!expectTrue("roadmap has at least start+goal vertices",
                        view.vertices.size() >= 2))
            return false;
        if (!expectTrue("adjacency is sized per-vertex",
                        view.adjacency.size() == view.vertices.size()))
            return false;
        if (!expectTrue("start_vertex is the query-start milestone (index 0)",
                        view.start_vertex == 0))
            return false;
        if (!expectTrue("goal_vertex is the query-goal milestone (index 1)",
                        view.goal_vertex == 1))
            return false;
        for (const auto &vertex : view.vertices) {
            if (!expectTrue("sphere robot vertex is a 3D config",
                            vertex.size() == 3))
                return false;
        }
    }

    bool threw_out_of_range = false;
    try {
        planner.roadmap(planner.numRoadmaps());
    } catch (const std::out_of_range &) {
        threw_out_of_range = true;
    }
    if (!expectTrue(
            "roadmap() throws std::out_of_range for an invalid robot_index",
            threw_out_of_range))
        return false;

    return true;
}

bool runEdgeDedupCase() {
    comotion::seedOmplGlobalFromUserPlanningSeed(202);
    auto problem = makeDirectProblem();

    auto planner = std::make_shared<comotion::MRdRRT>();
    planner->setProblem(problem);
    planner->setPlanningSeed(202);
    planner->setRoadmapSize(10);
    planner->setIterationsPerBatch(1);
    planner->solve(1.0);

    json result;
    result["robots"] = json::array();
    for (int r = 0; r < problem->numRobots(); ++r)
        result["robots"].push_back({{"robot_type", "sphere"}});

    common::appendRoadmaps(
        result, std::static_pointer_cast<comotion::MultiRobotPlanner>(planner),
        /*output_paths=*/true, /*output_roadmaps=*/true);

    if (!expectTrue("appendRoadmaps sets the \"roadmaps\" key",
                    result.contains("roadmaps")))
        return false;
    if (!expectTrue(
            "one roadmap entry per sphere robot",
            result["roadmaps"].size() ==
                static_cast<std::size_t>(problem->numRobots())))
        return false;

    for (const auto &entry : result["roadmaps"]) {
        const int robot_index = entry.at("robot_index").get<int>();
        const auto view =
            planner->roadmap(static_cast<std::size_t>(robot_index));

        // Independently recompute the expected deduplicated undirected edge
        // set straight from the roadmap's raw (bidirectional) adjacency
        // lists, so this doesn't just re-derive appendRoadmaps's own math.
        std::set<std::pair<int, int>> expected;
        std::size_t raw_adjacency_entries = 0;
        for (std::size_t u = 0; u < view.adjacency.size(); ++u) {
            for (int v : view.adjacency[u]) {
                ++raw_adjacency_entries;
                expected.emplace(std::min(static_cast<int>(u), v),
                                 std::max(static_cast<int>(u), v));
            }
        }

        const auto &edges_json = entry.at("edges");
        if (!expectTrue("edges count matches the deduplicated expectation",
                        edges_json.size() == expected.size()))
            return false;
        // OMPL PlannerData::getEdges lists undirected edges symmetrically
        // (both endpoints see each other); a naive un-deduplicated dump
        // would therefore be exactly 2x the deduplicated count.
        if (!expectTrue(
                "deduplication actually halved a symmetric adjacency dump",
                raw_adjacency_entries == 2 * edges_json.size()))
            return false;

        std::set<std::pair<int, int>> seen;
        for (const auto &edge : edges_json) {
            if (!expectTrue("edge entry has exactly two endpoints",
                            edge.size() == 2))
                return false;
            const int a = edge.at(0).get<int>();
            const int b = edge.at(1).get<int>();
            if (!expectTrue("edge is normalized as (min, max)", a <= b))
                return false;
            const auto normalized = std::make_pair(a, b);
            if (!expectTrue("no duplicate {a,b}/{b,a} pair in edges",
                            seen.insert(normalized).second))
                return false;
            if (!expectTrue("emitted edge is one of the expected pairs",
                            expected.count(normalized) == 1))
                return false;
        }
    }

    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok &= runAccessorCase();
    ok &= runEdgeDedupCase();
    return ok ? 0 : 1;
}
