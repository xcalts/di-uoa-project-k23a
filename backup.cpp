// // == Debug `F_Point` Dataset ==
// spdlog::info("+--------------------------------------------------+");
// spdlog::info("| {:<5} | {:<10} | {:<8} | {:<16} |", "Index", "Dimensions", "Category", "Timestamp");
// spdlog::info("+--------------------------------------------------+");
// for (const auto &p : dummyData)
// {
//     spdlog::info("| {:<5} | {:<10} | {:<8} | {:<16} |", p.index, p.dimensions, p.C, p.T);
// }
// spdlog::info("+--------------------------------------------------+");

// == Debug `F_Query` Queries ==
// spdlog::info("+-------------------------------------------------------------------------+");
// spdlog::info("| {:>5} | {:>10} | {:>10} | {:>7} | {:>11} | {:>11} |",
//              "Index", "Dimensions", "QueryType", "v", "l", "r");
// spdlog::info("+-------------------------------------------------------------------------+");
// for (const auto &q : dummyQueries)
// {
//     spdlog::info("| {:>5} | {:>10} | {:>10} | {:>7} | {:>11.8f} | {:>11.8f} |",
//                  q.index, q.dimensions, q.query_type, q.v, q.l, q.r);
// }
// spdlog::info("+-------------------------------------------------------------------------+");

// == Debug `fvamana.findMedoids()` ==
// for (const auto &m : fvamana.medoid_indices)
// {
//     spdlog::info("    {} -> {}", m.first, m.second);
// }

// Print the medoids per filter {} -> {}
// for (const auto &m : fvamana.medoid_indices)
// {
//     spdlog::info("{} -> {}", m.first, m.second);
// }

// class S_Vamana
// {
// public:
//     /**
//      * @brief
//      * Stiched Vamana Indexing Algorithm.
//      * @param a
//      * @param L_small
//      * @param R_small
//      * @param R_stiched
//      */
//     void stichedVamanaIndexing(std::vector<F_Point> &P, float a, int L_small, int R_small, int R_stiched)
//     {
//         // Initialize G = (V, E) to an empty graph
//         // Let F_x ⊆ F be the label-set for every x ∈ P
//         // Let P_F ⊆ P be the set of points with label f ∈ F
//         // foreach f ∈ F do
//         for (auto f : F)
//         {
//             // Let G_f = Vamana(P_f, a, R_small, L_small)
//             filteredVamanaIndexing(a, L_small, R_small);
//         }
//         // foreach u ∈ V do
//         for (auto)
//         //     FilteredRobustPrune(u, Nout(u), a, R_stiched)
//     }
// }