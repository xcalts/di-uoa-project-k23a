#ifndef FILTERED_VAMANA_H
#define FILTERED_VAMANA_H

#include <map>
#include "vamana.h"
#include <vector>
#include <cstdlib> // For rand()
#include <ctime>   // For seeding rand()
#include <limits> // For INT_MAX


/**
 * @brief
 * Randomly sample tau points from a given vector 
 * @param Pf
 * The vector with integers
 * @param tau
 * The number of random points from the vector that will be returned
 * @return 
 * A vector with random tau points from Pf without duplicates
 */
std::vector<int> random_sample(std::vector<int>& Pf, int tau) {
    //  vector of all the indexes
    std::vector<int> all_points = Pf;
    //  vector with the tau random points out of all_points
    std::vector<int> sampled_points;
    int n = all_points.size();
    for (int i = 0; i < tau && n > 0; ++i) {
        int rand_index = rand() % n; // random index
        sampled_points.push_back(all_points[rand_index]);
        all_points.erase(all_points.begin() + rand_index); // remove chosen point to avoid duplicates
        --n; // decrease size of available points
    }

    return sampled_points;
}

/**
 * @brief
 * Select the point with the smallest load from the vector rf according to the map T
 * @param T
 * A map with the index of points and their load
 * @param rf
 * A vector with the indexes of points
 * @return 
 * The index of a point in the rf with the minimum load
 */
int select_least_loaded(const std::map<int, int>& T, const std::vector<int>& rf) {
    int min_load = INT_MAX;
    int selected_point = -1;

    for (int point : rf) {
        if (T.at(point) < min_load) {
            min_load = T.at(point);
            selected_point = point;
        }
    }
    return selected_point;
}

/**
 * @brief
 * Checks compatibility between a query and a dataset point.
 * @param P
 * The struct of the dataset point
 * @param Q
 * The struct of the query point
 * @param q_type
 * The type of the query for comparison
 * @return A true or false value is returned :
 * 1 if the points are compatible or  0 if they are not compatible
 */
int check_filters(Point &p ,Point &q ,int q_type){
    if (q_type == 0){
        return 1;
    }
    else if (q_type == 1 && p.category == q.category){
        return 1;
    }
    else if (q_type == 2 && q.lower_timestamp <= p.upper_timestamp && p.upper_timestamp <= q.upper_timestamp ){
        return 1;
    }
    else if (q_type == 3 && p.category == q.category && q.lower_timestamp <= p.upper_timestamp && p.upper_timestamp <= q.upper_timestamp ){
        return 1;
    }
    return 0;
}

/**
 * @brief
 * It expands the Vamana indexing algorithm with a set of utilities.
 */
class FilteredVamana : public Vamana {
public:
    // Constructor inheriting the Vamana constructor
    FilteredVamana(std::vector<Point>& _dataset) : Vamana(_dataset) {}
    FilteredVamana(std::vector<Point>& _dataset, std::vector<Point>& _queryset) : Vamana(_dataset, _queryset) {}
    
    
    /**
     * @brief
     * Greedy search algorithm.
     * @param S 
     * The set of the starting points.
     * @param query_vector
     * Point of the query.
     * @param k
     * Number of nearest neighbors to find.
     * @param L_size
     * Maximum size of the candidate list L.
     * @param filter_q
     * Filter for the query.
     * @return std::pair<std::vector<Edge>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
    */
    std::pair<std::vector<Edge> , std::vector<int>> filteredGreedySearch(std::vector<int> S, Point* query_vector, int k, int L_size , int filter_q)
    {
        std::vector<Edge> L;            // Candidate points
        std::vector<int> V;             // Visited points
        std::vector<Edge> L_minus_V;    // L \ V

        Point* P = nullptr;
        Point* Q = query_vector;

        //  random source_idx for each filter 
        //  S is a set of start points for each filter        
        //  iterates over the start points and adds the index and distance to the search list if they match
        for(int s : S){
            P = &dataset[s];
            if (check_filters(*P , *Q , filter_q)){
                L.emplace_back(s , euclideanDistance(P->vec , Q->vec));
            }
        }


        //  while L there are still elements in L that are not in V {the set L / V is not empty}
        while (1)
        {
            L_minus_V.clear();

            for (Edge &E_not_in_V : L)
                if (std::find(V.begin(), V.end(), E_not_in_V.to_index) == V.end())
                    L_minus_V.push_back(E_not_in_V);


            if(L_minus_V.empty())
                break;

            int min = L_minus_V[0].to_index;
            float closest_distance = L_minus_V[0].weight;
            //  P* <-  closest neighbor in L from the query xq that is not in the visited and satisfies filters
            for(Edge &N : L_minus_V){
                P = &dataset[N.to_index];
                //  check if N is compatible
                if(check_filters(*P , *Q , filter_q)){
                    if ( N.weight < closest_distance ){
                        closest_distance = N.weight;
                        min = N.to_index;
                    }
                }
            }
            //  closest point from query that is not visited yet
            P = &dataset[min];

            //  add Ρ to visited vector
            V.push_back(P->index);


            
            //  iterates through P*'s neighbors and adds them to L 
            for(Edge &N : P->outgoing_edges){
                //  if edge is not in visited , is compatible and is not already in the L vector
                // (std::find_if(L.begin(), L.end(), [&N](const Edge& edge) {return edge.to_index == N.to_index}))
                if( (std::find(V.begin(), V.end(),  N.to_index) == V.end()) && (check_filters( dataset[N.to_index] , *Q , filter_q) == 1) ){
                    //  adding the neighbors of P to the queue and their distance from Q
                    L.emplace_back(N.to_index , euclideanDistance(dataset[N.to_index].vec , Q->vec));
                }
            }

            // *****
            //      Improvement :
            //      add an integer if the if is true integer becomes 1 and if not integer becomes 0
            //      when the while starts again it will check if the integer is 1 and it will chose the item in position 0 from the L_minus_V vector
            //      if the integer is 0 it will go through the loop of finding the minimum distance
            //      This way if the L vector was sorted in the previous loop we will avoid searching for the minimum distance
            // *****

            //  if the L list is larger than L_size remove furthest points
            if (L.size() > L_size) {
                // Sort by weight in ascending order
                std::sort(L.begin(), L.end(), [](const Edge& a, const Edge& b) {
                    return a.weight < b.weight; //  sort by weight
                });

                //  remove furthest points (those with the largest weights)
                //  erase the last elements
                int num_to_erase = L.size() - L_size;
                L.erase(L.end() - num_to_erase, L.end());
            }
        }

        // return closest k points from L
        if (L.size() > k)
            L.resize(k);

        return std::make_pair(L, V);

    }

     /**
     * @brief
     * Calculate the medoids of the dataset P with associated filters.
     * For each filter in the dataset it searches for the compatible points and chooses a medoid.
     * @param P
     * The dataset that you want to calculate the medoids for each filter.
     * @param tau
     * Randomly sample tau point IDs in the Pf of each filter.
     * By choosing a small, random subset of points from Pf it makes the algorithm more efficient.
     * @return map<int,int> 
     * A map with the filter category and the index of the medoid.
     */
    std::map <int ,  int> FindMedoid(std::vector<Point> P , int tau){
        //  a map with the load of each point
        std::map<int, int> T;

        //  a map with all the available filters {category  , medoid_idx}
        std::map <int ,  int> fmap ;

        //  a map {category  , vector with the idx of compatible points }
        std::map <int , std::vector<int>> Pf;

        //  for each point if the dataset
        for(Point &f : P){
            
            //  initialize T map to 0
            T[f.index] = 0 ;

            //  if the filter is in the map
            if(fmap.find(f.category) != fmap.end()){
                // create a v
                std::vector<int> pf = Pf[f.category];
                pf.push_back(f.index);
                Pf[f.category] = pf;
            }
            //  if the category of that point is not on the map 
            else{
                //  add the filter to the map with idx -1 
                fmap[f.category] = -1;
                //  create a vector for that filter 
                std::vector<int> pf;
                //  add point idx
                pf.push_back(f.index);
                //  add vector to the map of filters
                Pf[f.category] = pf;
            }
        }

        //  for each filter
        for(auto filter : Pf){
            std::vector<int> rf;
            //  randomly select tau point out of Pf
            rf = random_sample(filter.second , tau);
            int p_star;
            //  select medoid with minimum T out of Pf
            p_star = select_least_loaded(T, rf);
            //  add p_star as medoid to current filter
            fmap[filter.first] = p_star;
            //  increase T count
            T[p_star] = T[p_star] + 1;
        }
        return fmap;
    }
    
    

    /**
     * @brief
     * It prunes the list of candidate neighbors for a given `Point` based on a
     * pruning condition that involves an `alpha` scaling factor.
     * It ensures that the `Point` has at most R outgoing edges to its nearest neighbors.
     * @param p_idx
     * The node for which we are pruning the candidate neighbors.
     * @param V
     * The set of candidate nodes.
     * @param alpha
     * The distance threshold.
     * @param R
     * The maximum number of outgoing edges.
     */
    void FilteredRobustPrune(int p_idx, std::vector<int> &V, float a, int R)
    {
        // V ← (V ∪ Nout(p)) \ {p}
        std::vector<Edge> &p_edges = dataset[p_idx].outgoing_edges;
        for (Edge &e : p_edges)
            if ((std::find(V.begin(), V.end(), e.to_index)) == V.end())
                V.push_back(e.to_index);

        V.erase(std::remove(V.begin(), V.end(), p_idx), V.end());

        // Nout(p) ← {}
        p_edges.clear();

        // while V != {} do
        while (!V.empty())
        {
            // p∗ ← min(d(p, v), v ∈ V)
            int pstar_idx = -1;
            float pstar_dist = std::numeric_limits<float>::max();
            for (int v_idx : V)
            {
                float distance = euclideanDistance(dataset[v_idx].vec, dataset[p_idx].vec);
                if (distance < pstar_dist)
                {
                    pstar_dist = distance;
                    pstar_idx = v_idx;
                }
            }

            // Nout(p) ← Nout(p) ∪ {p∗} (check that p* not in Nout(p))
            if (std::find_if(
                    p_edges.begin(),
                    p_edges.end(),
                    [pstar_idx](const Edge &edge)
                    { return edge.to_index == pstar_idx; }) == p_edges.end())
                p_edges.push_back(Edge(pstar_idx, pstar_dist));

            // if |Nout(p)| = R then break
            if (p_edges.size() == R)
                break;

            // for v ∈ V do.
            std::vector<int> to_remove;
            for (int v_idx : V)
            {
                // if Fv ∪ Fp not a subset in Fp* then continue
                if(!(dataset[v_idx].category == dataset[p_idx].category && dataset[p_idx].category == dataset[pstar_idx].category)){
                    continue;
                }

                float pstar_to_v = euclideanDistance(dataset[pstar_idx].vec, dataset[v_idx].vec);
                float p_to_v = euclideanDistance(dataset[p_idx].vec, dataset[v_idx].vec);

                // if α · d(p∗, v) ≤ d(p, v) then remove v from V
                if (a * pstar_to_v <= p_to_v)
                    to_remove.push_back(v_idx);
            }

            // remove v from V
            for (int remove_idx : to_remove)
                V.erase(std::remove(V.begin(), V.end(), remove_idx), V.end());
        }
    }

    
    /**
     * @brief
     * Creates an in-memory index on the supplied nodes to efficiently answer approximate nearest neighbor queries.
     * The N nodes must already be initialized as a random graph of outgoing edges with maximum of log(N) outgoing edges per node.
     * @param a
     * Scaling factor to prune outgoing eges of a node (alpha).
     * @param L
     * Maximum list of search candidates to use in graph traversal.
     * @param R
     * Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    void FilteredVamanaIndex(float a, int L, int R)
    {
        spdlog::debug("+---------------------------+");
        spdlog::debug("| Vamana Indexing Algorithm |");
        spdlog::debug("+---------------------------+");
        spdlog::debug("# a ← {} & L_size ← {} & R ← {} & s ← {}", a, L, R, medoid_idx);

        // initialize G to an empty graph
        

        // let s denote the medoid of dataset P
        int s = medoid_idx;
        
        // Let st(f) denote the start node for filter label f for every f ∈ F
        std::map<int ,int> f_and_st = FindMedoid(dataset , 5);
        std::vector<int> st;
        for(auto st_p : f_and_st){
            st.push_back(st_p.first);
        }

        // let sigma denote a random permutation of 1..n
        std::vector<int> sigma = generateSigma(dataset_size);

        // Let Fx be the label-set for every x ∈ P 
        std::vector<int> Fx;
        for(auto f : f_and_st){
            Fx.push_back(f.first);
        }

        // for 1 ≤ i ≤ n do
        for (int i = 0; i < dataset_size; ++i)
        {
            // Let Sfx_σ(i) = { st(f) : f ∈ Fx_σ(i) }   //  sfx is the start point for filter f of database point in position sigma[i]
            std::vector<int> Sfx;
            for(auto st_f : f_and_st){
                if(st_f.first == dataset[sigma[i]].category){
                    Sfx.push_back(st_f.second);
                }
            }
            //Sfx.push_back(f_and_st.at(dataset[sigma[i]].category));

            // let [L; V] ← GreedySearch(sfx, Xσ(i),0 , L_size , Fx_σ(i))
            std::pair<std::vector<Edge>,  std::vector<int>> r = filteredGreedySearch(Sfx, &dataset[sigma[i]] , 1, L , 1);

            // Let V ← V ∪ V_Fx_σ(i)
            //  ********************

            // run RobustPrune(σ(i), V, a, R) to update out-neighbors of σ(i)
            FilteredRobustPrune(sigma[i], r.second, a, R);

            spdlog::debug("=====================================================================");
            spdlog::debug("+ GreedySearch(s, σ({}), 1, {})", i, L);
            spdlog::debug("# V ← {}", vectorTable(r.second));
            spdlog::debug("+ RobustPrune(σ({}), V, a, R)", i);
            spdlog::debug("# Neighbors(σ({}))) ← {}", i, neighborsTable(sigma[i]));

            // for all points j in Neighbors(σ(i))
            for (Edge &e : dataset[sigma[i]].outgoing_edges)
            {
                int j_idx = e.to_index;

                // W ← Nout(j) ∪ {σ(i)}
                std::vector<int> W(dataset[j_idx].outgoing_edges.size());

                std::transform(
                    dataset[j_idx].outgoing_edges.begin(),
                    dataset[j_idx].outgoing_edges.end(),
                    W.begin(),
                    [](const Edge &edge)
                    { return edge.to_index; });

                W.push_back(sigma[i]);

                spdlog::debug("---------------------------------------------------------------------");
                spdlog::debug("# j ← {}", j_idx);
                spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));

                // |W| > R => run RobustPrune(j, W, a, R) to update out-neighbors of j
                if (W.size() > R)
                {
                    FilteredRobustPrune(j_idx, W, a, R);

                    spdlog::debug("+ |W| > R => RobustPrune(j, W, {}, {})", j_idx, a, R);
                    spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));
                }
                // |W| < R | => update Neighbors(j) ← Neighbors(j) U {sigma(i)}
                else
                {
                    dataset[j_idx].outgoing_edges.push_back(Edge(sigma[i], euclideanDistance(dataset[sigma[i]].vec, dataset[j_idx].vec)));

                    spdlog::debug("+ |W| < R => Neighbors(j) ← Neighbors(j) U {}", R, j_idx, j_idx, sigma[i]);
                    spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));
                }

                spdlog::debug("---------------------------------------------------------------------");
            }
        }

        spdlog::debug("+---------------------------------------------------------------------------------+");
    }
    
};

#endif // FILTERED_VAMANA_H
