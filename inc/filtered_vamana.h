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
 * @return A true or false value is returned :
 * 1 if the points are compatible or  0 if they are not compatible
 */
int check_filters(Point &p ,Point &q){
    if (q.query_type == 0){
        return 1;
    }
    else if (q.query_type == 1 && p.category == q.category){
        return 1;
    }
    else if (q.query_type == 2 && q.lower_timestamp <= p.upper_timestamp && p.upper_timestamp <= q.upper_timestamp ){
        return 1;
    }
    else if (q.query_type == 3 && p.category == q.category && q.lower_timestamp <= p.upper_timestamp && p.upper_timestamp <= q.upper_timestamp ){
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

    // Overriding or extending methods


    
    
    /**
     * @brief
     * Greedy search algorithm.
     * @param S 
     * The set of the starting points.
     * @param query_idx
     * Index of the query node.
     * @param k
     * Number of nearest neighbors to find.
     * @param L_size
     * Maximum size of the candidate list L.
     * @param filter_q
     * Filter for the query.
     * @return std::pair<std::vector<Edge>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
    */
    std::pair<std::vector<Edge> , std::vector<int>> filteredGreedySearch(std::vector<int> S, int query_idx, int k, int L_size , int filter_q)
    {
        std::vector<Edge> L;            // Candidate points
        std::vector<int> V;             // Visited points
        std::vector<Edge> L_minus_V;    // L \ V

        Point* P = nullptr;
        Point* Q = nullptr;

        //  random source_idx for each filter 
        //  S is a set of start points for each filter        
        //  iterates over the start points and adds the index and distance to the search list if they match
        for(int s : S){
            P = &dataset[s];
            Q = &queryset[query_idx];
            if (check_filters(*P , *Q)){
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
                if(check_filters(*P , *Q)){
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
                if( (std::find(V.begin(), V.end(),  N.to_index) == V.end()) && (check_filters( dataset[N.to_index] , *Q) == 1) ){
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
     * Calculate the medoid of the dataset.
     * The medoid of the dataset is similar in concept to mean or centroid, but the medoid is always restricted to be a member of the data set.
     * This function calculates the total distance from each point to all other points and then select the point with the smallest total distance as the medoid.
     * @param data
     * The dataset that you want to calculate the medoid for.
     * @param tau
     * The threshold to select tau random numbers.
     * @return int
     * The index to the Medoid.
     */
    std::map <int ,  int> FindMedoid(std::vector<Point> data , int tau)
    {
        
        
        //  a map with the load of each point
        std::map<int, int> T;

        //  a map with all the available filters {category  , medoid_idx}
        std::map <int ,  int> fmap ;

        //  a map {category  , vector with the idx of compatible points }
        std::map <int , std::vector<int>> Pf;

        //  for each point if the dataset
        for(Point &f : data){
            
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

    
};

#endif // FILTERED_VAMANA_H
