#ifndef FILTERED_VAMANA_H
#define FILTERED_VAMANA_H

#include "vamana.h"


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

        // return [closest k points from L; V]
        if (L.size() > k)
            L.resize(k);

        return std::make_pair(L, V);

    }

};

#endif // FILTERED_VAMANA_H
