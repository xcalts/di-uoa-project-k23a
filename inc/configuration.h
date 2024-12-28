#ifndef CONF_H
#define CONF_H

/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/**
 * @brief This class controls all the configuration logic.
 */
class Configuration
{
public:
    /**
     * @brief
     * The filepath to the FVECS dataset of image vectors.
     */
    std::string dataset_filepath;

    /**
     * @brief
     * The filepath to the FVECS queries of image vectors.
     */
    std::string queries_filepath;

    /**
     * @brief
     * The filepath to the IVECS evaluation metrics.
     */
    std::string evaluation_filepath;

    /**
     * @brief
     * The number of nearest neighbors to find.
     */
    int k;

    /**
     * @brief
     * Scaling factor to prune outgoing eges of a node (alpha).
     */
    float a;

    /**
     * @brief
     * Maximum list of search candidates to use in graph traversal.
     */
    int L;

    /**
     * @brief
     * Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    int R;

    /**
     * @brief
     * Smaller than L.
     */
    int L_small;

    /**
     * @brief
     * Smaller than R.
     */
    int R_small;

    /**
     * @brief
     * The number of outgoing edges of a node in the stiched graph.
     */
    int R_stiched;

    /**
     * @brief
     * The filepath to the Sigmod-Contest-2024 dataset of vectors.
     */
    std::string dummy_data_filepath;

    /**
     * @brief
     * The filepath to the Sigmod-Contest-2024 dataset of query vectors.
     */
    std::string dummy_queries_filepath;

    /**
     * @brief
     * The filepath to the ground-truth 10 nearest neighbors of the dummy queries.
     */
    std::string groundtruth_nn_filepath;

    /**
     * @brief
     * The dimension of the dummy data of vectors.
     */
    int data_dimensions;

    /**
     * @brief
     * The dimension of the dummy queries of vectors.
     */
    int queries_dimensions;

    /**
     * @brief
     * The number of nearest neighbors to find.
     */
    int tau;

    /**
     * @brief
     * Construct a new `Configuration` object.
     */
    Configuration();

    /**
     * @brief
     * This function initializes the `Configuration` object.
     * It parses the contents of the file pointed by `filepath`.
     * Then, it initializes the YAML tree using the `rapidyaml.h` library.
     * Finally, it will deserialize the value to the respected variables.
     *
     * @param filepath
     * The path to the YAML configuration file.
     */
    void initialize(const std::string &filepath);
};

#endif // CONF_H