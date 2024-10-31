#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <fstream>
#include <vector>

#include "log.h"

/**
 * @brief
 * It represents an `Image` in the form of a `vector` of X dimensions.
 * The data are correspond to the `SIFT` descriptor of the original image.
 */
class Image
{
private:
    int _dimensions;
    std::vector<float> _data;

public:
    /**
     * @brief
     * Base constructor.
     *
     */
    Image() {}

    /**
     * @brief
     * Construct a new `Image` object.
     *
     * @param dimensions
     * The number of dimensions of the `Image`.
     *
     * @param data
     * The initial data of the `Image`.
     */
    Image(int dimensions, const std::vector<float> &data)
    {
        _dimensions = dimensions;
        _data = data;
    }

    /**
     * @brief
     * Prints the data vector in a table format.
     */
    void printTable() const
    {
        std::cout << "[ ";
        for (size_t i = 0; i < _data.size(); ++i)
        {
            std::cout << _data[i];
            if (i < _data.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << " ]" << std::endl;
    }

    /**
     * @brief
     * Prints the data vector in a histogram format.
     */
    void printHistogram() const
    {
        const size_t height = 20; // Height of histogram in rows
        float max_value = *std::max_element(_data.begin(), _data.end());

        // Loop over each row from top to bottom
        for (size_t row = height; row > 0; --row)
        {
            float threshold = (static_cast<float>(row) / height) * max_value;

            for (size_t i = 0; i < _data.size(); ++i)
            {
                if (_data[i] >= threshold)
                {
                    std::cout << "#";
                }
                else
                {
                    std::cout << " ";
                }
            }
            std::cout << std::endl;
        }

        // Print the x-axis with index labels
        for (size_t i = 0; i < _data.size(); ++i)
        {
            std::cout << "-";
        }
        std::cout << std::endl;
    }

    /**
     * @brief
     * Generates a histogram of the data vector as a formatted string.
     *
     * @return std::string
     */
    std::string getHistogram() const
    {
        const size_t height = 20; // Height of histogram in rows
        float max_value = *std::max_element(_data.begin(), _data.end());
        std::ostringstream oss;

        // Loop over each row from top to bottom
        for (size_t row = height; row > 0; --row)
        {
            float threshold = (static_cast<float>(row) / height) * max_value;

            for (size_t i = 0; i < _data.size(); ++i)
            {
                if (_data[i] >= threshold)
                {
                    oss << ".";
                }
                else
                {
                    oss << " ";
                }
            }
            oss << "\n";
        }

        // Append the x-axis with index labels
        for (size_t i = 0; i < _data.size(); ++i)
        {
            oss << "-";
        }
        oss << "\n";

        // Append index labels below the histogram
        for (size_t i = 0; i < _data.size(); ++i)
        {
            oss << i % 10; // Single-digit labels for compactness
        }
        oss << "\n";

        return oss.str();
    }
};

/**
 * @brief
 * It represents the in-memory database of `Image` objects.
 * It can either contain the dataset or the queries.
 * It is initialized using a path to a `.fvecs` file.
 */
class ImageDatabase
{
private:
    std::string _filepath;
    std::vector<Image> _images;
    int _total_images;

    void initialize()
    {
        verbose("(data.h) Parsing the .fvecs file at: " + _filepath + ".");

        std::ifstream filestream(_filepath, std::ios::binary);

        while (filestream)
        {
            int vector_no_dimensions;

            // 1. The first four bytes(int) represent the number of dimensions of the vector data.
            filestream.read(reinterpret_cast<char *>(&vector_no_dimensions), sizeof(int));

            if (!filestream)
                break;

            std::vector<float> v(vector_no_dimensions);

            // 2. Read the rest of the values as a vector of size `vector_no_dimensions`.
            filestream.read(reinterpret_cast<char *>(v.data()), vector_no_dimensions * sizeof(float));

            // 3. Create a new `Image` object.
            Image new_image = Image(vector_no_dimensions, v);

            // 4. Add it to the database.
            _images.push_back(new_image);
        }

        filestream.close();

        _total_images = _images.size();
    }

public:
    /**
     * @brief
     * Base constructor.
     *
     */
    ImageDatabase() {}

    /**
     * @brief
     * Construct a new `ImageDatabase` object.
     * Note that there are not validation checks for the `filepath`.
     *
     * @param fvecspath
     * The path to the file that contains the `fvecs` vectors dataset.
     */
    ImageDatabase(const std::string &fvecspath)
    {
        _filepath = fvecspath;

        initialize();
    }

    /**
     * @brief
     * It returns how many images there are in the database.
     *
     * @return int
     */
    int getTotal()
    {
        return _total_images;
    }

    /**
     * @brief Get the vector containing the `Image` objects.
     *
     * @return std::vector<Image>&
     */
    std::vector<Image> &getImages()
    {
        return _images;
    }

    /**
     * @brief
     * Get the `Image` object by index
     * @param index
     * The index of the desired `Image`.
     * @return const Image&
     */
    const Image &getImage(int index) const
    {
        return _images[index];
    }
};

/**
 * @brief
 * It represents the ground-truth nearest neighbours of an indexed `Image` in the dataset `ImageDatabase`.
 */
class NearestNeighbours
{
private:
    int _image_index;
    int _dimensions;
    std::vector<int> _neighbours_indexes;

public:
    /**
     * @brief
     * Base constructor.
     */
    NearestNeighbours() {}

    /**
     * @brief
     * Construct a new `NearestNeighbours` object.
     *
     * @param image_index
     * The index of the image in the `ImageDatabase`, that this class is refering to.
     * @param dimensions
     * The dimensions of the `neighbours_indexes` vector.
     * @param neighbours_indexes
     * The vector containing the indices of the nearest neighbours.
     */
    NearestNeighbours(int image_index, int dimensions, const std::vector<int> &neighbours_indexes)
    {
        _image_index = image_index;
        _dimensions = dimensions;
        _neighbours_indexes = neighbours_indexes;
    }

    /**
     * @brief
     * Prints the nearest neighbours in a table format.
     */
    void printTable() const
    {
        std::cout << "[ ";
        for (size_t i = 0; i < _neighbours_indexes.size(); ++i)
        {
            std::cout << _neighbours_indexes[i];
            if (i < _neighbours_indexes.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << " ]" << std::endl;
    }
};

/**
 * @brief
 * It represents the in-memory database of all `NearestNeighbours` for each and every indexed image in the dataset `ImageDatabase`.
 * It is used for evaluating the KNN search algorithms.
 */
class NearestNeighboursDatabase
{
private:
    std::string _filepath;
    int _dimensions;
    std::vector<NearestNeighbours> _nns;

    void initialize()
    {
        verbose("(data.h) Parsing the .ivecs file at: " + _filepath + ".");

        std::ifstream filestream(_filepath, std::ios::binary);
        std::vector<std::vector<int>> data;
        int count = 0;

        while (filestream)
        {
            // 1. The first four bytes(int) represent the number of dimensions of the vector data.
            int dim = 0;
            filestream.read(reinterpret_cast<char *>(&dim), sizeof(int));

            if (!filestream)
                break;

            std::vector<int> v(dim);

            // 2. Read the rest of the values as a vector of size `_dimensions`.
            filestream.read(reinterpret_cast<char *>(v.data()), dim * sizeof(int));

            // 3. Create a new `NearestNeighbours` object.
            NearestNeighbours new_nn = NearestNeighbours(count, dim, v);

            // 4. Add it to the database.
            _nns.push_back(new_nn);

            // 5. Increase the idx.
            count++;
        }

        filestream.close();
    }

public:
    /**
     * @brief
     * Base constructor.
     */
    NearestNeighboursDatabase() {}

    /**
     * @brief
     * Construct a new `NearestNeighboursDatabase` object.
     * Note that there are not validation checks for the `filepath`.
     *
     * @param ivecspath
     * The path to the file that contains the `ivecs` evaluation vector.
     */
    NearestNeighboursDatabase(const std::string &ivecspath)
    {
        _filepath = ivecspath;
        initialize();
    }

    /**
     * @brief Get the vector containing the `NearestNeighbours` objects.
     *
     * @return std::vector<NearestNeighbours>&
     */
    std::vector<NearestNeighbours> &getNNs()
    {
        return _nns;
    }
};

/**
 * @brief
 * It represents a directed edge from one node to another with an associated weight.
 */
class Edge
{
private:
    int _to_index;
    float _weight;

public:
    /**
     * @brief
     * Base constructor.
     *
     */
    Edge() {}

    /**
     * @brief Construct a new `Edge` object.
     *
     * @param to_index
     * Index of the destination image in the dataset.
     * @param weight
     * Weight of the edge.
     */
    Edge(int to_index, float weight)
    {
        _to_index = to_index;
        _weight = weight;
    }
};

/**
 * @brief
 * It represents the database that controls the nodes and edges of the graph.
 */
class Graph
{
private:
    int _total_nodes;
    std::vector<std::vector<Edge>> _adjacency_list;

public:
    /**
     * @brief
     * Base constructor.
     */
    Graph() {}

    /**
     * @brief
     * Construct a new `Graph` object.
     * @param images
     * The vector containing the `Image` dataset.
     */
    Graph(const std::vector<Image> images)
    {
        _total_nodes = images.size();
        _adjacency_list.resize(_total_nodes);
    }

    /**
     * @brief
     * Add a directed edge with a weight.
     * @param from_index
     * The source node index.
     * @param to_index
     * The destination node index.
     * @param weight
     * The weight of the edge.
     */
    void addEdge(int from_index, int to_index, float weight)
    {
        if (from_index >= 0 && from_index < _adjacency_list.size())
            _adjacency_list[from_index].push_back(Edge(to_index, weight));
        else
            throw std::runtime_error("Invalid from_index: " + from_index);
    }

    /**
     * @brief
     * Get the `Edges` object of specified node.
     * @param index
     * The index of the desired node.
     * @return const std::vector<Edge>&
     */
    const std::vector<Edge> &getEdges(int index) const
    {
        return _adjacency_list[index];
    }
};

#endif // DATA_H