#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <fstream>
#include <vector>

/**
 * @brief
 * This class represents an "Image" in the form of a `SIFT` descriptor.
 * Basically, the image is represented as a vector of X dimensions.
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
    Image(int dimensions, std::vector<float> data)
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
 * This class controls an in-memory database of `Image` objects.
 */
class ImageDatabase
{
private:
    std::string _filepath;
    std::vector<Image> _images;

    void initialize(const std::string &filepath)
    {
        _filepath = filepath;

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
        initialize(fvecspath);
    }

    /**
     * @brief Get the vector containing the `Image` objects.
     *
     * @return std::vector<Image>
     */
    std::vector<Image> getImages()
    {
        return _images;
    }
};

/**
 * @brief
 * This class contains the nearest images-neighbours of the image specified by `_image_index`.
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
    NearestNeighbours(int image_index, int dimensions, std::vector<int> neighbours_indexes)
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
 * This class controls an in-memory database of all the nearest images-neighbours for each and every image in the `ImageDatabase`.
 */
class NearestNeighboursDatabase
{
private:
    std::string _filepath;
    int _dimensions;
    std::vector<NearestNeighbours> _nns;

    void initialize(const std::string filepath)
    {
        _filepath = filepath;

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
     * @param fvecspath
     * The path to the file that contains the `ivecs` evaluation vector.
     */
    NearestNeighboursDatabase(const std::string &filepath)
    {
        initialize(filepath);
    }

    /**
     * @brief Get the vector containing the `NearestNeighbours` objects.
     *
     * @return std::vector<NearestNeighbours>
     */
    std::vector<NearestNeighbours> getNNs()
    {
        return _nns;
    }
};

#endif // DATA_H