/**********************/
/* Standard Libraries */
/**********************/
#include <string>
#include <fstream>
#include <iostream>
#include <cstdint>

/**********************/
/* Project Components */
/**********************/
#include "parse.h"

std::vector<Point> parsePointsFvecsFile(std::string &fvecs_filepath)
{
    std::vector<Point> points;
    std::ifstream fvecs(fvecs_filepath, std::ios::binary);
    int idx = 0;

    while (fvecs)
    {
        int dimensions;

        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        fvecs.read(reinterpret_cast<char *>(&dimensions), sizeof(int));

        if (!fvecs)
            break;

        std::vector<float> v(dimensions);

        // 2. Read the rest of the values as a vector of size `vector_no_dimensions`.
        fvecs.read(reinterpret_cast<char *>(v.data()), dimensions * sizeof(float));

        // 3. Create a new `Point` object.
        Point new_point = Point(idx, v);

        // 4. Add it to the database.
        points.push_back(new_point);

        // 5. Incremnt index.
        idx++;
    }

    fvecs.close();

    return points;
}

std::vector<Query> parseQueriesFvecsFile(std::string &fvecs_filepath)
{
    std::vector<Query> queries;
    std::ifstream fvecs(fvecs_filepath, std::ios::binary);
    int idx = 0;

    while (fvecs)
    {
        int dimensions;

        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        fvecs.read(reinterpret_cast<char *>(&dimensions), sizeof(int));

        if (!fvecs)
            break;

        std::vector<float> v(dimensions);

        // 2. Read the rest of the values as a vector of size `vector_no_dimensions`.
        fvecs.read(reinterpret_cast<char *>(v.data()), dimensions * sizeof(float));

        // 3. Create a new object.
        Query new_query = Query(idx, v);

        // 4. Add it to the database.
        queries.push_back(new_query);

        // 5. Incremnt index.
        idx++;
    }

    fvecs.close();

    return queries;
}

std::vector<Groundtruth> parseIvecsFile(std::string &ivecs_filepath)
{
    std::vector<Groundtruth> groundtruths;

    std::ifstream ivecs(ivecs_filepath, std::ios::binary);

    int i = 0;

    while (ivecs)
    {
        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        int dimensions = 0;
        ivecs.read(reinterpret_cast<char *>(&dimensions), sizeof(int));

        if (!ivecs)
            break;

        std::vector<int> v(dimensions);

        // 2. Read the rest of the values as a vector of size `_dimensions`.
        ivecs.read(reinterpret_cast<char *>(v.data()), dimensions * sizeof(int));

        // 3. Create a new object.
        Groundtruth g = Groundtruth(i, v);

        // 4. Add it to the database.
        groundtruths.push_back(g);

        // 5. Incremnt index.
        i++;
    }

    ivecs.close();

    return groundtruths;
}

std::vector<Point> parseDummyData(std::string &filepath, int no_dimensions)
{
    std::ifstream ifs;
    ifs.open(filepath, std::ios::binary);

    // 1. The first four bytes(int) represent the number of points in the file.
    uint32_t N;
    ifs.read((char *)&N, sizeof(uint32_t));

    std::vector<Point> points;
    std::vector<float> buff(no_dimensions);

    // 2. Read and parse each point.
    for (int _idx = 0; _idx < N; ++_idx)
    {
        // 3. Read point and store it in buff.
        if (!ifs.read(reinterpret_cast<char *>(buff.data()), no_dimensions * sizeof(float)))
        {
            std::cerr << "Error reading point data at index " << _idx << std::endl;
            break;
        }

        // 4. Parse metadata.
        float _c = buff[0];
        float _t = buff[1];

        // 5. Parse vector data (excluding the first 4 metadata values)
        std::vector<float> _vec(buff.begin() + 2, buff.end());

        // 6. Create a new point placing the category in the category argument an timestamp in both bounds(lower , upper).
        Point new_point = Point(_idx, _vec, _c, _t);

        // 7. Add it to the database.
        points.push_back(new_point);
    }

    // 8. Close file and return the database
    ifs.close();

    return points;
}

std::vector<Query> parseDummyQueries(std::string &filepath, int no_dimensions)
{
    std::ifstream ifs;
    ifs.open(filepath, std::ios::binary);

    // 1. The first four bytes(int) represent the number of points in the file.
    uint32_t N;
    ifs.read((char *)&N, sizeof(uint32_t));

    std::vector<Query> queries;
    std::vector<float> buff(no_dimensions);

    // 2. Read and parse each point.
    for (int _idx = 0; _idx < N; ++_idx)
    {
        // 3. Read point and store it in buff.
        if (!ifs.read(reinterpret_cast<char *>(buff.data()), no_dimensions * sizeof(float)))
            break;

        // 4. Parse metadata.
        QueryType _query_type = static_cast<QueryType>(buff[0]);
        float _v = buff[1];
        float _l = buff[2];
        float _r = buff[3];

        // 5. Parse vector data (excluding the first 4 metadata values)
        std::vector<float> _vec(buff.begin() + 4, buff.end());

        // 6. Create a new F_Point object.
        Query new_query = Query(
            _idx,
            _vec,
            _query_type,
            _v,
            _l,
            _r);

        // 7. Add it to the database.
        queries.push_back(new_query);
    }

    // 8. Close file and return the database
    ifs.close();

    return queries;
}