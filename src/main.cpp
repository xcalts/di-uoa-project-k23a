/**********************/
/* Standard Libraries */
/**********************/
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <string>

/**********************/
/* External Libraries */
/**********************/
#include "argh.h"
#include "stopwatch.h"
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "brute.h"
#include "constants.h"
#include "configuration.h"
#include "data.h"
#include "validation.h"
#include "parse.h"
#include "vamana.h"
#include "vamana-filtered.h"
#include "vamana-stiched.h"
#include "sets.h"
#include "math.h"
#include "utils.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

int main(int argc, char *argv[])
{
    try
    {
        /*************/
        /* Variables */
        /*************/
        std::string command;
        std::string yaml_conf_filepath;
        std::string algorithm;
        Configuration conf;

        /********************************/
        /* Print the welcoming message. */
        /********************************/
        std::cout << termcolor::magenta << WELCOMING_MSG << termcolor::reset << std::endl;

        /******************************************/
        /* Check for invalid number of arguments. */
        /******************************************/
        if (argc < 4)
            throw std::runtime_error("Usage: ./bin/app [COMMAND] --conf [CONF] --algo [ALGORITHM]");

        /*************************************/
        /* Parse the command line arguments. */
        /*************************************/
        std::cout << termcolor::green << INFO << "Parsing the command-line arguments" << termcolor::reset << std::endl;
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        cmdl({"-a", "--algo"}) >> algorithm;
        cmdl({"-c", "--conf"}) >> yaml_conf_filepath;
        command = argv[1];
        std::cout << termcolor::yellow << DEBUGG << "cli_command:        \"" << command << "\"" << termcolor::reset << std::endl;
        std::cout << termcolor::yellow << DEBUGG << "algorithm:          \"" << algorithm << "\"" << termcolor::reset << std::endl;
        std::cout << termcolor::yellow << DEBUGG << "yaml_conf_filepath: \"" << yaml_conf_filepath << "\"" << termcolor::reset << std::endl;

        /****************************************/
        /* Validate the command line arguments. */
        /****************************************/
        std::cout << termcolor::green << INFO << "Validating the command-line arguments" << termcolor::reset << std::endl;
        validateFileExists(yaml_conf_filepath);
        validateCommand(command);

        /**************************************/
        /* Parse the YAML configuration file. */
        /**************************************/
        std::cout << termcolor::green << INFO << "Parsing the YAML configuration file" << termcolor::reset << std::endl;
        conf.initialize(yaml_conf_filepath);

        /**************************/
        /* Executing the command. */
        /**************************/
        if (algorithm == VAMANA && command == CLI_INIT)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_INIT + "\" command for the \"" + VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.dataset_filepath);
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");
            std::cout << termcolor::yellow << DEBUGG << "k: " << conf.k << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "a: " << conf.a << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "L: " << conf.L << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "R: " << conf.R << termcolor::reset << std::endl;

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parsePointsFvecsFile(conf.dataset_filepath);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;

            /************************************/
            /* Initialize the Vamana Algorithm. */
            /************************************/
            std::cout << termcolor::green << INFO << "Initializing the Vamana Indexing Algorithm." << termcolor::reset << std::endl;
            Vamana v = Vamana();
            VamanaStatistics stats = v.index(points, points, conf.a, conf.L, conf.R);

            /**************************************/
            /* Save the produced graph to a file. */
            /**************************************/
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/vamana.bin'" << termcolor::reset << std::endl;
            v.saveGraph(points, "store/vamana.bin");

            /**************************/
            /* Output the statistics. */
            /**************************/
            std::cout << termcolor::green << INFO << "Outputing the gathered statistics" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(findMedoid): " << stats.medoid_calculation_time << " sec" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(vamanaIndex): " << stats.vamana_indexing_time << " sec" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"findMedoidTime", stats.medoid_calculation_time},
                {"vamanaIndexTime", stats.vamana_indexing_time}};
            appendResultsToFile("vamana-initialization-stats.json", command, conf, timings);
        }
        else if (algorithm == VAMANA && command == CLI_EVAL)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_EVAL + "\" command for the \"" + VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.evaluation_filepath);
            validateFileExists("store/vamana.bin");
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");

            std::cout << termcolor::yellow << DEBUGG << "k: " << conf.k << termcolor::reset << std::endl;

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parsePointsFvecsFile(conf.dataset_filepath);
            std::vector<Query> queries = parseQueriesFvecsFile(conf.queries_filepath);
            std::vector<Groundtruth> grountruths = parseIvecsFile(conf.evaluation_filepath);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(queries):     " << queries.size() << " (\"" + conf.queries_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(grountruths): " << grountruths.size() << " (\"" + conf.evaluation_filepath + "\")" << termcolor::reset << std::endl;

            /**********************************************/
            /* Load the Vamana Indexed graph from a file. */
            /**********************************************/
            std::cout << termcolor::green << INFO << "Loading the Vamana Indexing Algorithm from 'store/vamana.bin'" << termcolor::reset << std::endl;
            Vamana v = Vamana();
            v.loadGraph(points, "store/vamana.bin");

            /****************************************/
            /* Evaluate the Vamana Index algorithm. */
            /****************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Vamana Indexing Algorithm" << termcolor::reset << std::endl;
            int found = 0;
            int total = 0;
            std::set<int> nn, gt;
            double total_query_time = 0.0;
            for (int i = 0; i < queries.size(); i++)
            {
                auto start = std::chrono::high_resolution_clock::now();
                nn = v.greedySearch(points, v.S, queries[i], conf.k, conf.L).first;
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> query_time = end - start;
                total_query_time += query_time.count();

                gt = grountruths[i].getNNSet();

                found += getIntersectionSize(nn, gt);
                total += nn.size();
            }
            float recall = (static_cast<float>(found) / static_cast<float>(total)) * 100.0f;
            double average_query_time = total_query_time / queries.size();
            std::cout << termcolor::yellow << DEBUGG << "Recall: " << recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Query Time: " << average_query_time << " seconds" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"recall", recall},
                {"averageQueryTime", average_query_time}};
            appendResultsToFile("vamana-evaluation-stats.json", command, conf, timings);
        }
        else if (algorithm == BRUTE && command == CLI_EVAL)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_EVAL + "\" command for the \"" + BRUTE + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.evaluation_filepath);
            validateFileExists("store/vamana.bin");
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");

            std::cout << termcolor::yellow << DEBUGG << "k: " << conf.k << termcolor::reset << std::endl;

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parsePointsFvecsFile(conf.dataset_filepath);
            std::vector<Query> queries = parseQueriesFvecsFile(conf.queries_filepath);
            std::vector<Groundtruth> grountruths = parseIvecsFile(conf.evaluation_filepath);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(queries):     " << queries.size() << " (\"" + conf.queries_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(grountruths): " << grountruths.size() << " (\"" + conf.evaluation_filepath + "\")" << termcolor::reset << std::endl;

            /****************************************/
            /* Evaluate the Vamana Index algorithm. */
            /****************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Vamana Indexing Algorithm" << termcolor::reset << std::endl;
            Brute b = Brute(points);
            int found = 0;
            int total = 0;
            std::set<int> nns;
            std::set<int> gt;
            double total_query_time = 0.0;
            for (int i = 0; i < queries.size(); i++)
            {
                auto start = std::chrono::high_resolution_clock::now();
                nns = b.bruteForceNNs(queries[i], conf.k);
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> query_time = end - start;
                total_query_time += query_time.count();

                gt = grountruths[i].getNNSet();

                found += getIntersectionSize(nns, gt);
                total += nns.size();
            }
            float recall = (static_cast<float>(found) / static_cast<float>(total)) * 100.0f;
            double average_query_time = total_query_time / queries.size();
            std::cout << termcolor::yellow << DEBUGG << "Recall: " << recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Query Time: " << average_query_time << " seconds" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"recall", recall},
                {"averageQueryTime", average_query_time}};
            appendResultsToFile("vamana-evaluation-stats.json", command, conf, timings);
        }
        else if (algorithm == FILTERED_VAMANA && command == CLI_INIT)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_INIT + "\" command for the \"" + FILTERED_VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.dummy_data_filepath);
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");
            std::cout << termcolor::yellow << DEBUGG << "k: " << conf.k << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "a: " << conf.a << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "L: " << conf.L << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "R: " << conf.R << termcolor::reset << std::endl;

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;

            /************************************/
            /* Initialize the Vamana Algorithm. */
            /************************************/
            std::cout << termcolor::green << INFO << "Initializing the Filtered Vamana Indexing Algorithm." << termcolor::reset << std::endl;
            FilteredVamana fv = FilteredVamana(points);
            FilteredVamanaStatistics stats = fv.index(conf.tau, conf.a, conf.L, conf.R);

            /**************************************/
            /* Save the produced graph to a file. */
            /**************************************/
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/filtered-vamana.bin'" << termcolor::reset << std::endl;
            fv.saveGraph("store/filtered-vamana.bin");

            /***************************/
            /* Output the statistics . */
            /***************************/
            std::cout << termcolor::green << INFO << "Outputing the gathered statistics" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(findMedoid): " << stats.medoid_calculation_time << " sec" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(filteredVamanaIndex): " << stats.filtered_vamana_indexing_time << " sec" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"findMedoidTime", stats.medoid_calculation_time},
                {"filteredVamanaIndexTime", stats.filtered_vamana_indexing_time}};
            appendResultsToFile("filtered-initialization-stats.json", command, conf, timings);
        }
        else if (command == CLI_GTNN)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_GTNN + "\" command" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.dummy_data_filepath);
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
            std::vector<Query> queries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dummy_data_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(queries):     " << queries.size() << " (\"" + conf.dummy_queries_filepath + "\")" << termcolor::reset << std::endl;

            /***********************************/
            /* Initialize the Brute algorithm. */
            /***********************************/
            std::cout << termcolor::green << INFO << "Initializing the Brute Algorithm" << termcolor::reset << std::endl;
            Brute b = Brute(points);

            /****************************************************/
            /* Calculating the Groundtruth kNN for every query. */
            /****************************************************/
            std::cout << termcolor::green << INFO << "Generating the \"" << conf.k << "\"" << " nearest neighbors of the queries" << termcolor::reset << std::endl;
            b.calculateDummyGroundTruth(queries, conf.k);

            /*************************************/
            /* Save the produced kNNs to a file. */
            /*************************************/
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/groundtruth-nn.bin'" << termcolor::reset << std::endl;
            b.save("store/groundtruth-nn.bin");
        }
        else if (algorithm == FILTERED_VAMANA && command == CLI_EVAL)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_EVAL + "\" command for the \"" + FILTERED_VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.groundtruth_nn_filepath);
            validateFileExists("store/filtered-vamana.bin");
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
            std::vector<Query> queries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(queries):     " << queries.size() << " (\"" + conf.queries_filepath + "\")" << termcolor::reset << std::endl;

            /*******************************************************/
            /* Load the Filtered Vamana Indexed graph from a file. */
            /*******************************************************/
            std::cout << termcolor::green << INFO << "Loading the Filtered Vamana Indexing Algorithm from 'store/filtered-vamana.bin'" << termcolor::reset << std::endl;
            FilteredVamana fv = FilteredVamana(points);
            fv.loadGraph("store/filtered-vamana.bin");

            /*****************************************/
            /* Load the Brute Algorithm from a file. */
            /*****************************************/
            std::cout << termcolor::green << INFO << "Loading the Brute Force Algorithm from 'store/groundtruth-nn.bin'" << termcolor::reset << std::endl;
            Brute b = Brute(points);
            b.load("store/groundtruth-nn.bin");

            /*************************************************/
            /* Evaluate the Filtered Vamana Index algorithm. */
            /*************************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Filtered Vamana Indexing Algorithm" << termcolor::reset << std::endl;
            int f_found = 0, f_total = 0, u_found = 0, u_total = 0;
            double filtered_query_time = 0.0, unfiltered_query_time = 0.0, brute_query_time = 0.0;
            int filtered_query_count = 0, unfiltered_query_count = 0, brute_query_count = 0.0;
            std::set<int> nn, gt;

            fv.initializingEmptyGraph();
            fv.st = fv.findMedoids(conf.tau);

            std::set<int> start_nodes;
            for (auto f : fv.F)
                start_nodes.insert(fv.st[f]);

            BlockProgressBar bar{
                option::BarWidth{80},
                option::ForegroundColor{Color::red},
                option::PrefixText{"     Evaluating the Algorithm"},
                option::FontStyles{
                    std::vector<FontStyle>{FontStyle::bold}},
                option::MaxProgress{queries.size()}};
            int i = 0;

            for (const auto &q : queries)
            {
                // Show iteration as postfix text
                bar.set_option(option::PostfixText{
                    std::to_string(i + 1) + "/" + std::to_string(queries.size())});

                // update progress bar
                bar.tick();

                i++;

                auto start = std::chrono::high_resolution_clock::now();
                nn = b.bruteForceNNs(q, conf.k);
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> query_time = end - start;

                brute_query_time += query_time.count();
                brute_query_count++;

                if (q.query_type == 2 || q.query_type == 3)
                    continue;

                if (q.query_type == 1) // Filtered query
                {
                    std::set<float> F_q;
                    F_q.insert(q.v);

                    auto start = std::chrono::high_resolution_clock::now();
                    nn = fv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, F_q).first;
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> query_time = end - start;

                    filtered_query_time += query_time.count();
                    filtered_query_count++;

                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    f_found += getIntersectionSize(nn, gt);
                    f_total += nn.size();
                }
                else if (q.query_type == 0) // Unfiltered query
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    nn = fv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, {}).first;
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> query_time = end - start;

                    unfiltered_query_time += query_time.count();
                    unfiltered_query_count++;

                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    u_found += getIntersectionSize(nn, gt);
                    u_total += nn.size();
                }
            }

            bar.mark_as_completed();

            /***************************/
            /* Output the statistics . */
            /***************************/
            float u_recall = (static_cast<float>(u_found) / static_cast<float>(u_total)) * 100.0f;
            float f_recall = (static_cast<float>(f_found) / static_cast<float>(f_total)) * 100.0f;
            double avg_filtered_query_time = filtered_query_count > 0 ? filtered_query_time / filtered_query_count : 0.0;
            double avg_unfiltered_query_time = unfiltered_query_count > 0 ? unfiltered_query_time / unfiltered_query_count : 0.0;
            double avg_brute_query_time = brute_query_count > 0 ? brute_query_time / brute_query_count : 0.0;

            std::cout << termcolor::yellow << DEBUGG << "Unfiltered Recall: " << u_recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Filtered Recall: " << f_recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Unfiltered Query Time: " << avg_unfiltered_query_time << " seconds" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Filtered Query Time: " << avg_filtered_query_time << " seconds" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Brute Query Time: " << avg_brute_query_time << " seconds" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"unfilteredRecall", u_recall},
                {"filteredRecall", f_recall},
                {"unfilteredQueryTime", avg_unfiltered_query_time},
                {"filteredQueryTime", avg_filtered_query_time}};
            appendResultsToFile("filtered-eval.json", command, conf, timings);
        }
        else if (algorithm == STICHED_VAMANA && command == CLI_INIT)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_INIT + "\" command for the \"" + STICHED_VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.dummy_data_filepath);
            if (conf.L <= conf.k)
                throw std::runtime_error("L must be greater than k.");
            if (conf.L_small <= conf.k)
                throw std::runtime_error("Lsmall must be greater than k.");
            std::cout << termcolor::yellow << DEBUGG << "k: " << conf.k << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "a: " << conf.a << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "L: " << conf.L << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "R: " << conf.R << termcolor::reset << std::endl;

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;

            /************************************/
            /* Initialize the Vamana Algorithm. */
            /************************************/
            std::cout << termcolor::green << INFO << "Initializing the Stiched Vamana Indexing Algorithm." << termcolor::reset << std::endl;
            StichedVamana sv = StichedVamana(points);
            StichedVamanaStatistics stats = sv.index(conf.a, conf.L_small, conf.R_small, conf.R_stiched);

            /**************************************/
            /* Save the produced graph to a file. */
            /**************************************/
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/stiched-vamana.bin'" << termcolor::reset << std::endl;
            sv.saveGraph("store/stiched-vamana.bin");

            /***************************/
            /* Output the statistics . */
            /***************************/
            std::cout << termcolor::green << INFO << "Outputing the gathered statistics" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(stichedVamanaIndex): " << stats.stiched_vamana_indexing_time << " sec" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/
            std::map<std::string, double> timings = {
                {"stichedVamanaIndexTime", stats.stiched_vamana_indexing_time}};
            appendResultsToFile("stiched-init.json", command, conf, timings);
        }
        else if (algorithm == STICHED_VAMANA && command == CLI_EVAL)
        {
            std::cout << termcolor::blue << COMMAND << "Executing the \"" + CLI_EVAL + "\" command for the \"" + STICHED_VAMANA + "\" algorithm" << termcolor::reset << std::endl;

            /**********************************************************/
            /* Validate the required YAML configuration's parameters. */
            /**********************************************************/
            std::cout << termcolor::green << INFO << "Validating the required YAML configuration's parameters" << termcolor::reset << std::endl;
            validateFileExists(conf.groundtruth_nn_filepath);
            validateFileExists("store/stiched-vamana.bin");
            if (conf.L <= conf.k && conf.L_small <= conf.k)
                throw std::runtime_error("L and L_small must be greater than k.");

            /********************************/
            /* Parse the provided datasets. */
            /********************************/
            std::cout << termcolor::green << INFO << "Parsing the provided datasets." << termcolor::reset << std::endl;
            std::vector<Point> points = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
            std::vector<Query> queries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);
            std::cout << termcolor::yellow << DEBUGG << "sizeof(points):      " << points.size() << " (\"" + conf.dataset_filepath + "\")" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "sizeof(queries):     " << queries.size() << " (\"" + conf.queries_filepath + "\")" << termcolor::reset << std::endl;

            /******************************************************/
            /* Load the Stitched Vamana Indexed graph from a file. */
            /******************************************************/
            std::cout << termcolor::green << INFO << "Loading the Stitched Vamana Indexing Algorithm from 'store/stiched-vamana.bin'" << termcolor::reset << std::endl;
            StichedVamana sv = StichedVamana(points);
            sv.loadGraph("store/stiched-vamana.bin");

            /*****************************************/
            /* Load the Brute Algorithm from a file. */
            /*****************************************/
            std::cout << termcolor::green << INFO << "Loading the Brute Force Algorithm from 'store/groundtruth-nn.bin'" << termcolor::reset << std::endl;
            Brute b = Brute(points);
            b.load("store/groundtruth-nn.bin");

            /************************************************/
            /* Evaluate the Stitched Vamana Index algorithm. */
            /************************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Stitched Vamana Indexing Algorithm" << termcolor::reset << std::endl;

            int f_found = 0, f_total = 0, u_found = 0, u_total = 0;
            double filtered_query_time = 0.0, unfiltered_query_time = 0.0;
            int filtered_query_count = 0, unfiltered_query_count = 0;
            std::set<int> nn, gt;

            sv.initializingEmptyGraph();
            sv.st = sv.findMedoids(conf.tau);

            std::set<int> start_nodes;
            for (auto f : sv.F)
                start_nodes.insert(sv.st[f]);

            BlockProgressBar bar{
                option::BarWidth{80},
                option::ForegroundColor{Color::red},
                option::PrefixText{"     Evaluating the Algorithm"},
                option::FontStyles{
                    std::vector<FontStyle>{FontStyle::bold}},
                option::MaxProgress{queries.size()}};
            int i = 0;

            for (const auto &q : queries)
            {
                // Show iteration as postfix text
                bar.set_option(option::PostfixText{
                    std::to_string(i + 1) + "/" + std::to_string(queries.size())});

                // update progress bar
                bar.tick();

                i++;

                if (q.query_type == 2 || q.query_type == 3)
                    continue;

                if (q.query_type == 1) // Filtered query
                {
                    std::set<float> F_q;
                    F_q.insert(q.v);

                    auto start = std::chrono::high_resolution_clock::now();
                    nn = sv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, F_q).first;
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> query_time = end - start;

                    filtered_query_time += query_time.count();
                    filtered_query_count++;

                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    f_found += getIntersectionSize(nn, gt);
                    f_total += nn.size();
                }
                else if (q.query_type == 0) // Unfiltered query
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    nn = sv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, {}).first;
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> query_time = end - start;

                    unfiltered_query_time += query_time.count();
                    unfiltered_query_count++;

                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    u_found += getIntersectionSize(nn, gt);
                    u_total += nn.size();
                }
            }

            bar.mark_as_completed();

            float u_recall = (static_cast<float>(u_found) / static_cast<float>(u_total)) * 100.0f;
            float f_recall = (static_cast<float>(f_found) / static_cast<float>(f_total)) * 100.0f;
            double avg_filtered_query_time = filtered_query_count > 0 ? filtered_query_time / filtered_query_count : 0.0;
            double avg_unfiltered_query_time = unfiltered_query_count > 0 ? unfiltered_query_time / unfiltered_query_count : 0.0;

            std::cout << termcolor::yellow << DEBUGG << "Unfiltered Recall: " << u_recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Filtered Recall: " << f_recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Unfiltered Query Time: " << avg_unfiltered_query_time << " seconds" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Average Filtered Query Time: " << avg_filtered_query_time << " seconds" << termcolor::reset << std::endl;

            /************************************/
            /* Save the results in JSON format. */
            /************************************/

            std::map<std::string, double> timings = {
                {"unfilteredRecall", u_recall},
                {"filteredRecall", f_recall},
                {"unfilteredQueryTime", avg_unfiltered_query_time},
                {"filteredQueryTime", avg_unfiltered_query_time}};
            appendResultsToFile("stiched-eval.json", command, conf, timings);
        }
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << termcolor::red << EXCEPTION << e.what() << termcolor::reset << std::endl;
    }
}