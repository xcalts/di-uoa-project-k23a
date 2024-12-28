/**********************/
/* Standard Libraries */
/**********************/
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <format>
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
            throw std::runtime_error("Usage: ./bin/main [COMMAND] --conf [CONF] --algo [ALGORITHM]");

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
            VamanaStatistics stats = v.index(points, conf.a, conf.L, conf.R);

            /**************************************/
            /* Save the produced graph to a file. */
            /**************************************/
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/vamana.bin'" << termcolor::reset << std::endl;
            v.saveGraph("store/vamana.bin");

            /***************************/
            /* Output the statistics . */
            /***************************/
            std::cout << termcolor::green << INFO << "Outputing the gathered statistics" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(findMedoid): " << stats.medoid_calculation_time << " sec" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(vamanaIndex): " << stats.vamana_indexing_time << " sec" << termcolor::reset << std::endl;
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
            v.dataset = points;
            v.loadGraph("store/vamana.bin");

            /****************************************/
            /* Evaluate the Vamana Index algorithm. */
            /****************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Vamana Indexing Algorithm" << termcolor::reset << std::endl;
            int found = 0;
            int total = 0;
            int s = 8736;
            std::set<int> nn, gt;
            for (int i = 0; i < queries.size(); i++)
            {
                nn = v.greedySearch(v.dataset[s], queries[i], conf.k, conf.L).first;
                gt = grountruths[i].getNNSet();

                found += getIntersectionSize(nn, gt);
                total += nn.size();
            }
            float recall = (static_cast<float>(found) / static_cast<float>(total)) * 100.0f;
            std::cout << termcolor::yellow << DEBUGG << "Recall: " << recall << "%" << termcolor::reset << std::endl;
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
            std::cout << termcolor::green << INFO << "Loading the Filtered Vamana Indexing Algorithm from 'store/vamana.bin'" << termcolor::reset << std::endl;
            FilteredVamana fv = FilteredVamana(points);
            fv.loadGraph("store/filtered-vamana.bin");

            /*****************************************/
            /* Load the Brute Algorithm from a file. */
            /*****************************************/
            std::cout << termcolor::green << INFO << "Loading the Brute Force Algorithm from 'store/groundtruth-nn.bin'" << termcolor::reset << std::endl;
            Brute b = Brute(points);
            b.load("store/groundtruth-nn.bin");

            /****************************************/
            /* Evaluate the Vamana Index algorithm. */
            /****************************************/
            std::cout << termcolor::green << INFO << "Evaluating the Filtered Vamana Indexing Algorithm" << termcolor::reset << std::endl;
            int f_found = 0;
            int f_total = 0;
            int u_found = 0;
            int u_total = 0;
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

                if (q.query_type == 2 || q.query_type == 3)
                    continue;

                if (q.query_type == 1)
                {
                    std::set<float> F_q;
                    F_q.insert(q.v);
                    nn = fv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, F_q).first;
                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    f_found += getIntersectionSize(nn, gt);
                    f_total += nn.size();
                }
                else if (q.query_type == 0)
                {
                    nn = fv.filteredGreedySearch(start_nodes, q, conf.k, conf.L, {}).first;
                    gt = b.getTrueNearestNeighborsAsSet(q.index, conf.k);

                    u_found += getIntersectionSize(nn, gt);
                    u_total += nn.size();
                }
            }

            bar.mark_as_completed();

            float u_recall = (static_cast<float>(u_found) / static_cast<float>(u_total)) * 100.0f;
            float f_recall = (static_cast<float>(f_found) / static_cast<float>(f_total)) * 100.0f;
            std::cout << termcolor::yellow << DEBUGG << "Unfiltered Recall: " << u_recall << "%" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Filtered Recall: " << f_recall << "%" << termcolor::reset << std::endl;
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
            std::cout << termcolor::green << INFO << "Saving the produced graph in 'store/filtered-vamana.bin'" << termcolor::reset << std::endl;
            sv.saveGraph("store/stiched-vamana.bin");

            /***************************/
            /* Output the statistics . */
            /***************************/
            std::cout << termcolor::green << INFO << "Outputing the gathered statistics" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(findMedoid): " << stats.medoid_calculation_time << " sec" << termcolor::reset << std::endl;
            std::cout << termcolor::yellow << DEBUGG << "Time(stichedVamanaIndex): " << stats.stiched_vamana_indexing_time << " sec" << termcolor::reset << std::endl;
        }
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << termcolor::red << EXCEPTION << e.what() << termcolor::reset << std::endl;
    }
}