#include "lazyrebuild.h"

#include <cstring>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>

#include <time.h>

#include <addon.h>
#include "sKdTree.h"
#include "dKdTree.h"


using namespace std;

/**
 * @brief testfun1 is used to test build function of static and dynamic kd tree
 */
void testfun1();
/**
 * @brief testfun2 is used to test the correctness of nn search result of static and dynamci kd tree
 */
void testfun2();
/**
 * @brief testfun3 is used to compare performance of static and dynamic kd tree
 */
void testfun3();

void testfun4();

void testfun5();

void testfun6();

void testLazy();



int main()
{
    testfun1();
//    testfun2();
//    testfun3();
//    testfun4();
//    testfun5();
//    testfun6();
//    testLazy();
    return 0;
}


void testfun1() {
        float datas[200] = {1.3, 1.3, 1.3,
                            8.3, 8.3, 8.3,
                            2.3, 2.3, 2.3,
                            1.2, 1.2, 1.2,
                            7.3, 7.3, 7.3,
                            9.3, 9.3, 9.3,
                            15, 15, 15,
                            3, 3, 3,
                            1.1, 1.1, 1.1,
                            12, 12, 12,
                            4, 4, 4,
                            5, 5, 5,
                            1, 1, 1,
                            2, 2, 2,
                            6, 6, 6};



        for (int i = 1; i < 35; i++) {
            for (int j = 0; j < 3; j++) {
                datas[(i-1)*3+j] = i;
            }
        }
        cout << "Static KD Tree" << endl;
        SKDTree<float, 3>* static_kdtree = new SKDTree<float, 3>(datas, 15, false);
        static_kdtree->printTree();
        cout << endl;
        cout << "---------------------------------------------" << endl;

        cout << "Dynamic KD Tree based on logarithmic rebuild" << endl;
        DKDTree<float, 3>* dynamic_kdtree = new DKDTree<float, 3>(datas, 15);
        dynamic_kdtree->printTree();
        cout << "---------------------------------------------" << endl;

        cout << "Dynamic KD Tree based on lazy rebuild" << endl;
        DKDTree2<float, 3>* lazy_kdtree1 = new DKDTree2<float, 3>(datas, 15);
        lazy_kdtree1->printTree();
        cout << "---------------------------------------------" << endl;

//        DKDTree2<float, 3>* lazy_kdtree2 = new DKDTree2<float, 3>();

//        for (int i = 0; i < 33; i++) {
//            vector<float> insert_p;
//            for (int j = 0; j < 3; j++) insert_p.emplace_back(datas[i*3+j]);
//            lazy_kdtree2->insertNode(insert_p);

//        }
//        lazy_kdtree2->printTree();

//        for (int i = 0; i < 10; i++) {
//            vector<float> p2(3);
//            vector<float> s_re, lazy_re1, lazy_re2;
//            for (int j = 0; j < 3; j++) p2[j] = 0.1*i+j;
//            s_re = static_kdtree->NNSearch(p2);
//            lazy_re2 = lazy_kdtree2->NNSearch(p2);
//            cout << "Static nn result = " << getDist(s_re, p2) << endl;
//            cout << "Lazy nn result = " << getDist(lazy_re2, p2) << endl;
//        }


//        vector<float> p2(3);
//        p2[0] = 10;
//        p2[1] = 11;
//        p2[2] = 12;
//        vector<float> result;
//        result = lazy_kdtree->NNSearch(p2);
//        for (int i = 0; i < 3; i++) {
//            cout << result[i] << "\t";
//        }
//        cout << endl;
//        cout << "Lazy rebuild KD Tree: " << endl;
//        DKDTree<float, 3> dynamic_kdtree(datas, 15);
//        cout << "Dynamic KD Tree: " << endl;
//        dynamic_kdtree.printTree();
//        cout << endl;

//        vector<float> p2(3);
//        p2[0] = 11;
//        p2[1] = 11;
//        p2[2] = 11;
//        cout << "Dynamic KD Tree: " << endl;
//        dynamic_kdtree.insertNode(p2);
//        dynamic_kdtree.printTree();
//        cout << endl;

}

void testfun2() {
    ofstream outfile;
    outfile.open("../DynamicKDTree/experiment/3.28/testfun2.txt");
    srand((int)time(NULL));

    int N = 999999;
    int tree_node_data_size[3] = {1000, 10000, 100000};
    int query_node_data_size[5] = {1, 10, 100, 1000, 10000};

    const size_t dimension = 3;

    clock_t static_ts1, static_te1, static_ts2, static_te2, dynamic_ts, dynamic_te;
    double static_query_time1, static_query_time2, dynamic_query_time;

    for (int t = 0; t < 3; t++) {
        for (int q = 0; q < 5; q++) {
            cout << "DATA_SIZE = " << tree_node_data_size[t] << "\t" << "QUERY_SIZE = " << query_node_data_size[q] << endl;
            outfile << tree_node_data_size[t] << "\t" << query_node_data_size[q] << endl;
            // generate random points
            float* tree_node_data = new float[tree_node_data_size[t]*dimension];
            for (int i = 0; i < tree_node_data_size[t]; i++) {
                for (int j = 0; j < dimension; j++) {
                    tree_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1);
                }
            }

            //generate query node
            float* query_node_data = new float[query_node_data_size[q]*dimension];
            for (int i = 0; i < query_node_data_size[q]; i++) {
                for (int j = 0; j < dimension; j++) {
                    query_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1);
                }
            }

            //build tree
            SKDTree<float, dimension> static_kdtree1;
            DKDTree<float, dimension> dynamic_kdtree;

            SKDTree<float, dimension> static_kdtree2(tree_node_data, tree_node_data_size[t], false);

            for (int i = 0; i < tree_node_data_size[t]; i++) {
                vector<float> p;
                for (int j = 0; j < dimension; j++) {
                    p.push_back(tree_node_data[i*dimension+j]);
                }
                static_kdtree1.insertNode(p);
                dynamic_kdtree.insertNode(p);
            }
            static_ts1 = clock();
            for (int i = 0; i < query_node_data_size[q]; i++) {
                vector<float> p;
                for (int j = 0; j < dimension; j++) {
                    p.push_back(query_node_data[i*dimension+j]);
                }
                static_kdtree1.NNSearch(p);
            }
            static_te1 = clock();
            static_query_time1 = (double)(static_te1 - static_ts1)/CLOCKS_PER_SEC;

            static_ts2 = clock();
            for (int i = 0; i < query_node_data_size[q]; i++) {
                vector<float> p;
                for (int j = 0; j < dimension; j++) {
                    p.push_back(query_node_data[i*dimension+j]);
                }
                static_kdtree2.NNSearch(p);
            }
            static_te2 = clock();
            static_query_time2 = (double)(static_te2 - static_ts2)/CLOCKS_PER_SEC;


            dynamic_ts = clock();
            for (int i = 0; i < query_node_data_size[q]; i++) {
                vector<float> p;
                for (int j = 0; j < dimension; j++) {
                    p.push_back(query_node_data[i*dimension+j]);
                }
                dynamic_kdtree.NNSearch(p);
            }
            dynamic_te = clock();
            dynamic_query_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;
            outfile << static_query_time2 << "\t" << static_query_time1 << "\t" << dynamic_query_time << endl;
        }
    }
    outfile.close();
}

void testfun3() {
        ofstream outfile;
        outfile.open("result.txt");
        int N = 10*MAX_SAMPLE_NUM-1;
        const size_t dimension = 3;

        srand((int)time(NULL));
        for (int tree_node_data_size = 100; tree_node_data_size <= MAX_SAMPLE_NUM; tree_node_data_size = tree_node_data_size*10) {
            for (int query_node_data_size = 100; query_node_data_size <= MAX_QUERY_NUM; query_node_data_size = query_node_data_size*10) {
                cout << "DATA_SIZE = " << tree_node_data_size << "\t" << "QUERY_SIZE = " << query_node_data_size << endl;
                // generate random points
                float* tree_node_data = new float[tree_node_data_size*dimension];
                for (int i = 0; i < tree_node_data_size; i++) {
                    for (int j = 0; j < dimension; j++) {
                        tree_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1);
                    }
                }

                //generate query node
                float* query_node_data = new float[query_node_data_size*dimension];
                for (int i = 0; i < query_node_data_size; i++) {
                    for (int j = 0; j < dimension; j++) {
                        query_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1);
                    }
                }

                //build tree
                DKDTree<float, dimension> dynamic_kdtree;
                clock_t static_ts, static_te;
                double static_cons_time, static_query_time;
                double dynamic_cons_time, dynamic_query_time;
                static_ts = clock();
//                for (int i = 0; i < tree_node_data_size; i++) {
//                    vector<float> p;
//                    for (int j = 0; j < dimension; j++) {
//                        p.push_back(tree_node_data[i*dimension+j]);
//                    }
//                    static_kdtree.insertNode(p);
//                }
                SKDTree<float, dimension> static_kdtree(tree_node_data, tree_node_data_size, false);
                static_te = clock();
                static_cons_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;
                cout << "Static Kd Tree construction time = " << static_cons_time << "s" << endl;

                clock_t dynamic_ts, dynamic_te;
                dynamic_ts = clock();
                for (int i = 0; i < tree_node_data_size; i++) {
                    vector<float> p;
                    for (int j = 0; j < dimension; j++) {
                        p.push_back(tree_node_data[i*dimension+j]);
                    }
                    dynamic_kdtree.insertNode(p);
                }
                dynamic_te = clock();
                dynamic_cons_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;
                cout << "Dynamic Kd Tree construction time = " << dynamic_cons_time << "s" << endl;

                static_ts = clock();
                for (int i = 0; i < query_node_data_size; i++) {
                    vector<float> p;
                    vector<float> static_nn_result;
                    for (int j = 0; j < dimension; j++) {
                        p.push_back(query_node_data[i*dimension+j]);
                    }
                    static_nn_result = static_kdtree.NNSearch(p);
                }
                static_te = clock();
                static_query_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;
                cout << "Static Kd Tree query time = " << static_query_time << "s" << endl;

                dynamic_ts = clock();
                for (int i = 0; i < query_node_data_size; i++) {
                    vector<float> p;
                    vector<float> dynamic_nn_result;
                    for (int j = 0; j < dimension; j++) {
                        p.push_back(query_node_data[i*dimension+j]);
                    }
                    dynamic_nn_result = dynamic_kdtree.NNSearch(p);
                }
                dynamic_te = clock();
                dynamic_query_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;
                cout << "Dynamic Kd Tree query time = " << dynamic_query_time << "s" << endl;

                outfile << "DATA_SIZE = " << tree_node_data_size << "\t" << "QUERY_SIZE = " << query_node_data_size << endl;
                outfile << "Construction time = " << static_cons_time << "\t" << dynamic_cons_time << endl;
                outfile << "Query time = " << static_query_time << "\t" << dynamic_query_time << endl << endl;;
            }
        }
        outfile.close();
}

void testfun4() {
    ofstream outfile;
    outfile.open("../DynamicKDTree/experiment/3.30/testfun4_100000.txt");
//    string des = "change NN return";
    outfile << "change NN return type" << endl;
    srand((int)time(NULL));
    int N = 9999999;
    int query_node_data_size = 10000;
    int rebuild_step[50] = {50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,\
                            1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,\
                            2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000,\
                            4000, 5000, 6000, 7000, 8000, 9000};
//    int rebuild_step[50] = {100, 500, 1000, 2000, 3000, 4000, 5000, 6000, 6500, 7000, 8000, 9000, 10000, 20000, 50000};

//    int query_node_data_size = 10;
    const size_t dimension = 3;
    int scale = 100;
    float step = 1;
    //generate query node
    float* query_node_data = new float[query_node_data_size*dimension];
    outfile << query_node_data_size << endl;

    clock_t static_ts, static_te, dynamic_ts, dynamic_te, static_insert_ts, static_insert_te, static_query_ts, static_query_te, dynamic_insert_ts, dynamic_insert_te, dynamic_query_ts, dynamic_query_te;
    double static_time, dynamic_time, static_insert_time, static_quert_time, dynamic_insert_time, dynamic_query_time;

    for (int k = 0; k < 37; k++) {
        static_quert_time = 0;
        dynamic_query_time = 0;
        cout << "rebuild_step = " << rebuild_step[k] << endl;
        outfile << rebuild_step[k] << "\t";
        for (int i = 0; i < query_node_data_size; i++) {
            for (int j = 0; j < dimension; j++) {
                query_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1)*scale;
            }
        }
        //build tree
        SKDTree<float, dimension> static_kdtree;
        DKDTree<float, dimension> dynamic_kdtree;

        //insert the first node into the both trees
        vector<float> p;
        for (size_t i = 0; i < dimension; i++) p.emplace_back(rand()%(N+1)/(float)(N+1)*scale);
        static_kdtree.insertNode(p);
        dynamic_kdtree.insertNode(p);
        static_ts = clock();
        for (int i = 0; i < query_node_data_size; i++) {
            //nn search first
            vector<float> static_random_point, static_nn_point, static_extend_point;
            for (size_t j = 0; j < dimension; j++) static_random_point.emplace_back(query_node_data[i*dimension+j]);
            static_query_ts = clock();
            static_nn_point = static_kdtree.NNSearch(static_random_point);
            static_query_te = clock();
            static_quert_time += (double)(static_query_te - static_query_ts)/CLOCKS_PER_SEC;
//            static_insert_te = clock();
//            static_insert_time += (double)(static_insert_te - static_insert_ts)/CLOCKS_PER_SEC;
            //extend a step toward random_point
            static_extend_point = getExtendPoint(static_nn_point, static_random_point, step);

            //insert extend point into tree

            static_kdtree.insertNode(static_extend_point);
            //rebuild static kd tree
            if (i%rebuild_step[k] == (rebuild_step[k]-1)) {
                static_kdtree.rebuild();
            }
        }
        static_te = clock();
        static_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;

        cout << "Static total time = " << static_time << endl;
        outfile << static_time << "\t" << static_quert_time << "\t";
        dynamic_ts = clock();
        for (int i = 0; i < query_node_data_size; i++) {
            //nn search first
            vector<float> dynamic_random_point, dynamic_nn_point, dynamic_extend_point;
            for (size_t j = 0; j < dimension; j++) dynamic_random_point.emplace_back(query_node_data[i*dimension+j]);
            dynamic_query_ts = clock();
            dynamic_nn_point = dynamic_kdtree.NNSearch(dynamic_random_point);
            dynamic_query_te = clock();
            dynamic_query_time += (double)(dynamic_query_te - dynamic_query_ts)/CLOCKS_PER_SEC;
            //extend a step toward random_point
            dynamic_extend_point = getExtendPoint(dynamic_nn_point, dynamic_random_point, step);

            //insert extend point into tree

            dynamic_kdtree.insertNode(dynamic_extend_point);

        }

        dynamic_te = clock();
        dynamic_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;

        cout << "Dynamic total time = " << dynamic_time << endl;
        cout << "----------------------------------------------" << endl;

        outfile << dynamic_time << "\t" << dynamic_query_time << endl;
    }
    outfile.close();


}

void testfun5() {
    ofstream outfile;
    outfile.open("experiment/testfun5_2^15-1.txt");
    srand((int)time(NULL));
    int N = 999999;
    int first_data_size = (1<<15)-1;
//    int second_data_size[20] = {(1<<0), (1<<2), (1<<3), 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 3800, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
    int second_data_size[20];
    for (int i = 0; i < 15; i++) {
        second_data_size[i] = (1<<i);
    }
    const size_t dimension = 3;
    int scale = 100;
    outfile << first_data_size << endl;
    for (int k = 0; k < 15; k++) {
        float* node_data = new float[(first_data_size+second_data_size[k])*dimension];

        for (int i = 0; i < (first_data_size+second_data_size[k]); i++) {
            for (int j = 0; j < dimension; j++) {
                node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1)*scale;
            }
        }

        cout << "first " << first_data_size << " nodes: " << endl;
        //build tree
        clock_t static_ts, static_te, dynamic_ts, dynamic_te;
        double static_time, dynamic_time;
        static_ts = clock();
        SKDTree<float, dimension> static_kdtree1(node_data, first_data_size, false);
        for (int i = 0; i < first_data_size; i++) {
            //nn search first
            vector<float> static_random_point;
            for (size_t j = 0; j < dimension; j++) static_random_point.emplace_back(node_data[i*dimension+j]);
            //insert point into tree
        }
        static_te = clock();
        static_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;
        cout << "Static kd tree build time = " << static_time << endl;

        DKDTree<float, dimension> dynamic_kdtree;
        dynamic_ts = clock();
        for (int i = 0; i < first_data_size; i++) {
            //nn search first
            vector<float> dynamic_random_point;
            for (size_t j = 0; j < dimension; j++) dynamic_random_point.emplace_back(node_data[i*dimension+j]);
            //insert point into tree
            dynamic_kdtree.insertNode(dynamic_random_point);
        }
        dynamic_te = clock();
        dynamic_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;
        cout << "Dynamic kd tree build time = " << dynamic_time << endl;
        cout << "---------------------------------------------" << endl;

        cout << "second " << second_data_size[k] << " nodes: " << endl;
        outfile << second_data_size[k] << "\t";
        //build tree
        static_ts = clock();
        SKDTree<float, dimension> static_kdtree2(node_data, (first_data_size+second_data_size[k]), false);
        for (int i = first_data_size; i < (first_data_size+second_data_size[k]); i++) {
            //nn search first
            vector<float> static_random_point;
            for (size_t j = 0; j < dimension; j++) static_random_point.emplace_back(node_data[i*dimension+j]);
            //insert point into tree
        }
        static_te = clock();
        static_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;
        cout << "Static kd tree build time = " << static_time << endl;

        dynamic_ts = clock();
        for (int i = first_data_size; i < (first_data_size+second_data_size[k]); i++) {
            //nn search first
            vector<float> dynamic_random_point;
            for (size_t j = 0; j < dimension; j++) dynamic_random_point.emplace_back(node_data[i*dimension+j]);
            //insert point into tree
            dynamic_kdtree.insertNode(dynamic_random_point);
        }
        dynamic_te = clock();
        dynamic_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;
        cout << "Dynamic kd tree build time = " << dynamic_time << endl;
        cout << "---------------------------------------------" << endl;
        outfile << static_time << "\t" << dynamic_time << endl;
    }
    outfile.close();
}


void testfun6() {
    ofstream outfile;
    outfile.open("../DynamicKDTree/experiment/3.28/testfun6_2000_3000.txt");
    srand((int)time(NULL));
    int N = 999999;
    int insert_times = 100;
    int insert_data_size = 2000;
    int query_data_size = 3000;

    const size_t dimension = 3;
    int scale = 100;


    float* insert_node_data = new float[insert_data_size*insert_times*dimension];
    float* query_node_data = new float[query_data_size*dimension];
    DKDTree<float, dimension> dynamic_kdtree;
    for (int k = 0; k < insert_times; k++) {
        outfile << insert_data_size*(k+1) << "\t";
        for (int i = insert_data_size*k; i < insert_data_size*(k+1); i++) {
            for (int j = 0; j < dimension; j++) {
                insert_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1)*scale;
            }
        }

        for (int i = 0; i < query_data_size; i++) {
            for (int j = 0; j < dimension; j++) {
                query_node_data[i*dimension+j] = rand()%(N+1)/(float)(N+1)*scale;
            }
        }

        //build tree
        clock_t static_ts1, static_te2, static_ts, static_te, dynamic_ts, dynamic_te;
        double static_build_time, static_query_time, static_total_time, dynamic_build_time, dynamic_query_time, dynamic_total_time;
        static_ts = clock();
        SKDTree<float, dimension> static_kdtree(insert_node_data, insert_data_size*(k+1), false);
        static_te = clock();
        static_build_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;


        static_ts = clock();
        for (int i = 0; i < query_data_size; i++) {
            vector<float> static_query_point;
            for (size_t j = 0; j < dimension; j++) static_query_point.emplace_back(query_node_data[i*dimension+j]);
            //
            static_kdtree.NNSearch(static_query_point);
        }
        static_te = clock();
        static_query_time = (double)(static_te - static_ts)/CLOCKS_PER_SEC;


        dynamic_ts = clock();
        for (int i = insert_data_size*k; i < insert_data_size*(k+1); i++) {
            // nn search first
            vector<float> dynamic_random_point;
            for (size_t j = 0; j < dimension; j++) dynamic_random_point.emplace_back(insert_node_data[i*dimension+j]);
            // insert point into tree
            dynamic_kdtree.insertNode(dynamic_random_point);
        }
        dynamic_te = clock();
        dynamic_build_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;

        dynamic_ts = clock();
        for (int i = 0; i < query_data_size; i++) {
            vector<float> dynamic_query_point;
            for (size_t j = 0; j < dimension; j++) dynamic_query_point.emplace_back(query_node_data[i*dimension+j]);
            //
            dynamic_kdtree.NNSearch(dynamic_query_point);
        }
        dynamic_te = clock();
        dynamic_query_time = (double)(dynamic_te - dynamic_ts)/CLOCKS_PER_SEC;

        cout << "Static kd tree time = " << (static_build_time+static_query_time) << "\t" << static_build_time << "\t" << static_query_time << endl;

        cout << "Dynamic kd tree time = " << (dynamic_build_time+dynamic_query_time) << "\t" << dynamic_build_time << "\t" << dynamic_query_time << endl;

        outfile << (static_build_time+static_query_time) << "\t" << (dynamic_build_time+dynamic_query_time) << "\t" << static_build_time << "\t" << static_query_time << "\t" << dynamic_build_time << "\t" << dynamic_query_time << endl;
    }

    outfile.close();
}
void testLazy() {
    int dec[100];
    for (int i = 0; i < 100; i++) {
        dec[i] = i;
    }

    int level_arr[15]={0};
    int new_arr[15]={0};
    int n_samples = 0;
    int level_num = 1;
    for (int i = 1; i < 34; i++) {
        n_samples++;
        for (int i = level_num-1; i >= 0; i--) {
            if (level_arr[i] >= 2) {
                if (i == level_num-1 && level_arr[i] == 2) level_num++;
                new_arr[i+1] += 1;
                if (new_arr[i+1] == 1<<(i+1)) {
                    level_arr[i] -= 2;
                    level_arr[i+1] += 1;
                    new_arr[i+1] = 0;
                }
            }
        }

        level_arr[0] += 1;
        new_arr[0] = 0;

        for (int i = 0; i < level_num; i++) {
            cout << level_arr[level_num-1-i];
        }
        cout << "\t";

        for (int i = 0; i < level_num; i++) {
            cout << new_arr[level_num-1-i];
        }
        cout << "\t";
//        if (flag) level_num++;
        cout << level_num << endl;
    }
}
