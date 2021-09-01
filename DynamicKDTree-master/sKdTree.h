#ifndef SKDTREE_H
#define SKDTREE_H
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <string>
#include <stack>
#include <cstring>
#include <math.h>

#include "addon.h"

using namespace std;

template <typename ElemType, size_t K>
class SKDTree {
public:
    SKDTree();
    SKDTree(const int total_num);
    SKDTree(ElemType* datas, const size_t n_samples, const bool is_allocate);

    // copy constructor, deep copy
//    SKDTree(const SKDTree &tree);
    ~SKDTree();
    void initBuffer();

    void clear();
    //build tree part
    sNode* build(const vector<size_t> &points, size_t depth = 0);
    size_t findSplitDim(const vector<size_t> &points, size_t depth);
    size_t midElement(const vector<size_t> &points, const size_t dim, vector<size_t> &left_points, vector<size_t> &right_points);
    void rebuild();

    void insertRebuild(const vector<ElemType> &point);
    void insertRebuild(ElemType* data_arr);

    //insert part
    void insertNode(const vector<ElemType> &point);
    sNode* insert(const vector<ElemType> &point, sNode* root, const int depth);

    //generate point part
    void randomPoints(const int num, const int dim);
    void readPointsFromFile(const string fn);

    //search part
    vector<ElemType> NNSearch(const vector<ElemType> &point);
    const sNode* search(const vector<ElemType> &point, sNode* root, const int &depth);

    //visualization part
    void printTree();
    void Print(const sNode* root, Trunk* prev, const bool isLeft);
    void printsNode(const sNode* sNode);

    //utils part
    vector<ElemType> getPointByID(size_t id);
    bool isEqual(ElemType* point, const int &id);
    bool isEqual(const vector<ElemType> &point, const int &id);
    ElemType getDimValue(const size_t sample, const size_t dim);

    ElemType getDist(const sNode *node, const vector<ElemType> &point);
    ElemType getDist(const int idx, const vector<ElemType> &point);
    ElemType getDist2(const sNode *node, const vector<ElemType> &point);
    ElemType getDist2(const size_t idx1, const size_t idx2);
    ElemType getDist2(const size_t idx, const vector<ElemType> &point);
    size_t getCloserIdx(const size_t idx1, const size_t idx2, const vector<ElemType> &point);
    const sNode* getCloserNode(const sNode *n1, const sNode *n2, const vector<ElemType> &point);

    ElemType bruteForce(const vector<ElemType> &p);
    size_t getTreeSize();

private:
    //is_allocate_
    bool is_allocate_;
    int total_num_;
    //树根节点
    sNode* root_;
    //数据集
    ElemType* datas_;
    //总的样本数
    size_t n_samples_;
    //样本的维数
    size_t n_dimensions_;
    // 寻找中位数时用到的缓存池
    tuple<size_t, ElemType> *get_mid_buf_;
    // 最近邻查询的结果
    vector<ElemType> nn_result_ = vector<ElemType>(K);
};


template <typename ElemType, size_t K>
SKDTree<ElemType, K>::SKDTree() :
    root_(NULL), n_samples_(0), n_dimensions_(K), is_allocate_(true), total_num_(MAX_SAMPLE_NUM) {
//    total_num_ = MAX_SAMPLE_NUM;
    datas_ = new ElemType[total_num_*K];
//    nn_result_.reserve(K);
    initBuffer();
}

template <typename ElemType, size_t K>
SKDTree<ElemType, K>::SKDTree(const int total_num):
    root_(NULL), n_samples_(0), n_dimensions_(K), total_num_(total_num), is_allocate_(true) {
    datas_ = new ElemType[total_num_*K];
//    nn_result_.reserve(K);
    initBuffer();
}


template <typename ElemType, size_t K>
SKDTree<ElemType, K>::SKDTree(ElemType datas[], const size_t n_samples, const bool is_allocate) :
    n_samples_(n_samples), n_dimensions_(K), is_allocate_(is_allocate) {
    vector<size_t> points;

    for (size_t i = 0; i < n_samples_; i++)
        points.emplace_back(i);
    if (!is_allocate_) datas_ = datas;
    else memcpy(datas_, datas, sizeof(ElemType)*n_samples);
//    nn_result_.reserve(K);
    initBuffer();
    root_ = build(points, 0);
}

//template<typename ElemType, size_t K>
//SKDTree<ElemType, K>::SKDTree(const SKDTree *tree) {

//}

template <typename ElemType, size_t K>
SKDTree<ElemType, K>::~SKDTree() {
    delete[] get_mid_buf_;
    if (is_allocate_) delete[] datas_;
    freeTreeMemory(root_);
    root_ = NULL;
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::clear() {
    n_samples_ = 0;
    freeTreeMemory(root_);
    root_ = NULL;
}

/*******************************************************************/
/******************** build tree part ******************************/
template <typename ElemType, size_t K>
sNode* SKDTree<ElemType, K>::build(const vector<size_t> &points, size_t depth) {
    if (points.size() == 0)
        return NULL;
    else {
        size_t dim = findSplitDim(points, depth);
        vector<size_t> left_points, right_points;
        size_t mid_id = midElement(points, dim, left_points, right_points);

        sNode *s_node = new sNode();
        s_node->id = mid_id;
        s_node->depth = depth;
        s_node->split_dim = dim;
        s_node->left_child = build(left_points, depth+1);
        s_node->right_child = build(right_points, depth+1);

        return s_node;
    }
}

template <typename ElemType, size_t K>
size_t SKDTree<ElemType, K>::midElement(const vector<size_t> &points, const size_t dim, vector<size_t> &left_points, vector<size_t> &right_points) {
    for (size_t i = 0; i < points.size(); i++)
        get_mid_buf_[i] = make_tuple(points[i], getDimValue(points[i], dim));

    nth_element(get_mid_buf_,
                get_mid_buf_ + points.size()/2,
                get_mid_buf_ + points.size(),
                compareFun<ElemType>);

    for (size_t i = 0; i < points.size()/2; i++) {
        left_points.emplace_back(get<0>(get_mid_buf_[i]));
    }
    for (size_t i = points.size()/2 + 1; i < points.size(); i++) {
        right_points.emplace_back(get<0>(get_mid_buf_[i]));
    }

    return get<0>(get_mid_buf_[points.size()/2]);
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::initBuffer() {
    if(is_allocate_) {
        get_mid_buf_ = new tuple<size_t, ElemType>[total_num_];
    }
    else {
        get_mid_buf_ = new tuple<size_t, ElemType>[n_samples_];
    }
}

template <typename ElemType, size_t K>
size_t SKDTree<ElemType, K>::findSplitDim(const vector<size_t> &points, size_t depth) {
    return depth % K;
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::rebuild() {
    vector<size_t> points;
    for (size_t i = 0; i < n_samples_; i++) points.emplace_back(i);
    root_ = build(points,0);
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::insertRebuild(const vector<ElemType> &point) {
    for (size_t i = 0; i < n_dimensions_; i++) datas_[n_samples_*n_dimensions_+i] = point[i];
    n_samples_++;
    vector<size_t> points;
    for (size_t i = 0; i < n_samples_; i++) points.emplace_back(i);
    root_ = build(points,0);
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::insertRebuild(ElemType* data_arr) {
    for (size_t i = 0; i < n_dimensions_; i++) datas_[n_samples_*n_dimensions_+i] = data_arr[i];
    n_samples_++;
    vector<size_t> points;
    for (size_t i = 0; i < n_samples_; i++) points.emplace_back(i);
    root_ = build(points,0);
}


/*******************************************************************/
/******************** insert node part ****************************/
template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::insertNode(const vector<ElemType> &point) {
    root_ = insert(point, root_, 0);
}

template <typename ElemType, size_t K>
sNode* SKDTree<ElemType, K>::insert(const vector<ElemType> &point, sNode* root, const int depth) {
    if (root == NULL) {
        // add point into datas_
        for(size_t i = 0; i < n_dimensions_; i++) {
            datas_[n_samples_*n_dimensions_+i] = point[i];
        }
        root = new sNode();
        root->id = n_samples_;
        n_samples_++;
        root->depth = 0;
        root->left_child = NULL;
        root->right_child = NULL;

    } else if (isEqual(point, root->id)) {
        cout << "This point in already in data set!!!" << endl;
    } else {
        if (point[root->split_dim] < getDimValue(root->id, root->split_dim)) {
            root->left_child = insert(point, root->left_child, depth+1);
        } else {
            root->right_child = insert(point, root->right_child, depth+1);
        }
    }
    return root;
}

/*******************************************************************/
/********************generate point part****************************/
template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::randomPoints(const int num, const int dim) {
    //
    ElemType* data = (ElemType*)malloc(num*dim*sizeof(ElemType));
    for(int i = 0; i < num; i++) {
        for(int j = 0; j < dim; j++) {
            data[i*dim+j] = 0;
        }
    }
    datas_ = data;
    n_samples_ = num;
    n_dimensions_ = dim;
}

template<typename ElemType, size_t K>
void SKDTree<ElemType, K>::readPointsFromFile(const string fn){
    fn = "./data" + fn;
    FILE* fp = fopen(fn.c_str(), "r");
    if (fp) {
        fscanf(fp, " %d %d", &n_samples_, &n_dimensions_);
        cout << "number of samples:" << n_samples_ << endl << \
                "dimention of samples: " << n_dimensions_ << endl;
        ElemType* data = (ElemType*) malloc(n_samples_*n_dimensions_*sizeof(ElemType));
        for(int i = 0; i < n_samples_; i++) {
            fscanf(fp, "%lf %lf %lf", &data[i*n_dimensions_], &data[i*n_dimensions_+1], &data[i*n_dimensions_+2]);
        }
        datas_ = data;
    }
    else {
        cout << "No such file: " << fn << endl;
        exit(1);
    }
}

/*******************************************************************/
/********************** search part ********************************/
template<typename ElemType, size_t K>
vector<ElemType> SKDTree<ElemType, K>::NNSearch(const vector<ElemType> &point) {
    vector<ElemType> res;
    if(false/*!isValid(point)*/) {
        cout << "point is illegal" << endl;
        return nn_result_;;
    } else {
        const sNode* node = search(point, root_, 0);
        for(int i = 0; i < n_dimensions_; i++) {
            res.emplace_back(getDimValue(node->id, i));
        }
        return res;
    }
}


template<typename ElemType, size_t K>
const sNode* SKDTree<ElemType, K>::search(const vector<ElemType> &point, sNode* root, const int &depth) {
    if (root == NULL) {
        return NULL;
    }
    const sNode* best_node;
    sNode* next_branch, *opposite_branch;
    if (point[root->split_dim] < getDimValue(root->id, root->split_dim)) {
        next_branch = root->left_child;
        opposite_branch = root->right_child;
    } else {
        next_branch = root->right_child;
        opposite_branch = root->left_child;
    }

    best_node = getCloserNode(search(point, next_branch, depth+1), root, point);

    if (getDist(best_node, point) > fabs(point[root->split_dim] - getDimValue(root->id, root->split_dim))) {
        best_node = getCloserNode(search(point, opposite_branch, depth+1), best_node, point);
    }
    return best_node;
}

/*******************************************************************/
/********************visualization part*****************************/
template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::printTree() {
    Print(root_, NULL, true);
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::Print(const sNode *root, Trunk *prev, const bool isLeft) {
    if (!root) {
        return;
    }
    string trunk_str = "      ";
    Trunk* trunk = new Trunk(prev, trunk_str);

    Print(root->left_child, trunk, true);

    if (!prev) {
        trunk->str = "------";
    }
    else if (isLeft) {
        trunk->str = ".------";
        trunk_str = "     |";
    }
    else {
        trunk->str = "`------";
        prev->str = trunk_str;
    }

    showTrunks(trunk);
    printsNode(root);

    if (prev) {
        prev->str = trunk_str;
    }

    trunk->str = "     |";
    Print(root->right_child, trunk, false);
}

template <typename ElemType, size_t K>
void SKDTree<ElemType, K>::printsNode(const sNode *sNode) {
    cout << "[";
    for (size_t i = 0; i < n_dimensions_; i++) {
        cout << " " << getDimValue(sNode->id, i);
    }
    cout << "]" << endl;
}

/*******************************************************************/
/************************** utils part *****************************/
template <typename ElemType, size_t K>
vector<ElemType> SKDTree<ElemType, K>::getPointByID(size_t id) {
    vector<ElemType> point;
    for(size_t i = 0; i < n_dimensions_;i++) {
        point.push_back(getDimValue(id, i));
    }
    return point;
}

template <typename ElemType, size_t K>
bool SKDTree<ElemType, K>::isEqual(ElemType* point, const int &id) {
    float* p = getPointByID(id);
    for (size_t i = 0; i < n_dimensions_; i++) {
        if (fabs(point[i]-p[i]) > EPSILON) {
            return false;
        }
    }
    return true;
}

template <typename ElemType, size_t K>
bool SKDTree<ElemType, K>::isEqual(const vector<ElemType> &point, const int &id) {
    vector<ElemType> p = getPointByID(id);
    for (size_t i = 0; i < n_dimensions_; i++) {
        if (fabs(point[i]-p[i]) > EPSILON) {
            return false;
        }
    }
    return true;
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDimValue(const size_t idx, const size_t dim) {
    return datas_[idx * n_dimensions_ + dim];
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDist(const sNode *node, const vector<ElemType> &point) {
    return sqrt(getDist2(node, point));
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDist(const int idx, const vector<ElemType> &point) {
    return sqrt(getDist2(idx, point));
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDist2(const sNode *node, const vector<ElemType> &point) {
    ElemType dist = 0;
    for (size_t i = 0; i < n_dimensions_; i++) {
        dist += (getDimValue(node->id, i) - point[i])*(getDimValue(node->id, i) - point[i]);
    }
    return dist;
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDist2(const size_t idx, const vector<ElemType> &point) {
    ElemType dist = 0;
    for (size_t i = 0; i < n_dimensions_; i++) {
        dist += (getDimValue(idx, i) - point[i])*(getDimValue(idx, i) - point[i]);
    }
    return dist;
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::getDist2(const size_t idx1, const size_t idx2) {
    ElemType dist = 0;
    for (int i = 0; i < n_dimensions_; i++) {
        dist += (getDimValue(idx1, i) - getDimValue(idx2, i))*(getDimValue(idx1, i) - getDimValue(idx2, i));
    }
    return dist;
}

template <typename ElemType, size_t K>
size_t SKDTree<ElemType, K>::getCloserIdx(const size_t idx1, const size_t idx2, const vector<ElemType> &point) {
    return (getDist2(idx1, point) < getDist2(idx2, point)) ? idx1 : idx2;
}

template <typename ElemType, size_t K>
const sNode* SKDTree<ElemType, K>::getCloserNode(const sNode *n1, const sNode *n2, const vector<ElemType> &point) {
    if(n1 == NULL) return n2;
    if(n2 == NULL)  return n1;
    return (getDist2(n1->id, point) < getDist2(n2->id, point)) ? n1 : n2;
}

template <typename ElemType, size_t K>
ElemType SKDTree<ElemType, K>::bruteForce(const vector<ElemType> &p) {
    ElemType closest_dist2, temp_dist2;
    for (size_t i = 0; i < n_samples_; i++) {
        temp_dist2 = getDist2(i, p);
        closest_dist2 = (i == 0) ? getDist2(i, p) : ((temp_dist2 < closest_dist2) ? temp_dist2 : closest_dist2);
    }
    return closest_dist2;
}

template <typename ElemType, size_t K>
size_t SKDTree<ElemType, K>::getTreeSize() {
    return n_samples_;
}


#endif // SKDTREE_H
