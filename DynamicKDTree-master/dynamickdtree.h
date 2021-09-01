#ifndef DYNAMICKDTREE_H
#define DYNAMICKDTREE_H
#include <vector>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <string>
#include "addon.h"

using namespace std;

struct Node
{
    size_t id;
    size_t depth;
    Node *left_child, *right_child;
};

template <typename ElemType, size_t K>
class DynamicKDTree {
public:
    DynamicKDTree();
    DynamicKDTree(const ElemType* datas, size_t rows, size_t cols);
    Node* build(const vector<size_t> &points, size_t depth = 0);
    size_t findSplitDim(const vector<size_t> &points, size_t depth);
    size_t midElement(const vector<size_t> &points, size_t dim, vector<size_t> &left_points, vector<size_t> &right_points);
    ElemType getDimValue(size_t sample, size_t dim);
    void initBuffer();
    void printTree();
    void Print(Node* root, Trunk* prev, bool isLeft);
    void printNode(Node* node);

    ~DynamicKDTree(){}
private:
    Node *root_;    //树根节点
    const ElemType* datas_;
    size_t n_samples_;  //总的样本数
    size_t n_dimensions_; //KDTree 的维数
    tuple<size_t, ElemType> *get_mid_buf_;
};
template <typename ElemType, size_t K>
DynamicKDTree<ElemType, K>::DynamicKDTree() :
    root_(NULL), datas_(NULL), n_samples_(0), n_dimensions_(0) {
//    cout << "in default constructor " <<
//            "K = " << K << endl;
}

template <typename ElemType, size_t K>
DynamicKDTree<ElemType, K>::DynamicKDTree(const ElemType *datas, size_t rows, size_t cols) :
    datas_(datas), n_samples_(rows), n_dimensions_(cols) {
//    cout << "in new constructor " << endl;
    vector<size_t> points;
    for (size_t i = 0; i < n_samples_; i++)
        points.emplace_back(i);
    initBuffer();
    root_ = build(points, 0);
}

template <typename ElemType, size_t K>
Node* DynamicKDTree<ElemType, K>::build(const vector<size_t> &points, size_t depth) {
    if (points.size() == 0)
        return NULL;
    else {
//        cout << "beigin build" << endl;
        size_t dim = findSplitDim(points, depth);
//        cout << "dim = " << dim << "\t" << "point.size = " << points.size() << endl;
        vector<size_t> left_points, right_points;
        size_t mid_id = midElement(points, dim, left_points, right_points);

        Node *node = new Node();
        node->id = mid_id;
        node->depth = depth;
        node->left_child = build(left_points, depth+1);
        node->right_child = build(right_points, depth+1);

        return node;
    }
}

template <typename ElemType, size_t K>
size_t DynamicKDTree<ElemType, K>::midElement(const vector<size_t> &points, size_t dim, vector<size_t> &left_points, vector<size_t> &right_points) {
    for (size_t i = 0; i < points.size(); i++)
        get_mid_buf_[i] = make_tuple(points[i], getDimValue(points[i], dim));
//    cout << "nth_element" << endl;
    nth_element(get_mid_buf_,
                get_mid_buf_ + points.size()/2,
                get_mid_buf_ + points.size(),
                compareFun<ElemType>);

//    cout << "left part" << endl;
    for (size_t i = 0; i < points.size()/2; i++) {
        left_points.emplace_back(get<0>(get_mid_buf_[i]));
//        cout << "left_point.size = " << left_points.size() << endl;
    }
//    cout << "right part" << endl;
    for (size_t i = points.size()/2 + 1; i < points.size(); i++) {
        right_points.emplace_back(get<0>(get_mid_buf_[i]));
//        cout << "right_points.size = " << right_points.size() << endl;
    }
    return get<0>(get_mid_buf_[points.size()/2]);
}

template <typename ElemType, size_t K>
void DynamicKDTree<ElemType, K>::initBuffer() {
    get_mid_buf_ = new tuple<size_t, ElemType>[n_samples_];
}
template <typename ElemType, size_t K>
size_t DynamicKDTree<ElemType, K>::findSplitDim(const vector<size_t> &points, size_t depth) {
    return depth % K;
}

template <typename ElemType, size_t K>
ElemType DynamicKDTree<ElemType, K>::getDimValue(size_t sample, size_t dim) {
    return datas_[sample * n_dimensions_ + dim];
}

template <typename ElemType, size_t K>
void DynamicKDTree<ElemType, K>::printTree() {
    Print(root_, NULL, true);
}

template <typename ElemType, size_t K>
void DynamicKDTree<ElemType, K>::Print(Node *root, Trunk *prev, bool isLeft) {
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
    printNode(root);

    if (prev) {
        prev->str = trunk_str;
    }

    trunk->str = "     |";
    Print(root->right_child, trunk, false);
}

template <typename ElemType, size_t K>
void DynamicKDTree<ElemType, K>::printNode(Node *node) {
    cout << "[";
    for (size_t i = 0; i < n_dimensions_; i++) {
        cout << " " << getDimValue(node->id, i);
    }
    cout << "]" << endl;
}
#endif // DYNAMICKDTREE_H
