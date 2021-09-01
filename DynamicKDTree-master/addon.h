#ifndef ADDON_H
#define ADDON_H
#include <vector>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <string>
#include <stack>
#include <math.h>
#include <cstring>
using namespace std;

#ifndef EPSILON
#define EPSILON 1e-9
#endif

#ifndef MAX_SAMPLE_NUM
#define MAX_SAMPLE_NUM 1000000

#endif

#ifndef MAX_QUERY_NUM
#define MAX_QUERY_NUM 10000

#endif

enum LayerTypes {
    OLDEST = 0,
    OLDER = 1,
    OLD  = 2,
    NEW = 3,
};

// for tree visualization
struct Trunk
{
    Trunk *prev;
    string str;

    Trunk(Trunk *prev, string str)
    {
        this->prev = prev;
        this->str = str;
    }
};

inline void showTrunks(Trunk *p)
{
    if (p == NULL)
        return;

    showTrunks(p->prev);

    cout << p->str;
}


struct sNode
{
    size_t id;
    size_t depth;
    size_t split_dim;
    sNode *left_child, *right_child;
};

template<typename ElemType>
ElemType getDist2(const vector<ElemType> &p1, const vector<ElemType> &p2) {
    ElemType dist = 0;
    for (int i = 0; i < p1.size(); i++)
        dist += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    return dist;
}

template<typename ElemType>
ElemType getDist(const vector<ElemType> &p1, const vector<ElemType> &p2) {
    return sqrt(getDist2(p1, p2));
}

template<typename ElemType>
vector<ElemType> getCloserPoint(const vector<ElemType> &p1, const vector<ElemType> &p2, const vector<ElemType> &target) {
    if (p1.size() == 0) return p2;
    if (p2.size() == 0) return p1;
    return (getDist2(p1, target) < getDist2(p2, target)) ? p1 : p2;
}

template<typename ElemType>
vector<ElemType> getUnitVector(const vector<ElemType> &from, const vector<ElemType> &to) {
    vector<ElemType> unit;
    ElemType dist = getDist(from, to);
    for (size_t i = 0; i < from.size(); i++) {
        unit.emplace_back((to[i] - from[i])/dist);
    }
    return unit;
}

template<typename ElemType>
vector<ElemType> getExtendPoint(const vector<ElemType> &from, const vector<ElemType> &to, const ElemType &step) {
    vector<ElemType> unit_vec = getUnitVector(from, to);
    vector<ElemType> extend_point;
    for (size_t i = 0; i < from.size(); i++) {
        extend_point.emplace_back(from[i] + unit_vec[i]*step);
    }
    return extend_point;
}


template <typename ElemType>
bool compareFun(const tuple<size_t, ElemType> &i, const tuple<size_t, ElemType> &j) {return get<1>(i) < get<1>(j);}


void freeTreeMemory(sNode *root) {
    std::stack<sNode *> node_stack;
    sNode *p;
    node_stack.push(root);
    while (!node_stack.empty()) {
        p = node_stack.top();
        node_stack.pop();
        if (p->left_child)
            node_stack.push(p->left_child);
        if (p->right_child)
            node_stack.push(p->right_child);
        delete p;
        p = NULL;
    }
}

template <typename ElemType>
vector<ElemType> getPointById(ElemType* datas, const size_t &dim, const size_t &id) {
    vector<ElemType> p;
    for (size_t i = 0; i < dim; i++) {
        p.push_back(datas[id*dim+i]);
    }
    return p;
}

#endif // ADDON_H
