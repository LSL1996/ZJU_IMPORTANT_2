#ifndef DKDTREE_H
#define DKDTREE_H
#include <iostream>
#include <vector>
#include "addon.h"
#include "sKdTree.h"

using namespace std;
template <typename ElemType, size_t K>
class DKDTree
{
public:
    DKDTree();
    DKDTree(ElemType* datas, const size_t n_samples);
    ~DKDTree();
    //insert part
    void insertNode(const vector<ElemType> &point);
    //search part
    vector<ElemType> NNSearch(const vector<ElemType> &point);
    //visualization part
    void printTree();

    size_t getTreeSize() {
        return n_samples_;
    }

private:
    //数据集
    ElemType* datas_;
    //总的样本数
    size_t n_samples_;
    //样本的维数
    size_t n_dimensions_;
    //kd tree的集合
    vector<SKDTree<ElemType, K>*> trees_;
    //
    bool is_allocate_;
};

template <typename ElemType, size_t K>
DKDTree<ElemType, K>::DKDTree(): n_samples_(0), n_dimensions_(K), is_allocate_(true) {
    datas_ = new ElemType[MAX_SAMPLE_NUM*n_dimensions_];
}

template <typename ElemType, size_t K>
DKDTree<ElemType, K>::DKDTree(ElemType* datas, const size_t n_samples)\
    :datas_(datas), n_samples_(n_samples), n_dimensions_(K), is_allocate_(false) {
    size_t num = n_samples_;
    int cnt = 0;
    int built_tree_samples = 0;
    while (num != 0) {
        if(num%2 == 1) {
            int new_tree_samples = 1<<cnt;
            built_tree_samples += new_tree_samples;
            int start_id = n_samples_ - built_tree_samples;
            SKDTree<ElemType, K>* new_tree = new SKDTree<ElemType, K>(datas_+start_id*n_dimensions_, new_tree_samples, false);
            trees_.push_back(new_tree);
        }
        num /= 2;
        cnt++;
    }

}

template <typename ElemType, size_t K>
DKDTree<ElemType, K>::~DKDTree() {
    if (is_allocate_) delete[] datas_;
}

/*******************************************************************/
/******************** insert node part ****************************/
template <typename ElemType, size_t K>
void DKDTree<ElemType, K>::insertNode(const vector<ElemType> &point) {
    if (n_dimensions_ != point.size()) {
        cout << "Point dimension wrong!!!" << endl;
        return;
    }
    for(size_t i = 0; i < n_dimensions_; i++) {
        datas_[n_samples_*n_dimensions_+i] = point.at(i);
    }
    int cnt = 0;
    size_t num = n_samples_;
    while(num%2 != 0) {
        delete trees_.back();
        trees_.pop_back();
        num /= 2;
        cnt++;
    }
    n_samples_++;
    int new_tree_samples = 1<<cnt;
    int start_id = n_samples_ - new_tree_samples;
    SKDTree<ElemType, K>* new_tree = new SKDTree<ElemType, K>(datas_+start_id*n_dimensions_, new_tree_samples, false);
    trees_.push_back(new_tree);
}

/*******************************************************************/
/********************** search part ********************************/
template <typename ElemType, size_t K>
vector<ElemType> DKDTree<ElemType, K>::NNSearch(const vector<ElemType> &point) {
    vector<ElemType> best_point(K);
    if(trees_.size() == 0 /*!isValid(point)*/) {
        cout << "Tree is Empty!!!" << endl;
        return best_point;
    } else {
        for (int i = 0; i < trees_.size(); i++) {
//            cout << getDist2(trees_[i]->NNSearch(point), point) << endl;
            best_point = (i == 0) ? trees_[i]->NNSearch(point) : getCloserPoint(best_point, trees_[i]->NNSearch(point), point);
        }
        return best_point;
    }
}

/*******************************************************************/
/********************visualization part*****************************/
template <typename ElemType, size_t K>
void DKDTree<ElemType, K>::printTree() {
    int cnt = 0;
    for(SKDTree<ElemType, K>* tree : trees_) {
        cout << "tree[" << cnt << "]: size = " << tree->getTreeSize() << endl << endl;
        tree->printTree();
        cnt++;
        cout << endl;
    }
}


#endif // DKDTREE_H
