#ifndef LAZYREBUILD_H
#define LAZYREBUILD_H
#include <iostream>
#include <vector>
#include "addon.h"
#include "sKdTree.h"

using namespace std;


template<typename ElemType, size_t K>
class DKDTree2 {
public:
    class Level
    {
    public:
        Level(const size_t level_id) : level_id_(level_id), arr_capacity_(1<<level_id) {
            new_tree_ = new SKDTree<ElemType, K>(arr_capacity_);
        }

        Level(ElemType* data_arr, const int layer_num, const int new_num, const size_t level_id)\
           : level_id_(level_id), arr_capacity_(1<<level_id) {
            // build trees
            switch (layer_num) {
            case 1:
                oldest_tree_ = new SKDTree<ElemType, K>(data_arr, arr_capacity_, false);
                oldest_empty_ = false;
                if(new_num != 0) new_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*K, new_num, false);
                break;
            case 2:
                oldest_tree_ = new SKDTree<ElemType, K>(data_arr, arr_capacity_, false);
                oldest_empty_ = false;
                older_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*K, arr_capacity_, false);
                older_empty_ = false;
                if (new_num != 0) new_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*2*K, new_num, false);
                break;
            case 3:
                oldest_tree_ = new SKDTree<ElemType, K>(data_arr, arr_capacity_, false);
                oldest_empty_ = false;
                older_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*K, arr_capacity_, false);
                older_empty_ = false;
                old_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*2*K, arr_capacity_, false);
                old_empty_ = false;
                if (new_num != 0) new_tree_ = new SKDTree<ElemType, K>(data_arr+arr_capacity_*3*K, new_num, false);
                break;
            default:
                if (new_num != 0) new_tree_ = new SKDTree<ElemType, K>(data_arr, new_num, false);
                break;
            }
        }

        void addItem(const vector<ElemType> &point) {
            new_tree_->insertRebuild(point);
        }

        void addItem(ElemType* data_arr) {
            if (!new_tree_) {
                new_tree_ = new SKDTree<ElemType, K>(arr_capacity_);
                new_tree_->insertRebuild(data_arr);
            } else {
                new_tree_->insertRebuild(data_arr);
            }
        }

        void age() {
//            cout << "age before" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;
            if (oldest_empty_) {
                oldest_tree_ = new_tree_;
                oldest_empty_ = false;
                new_tree_ = NULL;
            } else if (older_empty_) {
                older_tree_ = new_tree_;
                older_empty_ = false;
                new_tree_ = NULL;
            } else if (old_empty_) {
                old_tree_ = new_tree_;
                old_empty_ = false;
                new_tree_ = NULL;
            }
//            cout << "age end" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;
        }

        void destroyTree(const int layer) {
//            cout << "destroyTree begin" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;
            if (layer == OLDEST) {
                delete oldest_tree_;
                oldest_empty_ = true;
            } else if (layer == OLDER) {
                delete older_tree_;
                older_empty_ = true;
                older_tree_ = NULL;
            }
//            switch (layer) {
//            case OLDEST:
//                cout << "delete oldest" << endl;
//                delete oldest_tree_;
//                cout << "end oldest" << endl;
//                oldest_empty_ = true;
////                oldest_tree_ = NULL;
//                break;
//            case OLDER:
//                cout << "delete older" << endl;
//                delete older_tree_;
//                older_empty_ = true;
//                older_tree_ = NULL;
//                break;
//            case OLD:
//                delete old_tree_;
//                old_empty_ = true;
//                old_tree_ = NULL;
//                break;
//            case NEW:
//                delete new_tree_;
//                old_empty_ = true;
//                new_tree_ = NULL;
//                break;
//            default:
//                break;
//            }
//            cout << "destroyTree end" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;

        }

        void copyTree(const int &dst, const int &src) {
            SKDTree<ElemType, K>* dst_p, *src_p;
//            cout << "copyTree before" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;
//            switch (dst) {
//            case OLDEST:
//                dst_p = oldest_tree_;
//                oldest_empty_ = false;
//                break;
//            case OLDER:
//                dst_p = older_tree_;
//                older_empty_ = false;
//                break;
//            case OLD:
//                dst_p = old_tree_;
//                old_empty_ = false;
//                break;
//            case NEW:
//                dst_p = new_tree_;
//                break;
//            default:
//                break;
//            }

//            switch (src) {
//            case OLDEST:
//                src_p = oldest_tree_;
//                oldest_empty_ = true;
//                break;
//            case OLDER:
//                src_p = older_tree_;
//                older_empty_ = true;
//                break;
//            case OLD:
//                src_p = old_tree_;
//                old_empty_ = true;
//                break;
//            case NEW:
//                src_p = new_tree_;
//                break;
//            default:
//                break;
//            }

            oldest_tree_ = old_tree_;
            oldest_empty_ = false;
            old_empty_ = true;

//            cout << "copyTree end" << endl;
//            cout << level_id_ << endl;
//            cout <<  oldest_tree_ << "\t" << oldest_empty_ << endl;
//            cout << older_tree_ << "\t" << older_empty_ << endl;
//            cout << old_tree_ << "\t" << old_empty_ << endl;
//            cout << new_tree_ << endl;
        }

        vector<ElemType> NNSearch(const vector<ElemType> &point) {
            vector<ElemType> best_point;
            if (oldest_tree_) best_point = oldest_tree_->NNSearch(point);

            if (older_tree_) best_point = best_point.empty() ? \
                        older_tree_->NNSearch(point) : getCloserPoint(best_point, older_tree_->NNSearch(point), point);
            if (old_tree_) best_point = best_point.empty() ? \
                        old_tree_->NNSearch(point) : getCloserPoint(best_point, old_tree_->NNSearch(point), point);

            return best_point;
        }

        void printLevel() {
            if (!oldest_empty_) {
                cout << "Oldest[" << level_id_ << "]" << endl;
                oldest_tree_->printTree();
            }

            if (!older_empty_) {
                cout << "Older[" << level_id_ << "]" << endl;
                older_tree_->printTree();
            }

            if (!old_empty_) {
                cout << "Old[" << level_id_ << "]" << endl;
                old_tree_->printTree();
            }

            if (new_tree_) {
                cout << "New[" << level_id_ << "]" << endl;
                new_tree_->printTree();
            }
        }

    private:
//        ElemType* data_arr_;
//        size_t *oldest_arr_, *older_arr_, *old_arr_, *new_arr__;
        SKDTree<ElemType, K> *oldest_tree_=NULL, *older_tree_=NULL, *old_tree_=NULL, *new_tree_=NULL;
        // empty flag
        bool oldest_empty_ = true;
        bool older_empty_ = true;
        bool old_empty_ = true;
        int new_arr__size_;

        // 第i层
        size_t level_id_;
        // a
        int arr_capacity_;

        int layer_num_;
        int new_num_;


    };

    DKDTree2():n_samples_(0), n_dimensions_(K), is_allocate_(true) {
        data_arr_ = new ElemType[MAX_SAMPLE_NUM*n_dimensions_];
        level_num_ = 1;
        Level* level = new Level(0);
        level_vec_.emplace_back(level);
    }

    DKDTree2(ElemType* data_arr, const size_t n_samples):\
        data_arr_(data_arr), n_samples_(n_samples), n_dimensions_(K), is_allocate_(false) {
        //copy data_arr to data_arr_
//        memcpy(data_arr_, data_arr, sizeof(size_t)*n_samples_);

        int level_num = std::log2(n_samples_+1);
        int num = n_samples_;
        int cnt = 0;
        while(num != 0) {
            if (num/(1<<cnt) == 1) {
                level_arr_[cnt] = 1;
            } else {
                if (num/(1<<cnt)%2 == 0) {
                    level_arr_[cnt] = 2;
                } else {
                    level_arr_[cnt] = 3;
                }
            }
            num -= level_arr_[cnt]*(1<<cnt);
            if (cnt == 0) {
                new_arr_[cnt] = 0;
            } else {
                if (level_arr_[cnt-1] == 3) {
                    new_arr_[cnt] = new_arr_[cnt-1] + (1<<(cnt-1));
                } else {
                    new_arr_[cnt] = new_arr_[cnt-1];
                }
            }
            cnt++;
        }
        if (level_arr_[level_num-2] == 3) {
            new_arr_[level_num-1] = new_arr_[level_num-2] + (1<<(level_num-2));
        } else if (level_arr_[level_num-2] == 2) {
            new_arr_[level_num-1] = new_arr_[level_num-2];
        }

        int start_id = n_samples_;
        for (size_t i = 0; i < level_num; i++) {
            //
            start_id -= level_arr_[i]*(1<<i);

            Level* level = new Level(data_arr_+start_id*K, level_arr_[i], new_arr_[i], i);
            level_vec_.emplace_back(level);
        }
    }

    ~DKDTree2() {
        if (is_allocate_) delete[] data_arr_;
        for(Level* obj : level_vec_) delete obj;
    }

    void insertNode(const vector<ElemType> &point) {
        if (n_dimensions_ != point.size()) {
            cout << "Point dimension wrong!!!" << endl;
            return;
        }
        for(size_t i = 0; i < n_dimensions_; i++) {
            data_arr_[n_samples_*n_dimensions_+i] = point[i];
        }
        n_samples_++;
        int start_id = 0;

        for (int i = level_num_-1; i >= 0; i--) {
            if (level_arr_[i] >= 2) {
                if (i == level_num_-1 && level_arr_[i] == 2) {
                    level_num_++;
                    Level* level_i = new Level(i+1);
                    level_vec_.emplace_back(level_i);
                }
                level_vec_[i+1]->addItem(data_arr_+(start_id+new_arr_[i+1])*n_dimensions_);
                new_arr_[i+1] += 1;

                start_id += level_arr_[i]*(1<<i);

                if (new_arr_[i+1] == 1<<(i+1)) {
                    level_arr_[i] -= 2;
                    level_vec_[i]->destroyTree(OLDEST);
                    level_vec_[i]->destroyTree(OLDER);

                    level_arr_[i+1] += 1;
                    level_vec_[i]->copyTree(OLDEST, OLD);
//                    level_vec_[i]->destroyTree(OLD);
                    // age(i+1)
                    new_arr_[i+1] = 0;
                    level_vec_[i+1]->age();
                }
            } else {
                start_id += level_arr_[i]*(1<<i);
            }

        }
        level_arr_[0] += 1;
        new_arr_[0] = 0;
        level_vec_[0]->addItem(data_arr_+(n_samples_-1)*n_dimensions_);
        level_vec_[0]->age();

    }

    vector<ElemType> NNSearch(const vector<ElemType> &point) {
        vector<ElemType> best_point(K);
        if (level_vec_.empty()) {
            cout << "Level is Empty!!!" << endl;
            return best_point;
        } else {
            for (size_t i = 0; i < level_vec_.size(); i++) {
                best_point = (i == 0) ? level_vec_[i]->NNSearch(point) : getCloserPoint(best_point, level_vec_[i]->NNSearch(point), point);
            }
        }
    }

    void printTree() {
        for (int i = 0; i < level_vec_.size(); i++) {
            level_vec_[i]->printLevel();
        }
    }
    size_t getTreeSize();


private:
    //数据集
    ElemType* data_arr_;
    //样本数
    size_t n_samples_;
    //维数
    size_t n_dimensions_;
    //
    vector<Level*> level_vec_;

    bool is_allocate_;
    int level_arr_[15]={0};
    int new_arr_[15]={0};
    int level_num_;
};
#endif // LAZYREBUILD_H
